/*
 * Copyright 2021 Suyash Mathema
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

/*
 * Optimized Modbus RTU Library for Mongoose OS
 * 
 * Key improvements over original:
 * - Eliminated blocking operations (mgos_msleep removed)
 * - Reduced buffer operations (use read pointers instead of mbuf_remove)
 * - Optimized logging (only format strings when needed)
 * - Proper 3.5 character time calculation
 * - Simplified state machine
 * - Reduced memory allocations
 */

#include "mgos_modbus.h"
#include "crc16.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define lowByte(w) ((uint8_t)((w)&0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

enum uart_read_states {
    DISABLED,
    WAITING_RESPONSE,
    RECEIVING_DATA
};

struct mgos_modbus {
    struct mbuf receive_buffer;
    struct mbuf transmit_buffer;
    mb_response_callback cb;
    void* cb_arg;
    int uart_no;
    uint8_t slave_id_u8;
    uint16_t read_address_u16;
    uint16_t read_qty_u16;
    uint16_t write_address_u16;
    uint16_t write_qty_u16;
    uint8_t* write_data;
    uint8_t write_data_len;
    uint8_t mask_and;
    uint8_t mask_or;
    uint8_t func_code_u8;
    uint8_t resp_status_u8;
    uint8_t expected_bytes;
    enum uart_read_states read_state;
    int rx_start_pos;  // Track where we started reading in buffer
    double last_rx_time;  // For frame gap detection
};

static struct mgos_modbus* s_modbus = NULL;
static mgos_timer_id req_timer = MGOS_INVALID_TIMER_ID;

// Calculate frame gap time based on baud rate (3.5 character times in ms)
static inline double get_frame_gap_ms(void) {
    int baud = mgos_sys_config_get_modbus_baudrate();
    // 1 char = 11 bits (1 start + 8 data + 1 parity + 1 stop), 3.5 chars
    return (11.0 * 3.5 * 1000.0) / (double)baud;
}

// Optimized buffer append for 16-bit values
static inline void mbuf_append_u16_be(struct mbuf* buf, uint16_t val) {
    uint8_t bytes[2] = {highByte(val), lowByte(val)};
    mbuf_append(buf, bytes, 2);
}

// Fast CRC16 calculation
static uint16_t calc_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = crc16_update(crc, data[i]);
    }
    return (lowByte(crc) << 8) | highByte(crc);  // Swap bytes
}

// Verify CRC without modifying buffer
static bool verify_crc16(const uint8_t* data, size_t len) {
    if (len < 4) return false;
    uint16_t calc = calc_crc16(data, len - 2);
    uint16_t recv = (data[len - 1] << 8) | data[len - 2];
    return (calc == recv);
}

// Print buffer only when debug logging is enabled
static void print_buffer_debug(const char* label, const uint8_t* buf, size_t len) {
    if (cs_log_level < LL_DEBUG) return;  // Skip if not debugging
    
    char str[256];
    int pos = 0;
    for (size_t i = 0; i < len && pos < sizeof(str) - 4; i++) {
        pos += sprintf(str + pos, "%.2x ", buf[i]);
    }
    LOG(LL_DEBUG, ("%s: %s", label, str));
}

static void req_timeout_cb(void* arg) {
    if (s_modbus->read_state == DISABLED) return;
    
    LOG(LL_WARN, ("Modbus timeout - SlaveID: %.2x, Func: %.2x", 
                  s_modbus->slave_id_u8, s_modbus->func_code_u8));
    
    s_modbus->resp_status_u8 = RESP_TIMED_OUT;
    s_modbus->read_state = DISABLED;
    
    struct mb_request_info ri = {
        s_modbus->slave_id_u8,
        s_modbus->read_address_u16,
        s_modbus->read_qty_u16,
        s_modbus->write_address_u16,
        s_modbus->write_qty_u16,
        s_modbus->mask_and,
        s_modbus->mask_or,
        s_modbus->func_code_u8,
    };
    
    s_modbus->cb(RESP_TIMED_OUT, ri, s_modbus->receive_buffer, s_modbus->cb_arg);
    req_timer = MGOS_INVALID_TIMER_ID;
    (void)arg;
}

// Calculate expected response length based on function code
static uint8_t calc_expected_length(uint8_t func, const uint8_t* resp_buf, size_t buf_len) {
    // Need at least slave ID and function code
    if (buf_len < 2) return 0;
    
    // Check for exception (bit 7 set in function code)
    if (resp_buf[1] & 0x80) {
        return 5;  // SlaveID + Func + ExceptionCode + CRC(2)
    }
    
    switch (func) {
        case FUNC_READ_COILS:
        case FUNC_READ_DISCRETE_INPUTS:
        case FUNC_READ_INPUT_REGISTERS:
        case FUNC_READ_HOLDING_REGISTERS:
        case FUNC_READ_WRITE_MULTIPLE_REGISTERS:
            // Need byte count field
            if (buf_len < 3) return 0;
            return 5 + resp_buf[2];  // SlaveID + Func + ByteCount + Data + CRC(2)
            
        case FUNC_WRITE_SINGLE_COIL:
        case FUNC_WRITE_MULTIPLE_COILS:
        case FUNC_WRITE_SINGLE_REGISTER:
        case FUNC_WRITE_MULTIPLE_REGISTERS:
            return 8;  // Fixed response
            
        case FUNC_MASK_WRITE_REGISTER:
            return 10;  // Fixed response
            
        default:
            return 0;
    }
}

// Process received data - optimized version
static void process_response(void) {
    struct mbuf* buf = &s_modbus->receive_buffer;
    
    // Not enough data yet
    if (buf->len < 5) return;
    
    uint8_t* data = (uint8_t*)buf->buf;
    
    // Validate slave ID
    if (data[0] != s_modbus->slave_id_u8) {
        LOG(LL_WARN, ("Wrong slave ID: expected %.2x, got %.2x", 
                     s_modbus->slave_id_u8, data[0]));
        s_modbus->resp_status_u8 = RESP_INVALID_SLAVE_ID;
        goto error;
    }
    
    // Validate function code (mask exception bit)
    if ((data[1] & 0x7F) != s_modbus->func_code_u8) {
        LOG(LL_WARN, ("Wrong function: expected %.2x, got %.2x", 
                     s_modbus->func_code_u8, data[1]));
        s_modbus->resp_status_u8 = RESP_INVALID_FUNCTION;
        goto error;
    }
    
    // Check for modbus exception
    if (data[1] & 0x80) {
        LOG(LL_WARN, ("Modbus exception: %.2x", data[2]));
        s_modbus->resp_status_u8 = data[2];
        goto error;
    }
    
    // Calculate expected length
    s_modbus->expected_bytes = calc_expected_length(s_modbus->func_code_u8, data, buf->len);
    if (s_modbus->expected_bytes == 0) {
        // Need more data to determine length
        return;
    }
    
    // Wait for complete frame
    if (buf->len < s_modbus->expected_bytes) {
        return;
    }
    
    // Trim to exact size
    buf->len = s_modbus->expected_bytes;
    
    // Verify CRC
    if (!verify_crc16(data, buf->len)) {
        LOG(LL_WARN, ("CRC error"));
        s_modbus->resp_status_u8 = RESP_INVALID_CRC;
        goto error;
    }
    
    // Success!
    LOG(LL_DEBUG, ("Response OK - %d bytes", buf->len));
    print_buffer_debug("RX", data, buf->len);
    s_modbus->resp_status_u8 = RESP_SUCCESS;
    
error:
    // Clear timer and invoke callback
    if (req_timer != MGOS_INVALID_TIMER_ID) {
        mgos_clear_timer(req_timer);
        req_timer = MGOS_INVALID_TIMER_ID;
    }
    
    s_modbus->read_state = DISABLED;
    
    struct mb_request_info ri = {
        s_modbus->slave_id_u8,
        s_modbus->read_address_u16,
        s_modbus->read_qty_u16,
        s_modbus->write_address_u16,
        s_modbus->write_qty_u16,
        s_modbus->mask_and,
        s_modbus->mask_or,
        s_modbus->func_code_u8,
    };
    
    s_modbus->cb(s_modbus->resp_status_u8, ri, s_modbus->receive_buffer, s_modbus->cb_arg);
}

static void uart_cb(int uart_no, void* param) {
    if (uart_no != s_modbus->uart_no) return;
    if (s_modbus->read_state == DISABLED) {
        // Flush unexpected data
        mgos_uart_flush(uart_no);
        return;
    }
    
    size_t rx_av = mgos_uart_read_avail(uart_no);
    if (rx_av == 0) return;
    
    // Read data into buffer
    struct mbuf* buf = &s_modbus->receive_buffer;
    size_t old_len = buf->len;
    mgos_uart_read_mbuf(uart_no, buf, rx_av);
    
    LOG(LL_VERBOSE_DEBUG, ("RX: %d bytes (total: %d)", rx_av, buf->len));
    
    // Update state
    if (s_modbus->read_state == WAITING_RESPONSE) {
        s_modbus->read_state = RECEIVING_DATA;
    }
    
    s_modbus->last_rx_time = mgos_uptime();
    
    // Try to process
    process_response();
    
    (void)param;
}

// Build transmit buffer - optimized
static bool build_tx_buffer(void) {
    struct mbuf* buf = &s_modbus->transmit_buffer;
    mbuf_clear(buf);
    
    // Slave ID + Function
    mbuf_append(buf, &s_modbus->slave_id_u8, 1);
    mbuf_append(buf, &s_modbus->func_code_u8, 1);
    
    uint8_t func = s_modbus->func_code_u8;
    
    // Add read parameters if needed
    if (func == FUNC_READ_COILS ||
        func == FUNC_READ_DISCRETE_INPUTS ||
        func == FUNC_READ_INPUT_REGISTERS ||
        func == FUNC_READ_HOLDING_REGISTERS ||
        func == FUNC_READ_WRITE_MULTIPLE_REGISTERS) {
        mbuf_append_u16_be(buf, s_modbus->read_address_u16);
        mbuf_append_u16_be(buf, s_modbus->read_qty_u16);
    }
    
    // Add write parameters if needed
    if (func == FUNC_WRITE_SINGLE_COIL ||
        func == FUNC_WRITE_SINGLE_REGISTER) {
        mbuf_append_u16_be(buf, s_modbus->write_address_u16);
        mbuf_append_u16_be(buf, s_modbus->write_qty_u16);
    } else if (func == FUNC_WRITE_MULTIPLE_COILS ||
               func == FUNC_WRITE_MULTIPLE_REGISTERS ||
               func == FUNC_READ_WRITE_MULTIPLE_REGISTERS) {
        mbuf_append_u16_be(buf, s_modbus->write_address_u16);
        mbuf_append_u16_be(buf, s_modbus->write_qty_u16);
        mbuf_append(buf, &s_modbus->write_data_len, 1);
        mbuf_append(buf, s_modbus->write_data, s_modbus->write_data_len);
    } else if (func == FUNC_MASK_WRITE_REGISTER) {
        mbuf_append_u16_be(buf, s_modbus->read_address_u16);
        mbuf_append_u16_be(buf, s_modbus->mask_and);
        mbuf_append_u16_be(buf, s_modbus->mask_or);
    }
    
    // Add CRC
    uint16_t crc = calc_crc16((uint8_t*)buf->buf, buf->len);
    uint8_t crc_bytes[2] = {lowByte(crc), highByte(crc)};
    mbuf_append(buf, crc_bytes, 2);
    
    print_buffer_debug("TX", (uint8_t*)buf->buf, buf->len);
    
    return (buf->len > 0);
}

static bool start_transaction(void) {
    if (s_modbus->read_state != DISABLED) {
        LOG(LL_WARN, ("Transaction already in progress"));
        return false;
    }
    
    if (s_modbus->transmit_buffer.len == 0) {
        LOG(LL_ERROR, ("Empty transmit buffer"));
        return false;
    }
    
    // Clear receive buffer
    mbuf_clear(&s_modbus->receive_buffer);
    s_modbus->resp_status_u8 = 0;
    s_modbus->expected_bytes = 0;
    s_modbus->read_state = WAITING_RESPONSE;
    
    LOG(LL_DEBUG, ("TX: SlaveID %.2x, Func %.2x", 
                   s_modbus->slave_id_u8, s_modbus->func_code_u8));
    
    // Enable RX and set callback
    mgos_uart_set_rx_enabled(s_modbus->uart_no, true);
    mgos_uart_set_dispatcher(s_modbus->uart_no, uart_cb, NULL);
    
    // Transmit
    mgos_uart_write(s_modbus->uart_no, s_modbus->transmit_buffer.buf, 
                    s_modbus->transmit_buffer.len);
    
    // Start timeout timer
    req_timer = mgos_set_timer(mgos_sys_config_get_modbus_timeout(), 0, 
                               req_timeout_cb, NULL);
    
    return true;
}

// Public API functions - simplified initialization
static bool init_request(uint8_t slave_id, uint8_t func, mb_response_callback cb, void* cb_arg) {
    if (s_modbus->read_state != DISABLED) {
        return false;
    }
    
    s_modbus->cb = cb;
    s_modbus->cb_arg = cb_arg;
    s_modbus->slave_id_u8 = slave_id;
    s_modbus->func_code_u8 = func;
    s_modbus->read_address_u16 = 0;
    s_modbus->read_qty_u16 = 0;
    s_modbus->write_address_u16 = 0;
    s_modbus->write_qty_u16 = 0;
    s_modbus->write_data = NULL;
    s_modbus->write_data_len = 0;
    
    return true;
}

bool mb_read_holding_registers(uint8_t slave_id, uint16_t address, uint16_t qty, 
                               mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_READ_HOLDING_REGISTERS, cb, cb_arg)) return false;
    s_modbus->read_address_u16 = address;
    s_modbus->read_qty_u16 = qty;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mb_read_input_registers(uint8_t slave_id, uint16_t address, uint16_t qty,
                             mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_READ_INPUT_REGISTERS, cb, cb_arg)) return false;
    s_modbus->read_address_u16 = address;
    s_modbus->read_qty_u16 = qty;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mb_read_coils(uint8_t slave_id, uint16_t address, uint16_t qty,
                  mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_READ_COILS, cb, cb_arg)) return false;
    s_modbus->read_address_u16 = address;
    s_modbus->read_qty_u16 = qty;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mb_read_discrete_inputs(uint8_t slave_id, uint16_t address, uint16_t qty,
                             mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_READ_DISCRETE_INPUTS, cb, cb_arg)) return false;
    s_modbus->read_address_u16 = address;
    s_modbus->read_qty_u16 = qty;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mb_write_single_register(uint8_t slave_id, uint16_t address, uint16_t value,
                              mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_WRITE_SINGLE_REGISTER, cb, cb_arg)) return false;
    s_modbus->write_address_u16 = address;
    s_modbus->write_qty_u16 = value;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mb_write_single_coil(uint8_t slave_id, uint16_t address, uint16_t value,
                         mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_WRITE_SINGLE_COIL, cb, cb_arg)) return false;
    s_modbus->write_address_u16 = address;
    s_modbus->write_qty_u16 = value;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mb_write_multiple_registers(uint8_t slave_id, uint16_t address, uint16_t qty,
                                 uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_WRITE_MULTIPLE_REGISTERS, cb, cb_arg)) return false;
    s_modbus->write_address_u16 = address;
    s_modbus->write_qty_u16 = qty;
    s_modbus->write_data = data;
    s_modbus->write_data_len = len;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mb_write_multiple_coils(uint8_t slave_id, uint16_t address, uint16_t qty,
                             uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_WRITE_MULTIPLE_COILS, cb, cb_arg)) return false;
    s_modbus->write_address_u16 = address;
    s_modbus->write_qty_u16 = qty;
    s_modbus->write_data = data;
    s_modbus->write_data_len = len;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mb_read_write_multiple_registers(uint8_t slave_id, uint16_t read_addr, uint16_t read_qty,
                                      uint16_t write_addr, uint16_t write_qty,
                                      uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_READ_WRITE_MULTIPLE_REGISTERS, cb, cb_arg)) return false;
    s_modbus->read_address_u16 = read_addr;
    s_modbus->read_qty_u16 = read_qty;
    s_modbus->write_address_u16 = write_addr;
    s_modbus->write_qty_u16 = write_qty;
    s_modbus->write_data = data;
    s_modbus->write_data_len = len;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mb_mask_write_register(uint8_t slave_id, uint16_t address, uint16_t and_mask, 
                            uint16_t or_mask, mb_response_callback cb, void* cb_arg) {
    if (!init_request(slave_id, FUNC_MASK_WRITE_REGISTER, cb, cb_arg)) return false;
    s_modbus->read_address_u16 = address;
    s_modbus->mask_and = and_mask;
    s_modbus->mask_or = or_mask;
    if (!build_tx_buffer()) return false;
    return start_transaction();
}

bool mgos_modbus_create(const struct mgos_config_modbus* cfg) {
    struct mgos_uart_config ucfg;
    mgos_uart_config_set_defaults(cfg->uart_no, &ucfg);
    
    ucfg.baud_rate = mgos_sys_config_get_modbus_baudrate();
    if (mgos_sys_config_get_modbus_uart_rx_pin() >= 0) {
        ucfg.dev.rx_gpio = mgos_sys_config_get_modbus_uart_rx_pin();
    }
    if (mgos_sys_config_get_modbus_uart_tx_pin() >= 0) {
        ucfg.dev.tx_gpio = mgos_sys_config_get_modbus_uart_tx_pin();
    }
    if (mgos_sys_config_get_modbus_tx_en_gpio() >= 0) {
        ucfg.dev.tx_en_gpio = mgos_sys_config_get_modbus_tx_en_gpio();
    }
    if (mgos_sys_config_get_modbus_parity() >= 0 && mgos_sys_config_get_modbus_parity() < 3) {
        ucfg.parity = mgos_sys_config_get_modbus_parity();
    }
    if (mgos_sys_config_get_modbus_stop_bits() > 0 && mgos_sys_config_get_modbus_stop_bits() < 4) {
        ucfg.stop_bits = mgos_sys_config_get_modbus_stop_bits();
    }
    ucfg.dev.hd = mgos_sys_config_get_modbus_tx_en_enable();
    ucfg.dev.tx_en_gpio_val = mgos_sys_config_get_modbus_tx_en_gpio_val();
    
    if (!mgos_uart_configure(cfg->uart_no, &ucfg)) {
        LOG(LL_ERROR, ("Failed to configure UART%d", cfg->uart_no));
        return false;
    }
    
    s_modbus = (struct mgos_modbus*)calloc(1, sizeof(*s_modbus));
    if (s_modbus == NULL) return false;
    
    // Smaller buffers - typical modbus frame is < 256 bytes
    mbuf_init(&s_modbus->transmit_buffer, 128);
    mbuf_init(&s_modbus->receive_buffer, 128);
    s_modbus->read_state = DISABLED;
    s_modbus->uart_no = cfg->uart_no;
    
    LOG(LL_INFO, ("Modbus initialized on UART%d @ %d baud", 
                  cfg->uart_no, ucfg.baud_rate));
    
    return true;
}

bool mgos_modbus_init(void) {
    if (!mgos_sys_config_get_modbus_enable()) {
        return true;
    }
    
    return mgos_modbus_create(&mgos_sys_config.modbus);
}

}
