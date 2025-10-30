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

#ifndef MGOS_MODBUS_H_
#define MGOS_MODBUS_H_

#include "mgos.h"

#if defined(__cplusplus)
extern "C" {
#endif

// Modbus exception codes
#define EX_ILLEGAL_FUNCTION      0x01  // Function code not allowed
#define EX_ILLEGAL_DATA_ADDRESS  0x02  // Data address not allowed
#define EX_ILLEGAL_DATA_VALUE    0x03  // Data value not allowed
#define EX_SLAVE_DEVICE_FAILURE  0x04  // Unrecoverable error occurred

// Response status codes
#define RESP_SUCCESS             0x00  // Transaction successful
#define RESP_INVALID_SLAVE_ID    0xE0  // Slave ID mismatch
#define RESP_INVALID_FUNCTION    0xE1  // Function code mismatch
#define RESP_TIMED_OUT           0xE2  // Response timeout
#define RESP_INVALID_CRC         0xE3  // CRC check failed

// Modbus function codes for bit access
#define FUNC_READ_COILS                0x01  // Read Coils
#define FUNC_READ_DISCRETE_INPUTS      0x02  // Read Discrete Inputs
#define FUNC_WRITE_SINGLE_COIL         0x05  // Write Single Coil
#define FUNC_WRITE_MULTIPLE_COILS      0x0F  // Write Multiple Coils

// Modbus function codes for 16-bit register access
#define FUNC_READ_HOLDING_REGISTERS          0x03  // Read Holding Registers
#define FUNC_READ_INPUT_REGISTERS            0x04  // Read Input Registers
#define FUNC_WRITE_SINGLE_REGISTER           0x06  // Write Single Register
#define FUNC_WRITE_MULTIPLE_REGISTERS        0x10  // Write Multiple Registers
#define FUNC_MASK_WRITE_REGISTER             0x16  // Mask Write Register
#define FUNC_READ_WRITE_MULTIPLE_REGISTERS   0x17  // Read/Write Multiple Registers

/**
 * Modbus request information structure
 * Contains details about the modbus request for use in callbacks
 */
struct mb_request_info {
    uint8_t slave_id;
    uint16_t read_address;
    uint16_t read_qty;
    uint16_t write_address;
    uint16_t write_qty;
    uint8_t mask_and;
    uint8_t mask_or;
    uint8_t func_code;
};

/**
 * Modbus response callback function type
 * @param status Response status code (RESP_SUCCESS or error code)
 * @param info Request information
 * @param response Response buffer containing modbus data
 * @param param User-defined parameter passed to the request
 */
typedef void (*mb_response_callback)(uint8_t status, struct mb_request_info info, 
                                     struct mbuf response, void* param);

// Modbus read operations
bool mb_read_coils(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                   mb_response_callback cb, void* cb_arg);

bool mb_read_discrete_inputs(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                             mb_response_callback cb, void* cb_arg);

bool mb_read_holding_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                               mb_response_callback cb, void* cb_arg);

bool mb_read_input_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                             mb_response_callback cb, void* cb_arg);

// Modbus write operations
bool mb_write_single_coil(uint8_t slave_id, uint16_t write_address, uint16_t write_value,
                          mb_response_callback cb, void* cb_arg);

bool mb_write_single_register(uint8_t slave_id, uint16_t write_address, uint16_t write_value,
                              mb_response_callback cb, void* cb_arg);

bool mb_write_multiple_coils(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                             uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg);

bool mb_write_multiple_registers(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                                 uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg);

// Modbus advanced operations
bool mb_read_write_multiple_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                                      uint16_t write_address, uint16_t write_qty, uint8_t* data,
                                      uint8_t len, mb_response_callback cb, void* cb_arg);

bool mb_mask_write_register(uint8_t slave_id, uint16_t address, uint16_t and_mask, 
                            uint16_t or_mask, mb_response_callback cb, void* cb_arg);

// Data parsing utilities
/**
 * Parse 32-bit inverse long from modbus response buffer
 * Modbus byte order: BADC
 * @param strt_ptr Pointer to start of 4-byte value in buffer
 * @return Parsed long value
 */
long parse_value_long_inverse_32(uint8_t* strt_ptr);

/**
 * Parse 32-bit float from modbus response buffer
 * Modbus byte order: BADC -> C memory: DCBA
 * @param strt_ptr Pointer to start of 4-byte value in buffer
 * @return Parsed float value
 */
float parse_value_float32(uint8_t* strt_ptr);

/**
 * Map modbus response to JSON using JSON map string
 * 
 * JSON map format:
 * {
 *     "attribute_name": {
 *         "type": "float|long_inv|hex",
 *         "add": <register_address>
 *     }
 * }
 * Or simplified: { "attribute_name": <register_address> }
 * 
 * @param json_map JSON mapping string
 * @param mb_resp Modbus response buffer
 * @param info Request information
 * @return Allocated JSON string (caller must free) or NULL on error
 */
char* mb_map_register_response(const char* json_map, struct mbuf* mb_resp, 
                               struct mb_request_info* info);

/**
 * Map modbus response to JSON using JSON map from file
 * @param json_file Path to JSON map file
 * @param mb_resp Modbus response buffer
 * @param info Request information
 * @return Allocated JSON string (caller must free) or NULL on error
 */
char* mb_map_register_responsef(const char* json_file, struct mbuf* mb_resp, 
                                struct mb_request_info* info);

// Initialization functions
/**
 * Create and configure modbus instance
 * @param cfg Modbus configuration structure
 * @return true on success, false on failure
 */
bool mgos_modbus_create(const struct mgos_config_modbus* cfg);

/**
 * Initialize modbus subsystem
 * Called automatically during system init
 * @return true on success, false on failure
 */
bool mgos_modbus_init(void);

/**
 * Connect/reconnect modbus (reconfigure UART)
 * @return true on success, false on failure
 */
bool mgos_modbus_connect(void);

#ifdef __cplusplus
}
#endif

#endif /* MGOS_MODBUS_H_ */
