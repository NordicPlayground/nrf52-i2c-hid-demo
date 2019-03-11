/**
 * Copyright (c) 2009 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/
#ifndef __I2C_HID_INTERNAL_TYPES_H__
#define __I2C_HID_INTERNAL_TYPES_H__

#include <stdint.h>

#include "app_util_platform.h"
#include "i2c_hid_descriptors.h"

#define I2C_HID_REGISTER_LIST \
    X(HID_DESCRIPTOR,    i2c_hid_reg_write_hid_descriptor)    \
    X(REPORT_DESCRIPTOR, i2c_hid_reg_write_report_descriptor) \
    X(INPUT,             i2c_hid_reg_write_input)             \
    X(OUTPUT,            i2c_hid_reg_write_output)            \
    X(COMMAND,           i2c_hid_reg_write_command)           \
    X(DATA,              i2c_hid_reg_write_data)

#define I2C_HID_REQ_LIST \
    X(RESET,      0b0001, i2c_hid_req_reset)      \
    X(GET_REPORT, 0b0010, i2c_hid_req_get_report) \
    X(SET_REPORT, 0b0011, i2c_hid_req_set_report) \
    X(SET_POWER,  0b1000, i2c_hid_req_set_power)

#define I2C_HID_REGISTER_INVALID I2C_HID_REGISTER_COUNT

typedef enum
{
#define X(_name, _func) CONCAT_2(DRV_I2C_HID_REG_, _name),
    I2C_HID_REGISTER_LIST
#undef X
    I2C_HID_REGISTER_COUNT
} i2c_hid_reg_t;

typedef enum
{
#define X(_name, __opcode, _func) CONCAT_2(DRV_I2C_HID_REQ_, _name),
    I2C_HID_REQ_LIST
#undef X
    I2C_HID_REQ_COUNT
} i2c_hid_req_type_t;

typedef enum
{
#define X(_rep_id, _rep_length) CONCAT_2(_I2C_HID_INTERNAL_INP_REP_NAME_, _rep_id),
    I2C_HID_INPUT_REPORT_LIST
#undef X
    I2C_HID_INPUT_REPORT_COUNT,
} i2c_hid_internal_inp_rep_name_t;

typedef union
{
#define X(_rep_id, _rep_length) uint8_t CONCAT_2(_rep, _rep_id)[_rep_length];
    I2C_HID_INPUT_REPORT_LIST
#undef X
} i2c_hid_internal_inp_rep_t;
#define I2C_HID_MAX_SIZE_INPUT_REPORT sizeof(i2c_hid_internal_inp_rep_t)

typedef enum
{
#define X(_rep_id, _rep_length) CONCAT_2(_I2C_HID_INTERNAL_OUTP_REP_NAME, _rep_id),
    I2C_HID_OUTPUT_REPORT_LIST
#undef X
    I2C_HID_OUTPUT_REPORT_COUNT
} i2c_hid_internal_outp_rep_name_t;

typedef struct
{
    union
    {
#define X(_rep_id, _rep_length) uint8_t CONCAT_2(_rep, _rep_id)[_rep_length];
        I2C_HID_OUTPUT_REPORT_LIST
#undef X
    };
} i2c_hid_internal_outp_rep_t;
#define I2C_HID_MAX_SIZE_OUTPUT_REPORT sizeof(i2c_hid_internal_outp_rep_t)

typedef enum
{
#define X(_rep_id, _rep_length) CONCAT_2(_I2C_HID_INTERNAL_FEATURE_REP_NAME, _rep_id),
    I2C_HID_FEATURE_REPORT_LIST
#undef X
    I2C_HID_FEATURE_REPORT_COUNT
} i2c_hid_internal_feat_rep_name_t;

typedef struct
{
    union
    {
#define X(_rep_id, _rep_length) uint8_t CONCAT_2(_rep, _rep_id)[_rep_length];
        I2C_HID_FEATURE_REPORT_LIST
#undef X
    };
} i2c_hid_internal_feat_rep_t;
#define I2C_HID_MAX_SIZE_FEATURE_REPORT sizeof(i2c_hid_internal_feat_rep_t)

#define I2C_HID_REPORT_MAX_SIZE MAX(I2C_HID_MAX_SIZE_INPUT_REPORT, MAX(I2C_HID_MAX_SIZE_OUTPUT_REPORT, I2C_HID_MAX_SIZE_FEATURE_REPORT))

typedef enum
{
    I2C_HID_CMD_TYPE_NONE = 0,
    I2C_HID_CMD_TYPE_GET_HID_DESCRIPTOR,
    I2C_HID_CMD_TYPE_GET_REPORT_DESCRIPTOR,
    I2C_HID_CMD_TYPE_GET_REPORT,
    I2C_HID_CMD_TYPE_RESET,
} i2c_hid_internal_cmd_type_t;

typedef PACKED_STRUCT
{
    uint8_t wHIDDescLength[2];
    uint8_t bcdVersion[2];
    uint8_t wReportDescLength[2];
    uint8_t wReportDescRegister[2];
    uint8_t wInputRegister[2];
    uint8_t wMaxInputLength[2];
    uint8_t wOutputRegister[2];
    uint8_t wMaxOutputLength[2];
    uint8_t wCommandRegister[2];
    uint8_t wDataRegister[2];
    uint8_t wVendorID[2];
    uint8_t wProductID[2];
    uint8_t wVersionID[2];
    uint8_t RESERVED[4];
} i2c_hid_descriptor_t;

typedef PACKED_STRUCT
{
    uint8_t IGNORE1 : 8; // Ignored when validating opcode
    uint8_t opcode : 4;
    uint8_t IGNORE2 : 4; // Ignored when validating opcode
} i2c_hid_req_opcode_filter_t;

typedef PACKED_STRUCT
{
    uint8_t report_id : 4;
    uint8_t report_type : 2;
    uint8_t RESERVED1 : 2; // Should always be 0
    uint8_t opcode : 4;
    uint8_t RESERVED2 : 4; // Should always be 0
} i2c_hid_reset_req_t;

typedef PACKED_STRUCT
{
    uint8_t power_state : 2;
    uint8_t RESERVED1 : 6; // Should always be 0
    uint8_t opcode : 4;
    uint8_t RESERVED2 : 4; // Should always be 0
} i2c_hid_set_power_req_t;

typedef PACKED_STRUCT
{
    // uint8_t report_id; // Third Byte not yet supported
    uint8_t report_id : 4;
    uint8_t report_type : 2;
    uint8_t RESERVED1 : 2; // Should always be 0
    uint8_t opcode : 4;
    uint8_t RESERVED2 : 4; // Should always be 0
} i2c_hid_set_report_req_t;

typedef PACKED_STRUCT
{
    // uint8_t report_id; // Third Byte not yet supported
    uint8_t report_id : 4;
    uint8_t report_type : 2;
    uint8_t RESERVED1 : 2; // Should always be 0
    uint8_t opcode : 4;
    uint8_t RESERVED2 : 4; // Should always be 0
} i2c_hid_get_report_req_t;

typedef struct
{
    size_t  size; // Size given to TWI driver
    uint8_t value[I2C_HID_MAX_SIZE_INPUT_REPORT + 3]; // 2-byte size + 1-byte report ID + HID Report
} i2c_hid_input_report_msg_t;

typedef void (*i2c_hid_reg_write_func_t)(uint8_t const * p_val, size_t len);
typedef struct
{
    i2c_hid_reg_t            type;
    uint16_t                 addr;
    i2c_hid_reg_write_func_t process_func;
} i2c_hid_reg_mapping_t;

typedef struct
{
    uint8_t                  opcode : 4;
    i2c_hid_reg_write_func_t process_func;
} i2c_hid_req_mapping_t;

#endif /* __I2C_HID_INTERNAL_TYPES_H__ */