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
#ifndef __I2C_HID_H__
#define __I2C_HID_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_error.h"
#include "app_error.h"

#include "i2c_hid_descriptors.h"
#include "i2c_hid_internal_types.h"

#define I2C_HID_REGISTER_ADDR_HID_DESCRIPTOR    0x0001
#define I2C_HID_REGISTER_ADDR_REPORT_DESCRIPTOR 0x0002
#define I2C_HID_REGISTER_ADDR_INPUT             0x0003
#define I2C_HID_REGISTER_ADDR_OUTPUT            0x0004
#define I2C_HID_REGISTER_ADDR_COMMAND           0x0005
#define I2C_HID_REGISTER_ADDR_DATA              0x0006

#define I2C_HID_INPUT_REP_QUEUE_LEN 4
#define I2C_HID_EVT_QUEUE_LEN       2

#define I2C_HID_EVT_INT_PRIO_DEFAULT         7 /* Default priority used for event callbacks */

typedef enum
{
    I2C_HID_EVT_OUTPUT_REP,          /* Output Report received from Host */
    I2C_HID_EVT_TYPE_REQ_RESET,      /* RESET Request. Must be replied to using  @ref i2c_hid_req_reset_reply */
    I2C_HID_EVT_TYPE_REQ_SET_POWER,  /* SET_POWER Request. No reply required*/
    I2C_HID_EVT_TYPE_REQ_SET_REPORT, /* SET_REPORT Request. No reply required */
    I2C_HID_EVT_TYPE_REQ_GET_REPORT, /* GET_REPORT Request. Must be replied to using @ref TODO */
} i2c_hid_evt_type_t;

typedef enum
{
    I2C_HID_REPORT_TYPE_INPUT,
    I2C_HID_REPORT_TYPE_OUTPUT,
    I2C_HID_REPORT_TYPE_FEATURE,
} i2c_hid_report_type_t;

typedef enum
{
    I2C_HID_SET_POWER_ON,
    I2C_HID_SET_POWER_SLEEP
} i2c_hid_set_power_t;

typedef struct
{
    uint8_t               id;
    i2c_hid_report_type_t type;
    uint8_t               rep_value[I2C_HID_REPORT_MAX_SIZE];
    uint16_t              rep_length;
} i2c_hid_report_t;

typedef struct
{
    i2c_hid_evt_type_t type;

    union
    {
        i2c_hid_report_t    report;
        i2c_hid_set_power_t power;
    };
} i2c_hid_evt_t;

typedef void (*i2c_hid_evt_handler_t)(i2c_hid_evt_t const * p_evt);

/**@brief Initialize I2C HID functionality
 *
 * @param[in] evt_handler Event callback function
 *
 * @ret NRF_SUCCESS
 * @ret NRF_INVALID_PARAM
 * @ret NRF_ERROR_NOT_SUPPORTED Invalid HID Descriptor configuration
 * @ret NRF_ERROR_              Return from TWIS driver
 */
ret_code_t i2c_hid_init(i2c_hid_evt_handler_t evt_handler);

/**@brief Respond to a RESET request from the Host. Should only be called after I2C_HID_EVT_TYPE_REQ_RESET event (this is not fully enforced)
 *
 * @note If application performs a chip reset, it should ensure that this function is called right after @ref i2c_hid_init, before trying to send Input Reports.
 *
 * @ret NRF_SUCCESS
 * @ret NRF_ERROR_INVALID_STATE If another command is pending
 */
ret_code_t i2c_hid_req_reset_reply(void);

/**@brief Respond to a GET_REPORT request from the Host. Should only be called after I2C_HID_EVT_TYPE_REQ_GET_REPORT event
 *
 * @note This function should be called within 10 milliseconds after receiving I2C_HID_EVT_TYPE_REQ_GET_REPORT, as this is the maximum allowed I2C clock stretch
 *
 * @param[in] p_rep_value Report value
 * @param[in] length      Report length in bytes
 * @param[in] rep_id      Report ID
 *
 * @ret NRF_SUCCESS
 * @ret NRF_ERROR_INVALID_STATE GET_REPORT request not received from Host
 */
ret_code_t i2c_hid_get_report_reply(uint8_t * p_rep_value, uint16_t length, uint8_t rep_id);

/**@brief Send Input Report.
 *
 * @param[in] p_rep_value Report value
 * @param[in] length      Report length in bytes
 * @param[in] rep_id      Report ID
 *
 * @ret NRF_SUCCESS
 * @ret NRF_ERROR_NO_MEM        Input Report queue is full
 * @ret NRF_ERROR_INVALID_STATE Awaiting reset reply (@ref i2c_hid_req_reset_reply) in response to RESET request from Host
 */
ret_code_t i2c_hid_input_report_send(uint8_t * p_rep_value, uint16_t length, uint8_t rep_id);

#endif /* __I2C_HID_H__ */