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
#include "i2c_hid.h"

#include "app_util_platform.h"
#include "drv_i2c_hid.h"
#include "nrf_atomic.h"
#include "nrf_balloc.h"
#include "nrf_delay.h"
#include "nrf_queue.h"
#include "nrfx_swi.h"

#include "i2c_hid_descriptors.h"
#include "i2c_hid_internal_types.h"

#define NRF_LOG_MODULE_NAME i2c_hid
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#ifndef I2C_HID_EVT_INT_PRIO
#define I2C_HID_EVT_INT_PRIO I2C_HID_EVT_INT_PRIO_DEFAULT
#endif

#ifndef I2C_HID_INT_ALWAYS_EDGE_TRIGGER
#define I2C_HID_INT_ALWAYS_EDGE_TRIGGER I2C_HID_INT_ALWAYS_EDGE_TRIGGER_DEFAULT
#endif

#ifndef I2C_HID_INT_REASSERT_DELAY_US
#define I2C_HID_INT_REASSERT_DELAY_US I2C_HID_INT_REASSERT_DELAY_US_DEFAULT
#endif

const static uint8_t m_hid_report_descriptor[] = {I2C_HID_REPORT_DESCRIPTOR};

STATIC_ASSERT(sizeof(m_hid_report_descriptor) <= 255); // Maximum size of TWIS DMA buffer
STATIC_ASSERT(sizeof(i2c_hid_descriptor_t) == 0x1E);
STATIC_ASSERT(I2C_HID_MAX_SIZE_INPUT_REPORT >= 2); // I2C_HID_MAX_SIZE_INPUT_REPORT must at least be of size 2 to hold reset command response value

#if I2C_HID_INPUT_REP_QUEUE_LEN < 1
#error I2C_HID_INPUT_REP_QUEUE_LEN must have room for at least one report.
#endif

static void i2c_hid_reg_write_hid_descriptor(uint8_t const * p_val, size_t len);
static void i2c_hid_reg_write_report_descriptor(uint8_t const * p_val, size_t len);
static void i2c_hid_reg_write_input(uint8_t const * p_val, size_t len);
static void i2c_hid_reg_write_output(uint8_t const * p_val, size_t len);
static void i2c_hid_reg_write_command(uint8_t const * p_val, size_t len);
static void i2c_hid_reg_write_data(uint8_t const * p_val, size_t len);

static void i2c_hid_req_reset(uint8_t const * p_val, size_t len);
static void i2c_hid_req_get_report(uint8_t const * p_val, size_t len);
static void i2c_hid_req_set_report(uint8_t const * p_val, size_t len);
static void i2c_hid_req_set_power(uint8_t const * p_val, size_t len);

const static i2c_hid_reg_mapping_t m_reg_mappings[I2C_HID_REGISTER_COUNT] =
{
#define X(_name, _func) {.type = CONCAT_2(DRV_I2C_HID_REG_, _name), .addr = CONCAT_2(I2C_HID_REGISTER_ADDR_, _name), .process_func = _func},
    I2C_HID_REGISTER_LIST
#undef X
};

const static i2c_hid_req_mapping_t m_req_mappings[I2C_HID_REQ_COUNT] =
{
#define X(_name, __opcode, _func) {.opcode = __opcode, .process_func = _func},
    I2C_HID_REQ_LIST
#undef X
};

NRF_BALLOC_DEF(m_input_rep_pool, sizeof(i2c_hid_input_report_msg_t), I2C_HID_INPUT_REP_QUEUE_LEN);
NRF_BALLOC_DEF(m_evt_pool, sizeof(i2c_hid_evt_t), I2C_HID_EVT_QUEUE_LEN);
NRF_QUEUE_DEF(i2c_hid_input_report_msg_t*, m_input_rep_queue, I2C_HID_INPUT_REP_QUEUE_LEN, NRF_QUEUE_MODE_NO_OVERFLOW);
NRF_QUEUE_DEF(i2c_hid_evt_t*, m_evt_queue, I2C_HID_EVT_QUEUE_LEN, NRF_QUEUE_MODE_NO_OVERFLOW);

static i2c_hid_evt_handler_t m_evt_handler;
static nrfx_swi_t            m_swi_inst;
static nrf_atomic_u32_t      m_pending_cmd;
static nrf_atomic_flag_t     m_cmd_reply_buf_set;
static nrf_atomic_flag_t     m_i2c_read_buf_waiting;

// Generic buffer used for incoming writes and all outgoing reads except Input Reports
static uint8_t m_i2c_buf_generic[MAX(sizeof(m_hid_report_descriptor), sizeof(i2c_hid_descriptor_t))]; // TODO: Verify that size is also large enough for Requests. Usually Report Descriptor is a lot larger

void drv_i2c_hid_evt_handler(drv_i2c_hid_evt_t const * p_evt)
{
    switch (p_evt->type)
    {
        case DRV_I2C_HID_EVT_TYPE_WRITE_BUF_REQ:
        {
            // Provide buffer large enough for write procedures. Maximum length is normally output/feature report length + overhead
            drv_i2c_hid_write_buf_set(m_i2c_buf_generic, sizeof(m_i2c_buf_generic));
        } break;

        case DRV_I2C_HID_EVT_TYPE_READ_BUF_REQ:
        {
            if (m_pending_cmd != I2C_HID_CMD_TYPE_NONE && m_pending_cmd != I2C_HID_CMD_TYPE_RESET)
            {
                if (nrf_atomic_flag_clear_fetch(&m_cmd_reply_buf_set) != 0)
                {
                    // Buffer is already prepared for reply
                    drv_i2c_hid_read_buf_set(m_i2c_buf_generic, sizeof(m_i2c_buf_generic));
                }
                else
                {
                    nrf_atomic_flag_set(&m_i2c_read_buf_waiting);
                }
            }
            else if (drv_i2c_hid_interrupt_asserted())
            {
                i2c_hid_input_report_msg_t * p_input_rep;

                // Interrupt is only asserted when an Input Reports is to be sent
                APP_ERROR_CHECK_BOOL(!nrf_queue_is_empty(&m_input_rep_queue));

                APP_ERROR_CHECK(nrf_queue_peek(&m_input_rep_queue, &p_input_rep));

                drv_i2c_hid_read_buf_set(p_input_rep->value, sizeof(p_input_rep->value));
            }
            else
            {
                NRF_LOG_WARNING("Unexpected read"); // Appears to happen during module init on host side to see if I2C slave ACKs
                drv_i2c_hid_read_buf_set(m_i2c_buf_generic, sizeof(m_i2c_buf_generic));
            }
        } break;

        case DRV_I2C_HID_EVT_TYPE_READ_DONE:
        {
            if (drv_i2c_hid_interrupt_asserted() && m_pending_cmd == I2C_HID_CMD_TYPE_NONE)
            {
                i2c_hid_input_report_msg_t * p_input_rep;

                if (m_pending_cmd == I2C_HID_CMD_TYPE_RESET)
                {
                    APP_ERROR_CHECK_BOOL(nrf_atomic_u32_fetch_store(&m_pending_cmd, I2C_HID_CMD_TYPE_NONE) == I2C_HID_CMD_TYPE_RESET);
                    NRF_LOG_DEBUG("Reset reply sent");
                }

                APP_ERROR_CHECK(nrf_queue_pop(&m_input_rep_queue, &p_input_rep));
                nrf_balloc_free(&m_input_rep_pool, p_input_rep);

                if (nrf_queue_is_empty(&m_input_rep_queue))
                {
                    drv_i2c_hid_interrupt_deassert();
                }
#if I2C_HID_INT_ALWAYS_EDGE_TRIGGER
                else
                {
                    drv_i2c_hid_interrupt_deassert();
                    nrf_delay_us(I2C_HID_INT_REASSERT_DELAY_US);
                    drv_i2c_hid_interrupt_assert();
                }
#endif /* I2C_HID_INT_ALWAYS_EDGE_TRIGGER */
            }
            else if (nrf_atomic_u32_fetch_store(&m_pending_cmd, I2C_HID_CMD_TYPE_NONE) != I2C_HID_CMD_TYPE_NONE)
            {
                NRF_LOG_DEBUG("Command reply sent");
            }
        } break;

        case DRV_I2C_HID_EVT_TYPE_WRITE_DONE:
        {
            const i2c_hid_reg_mapping_t * p_reg;
            uint16_t reg_addr;

            APP_ERROR_CHECK_BOOL(p_evt->data.rw_done.p_data != NULL);

            if (p_evt->data.rw_done.len < 2)
            {
                NRF_LOG_WARNING("Unexpected I2C write length: %d", p_evt->data.rw_done.len);
                break;
            }

            reg_addr = uint16_decode((const uint8_t *)p_evt->data.rw_done.p_data);
            p_reg    = NULL;
            for (int i = 0; i < ARRAY_SIZE(m_reg_mappings); ++i)
            {
                if (m_reg_mappings[i].addr == reg_addr)
                {
                    p_reg = &m_reg_mappings[i];
                    break;
                }
            }
            if (p_reg != NULL)
            {
                p_reg->process_func(&p_evt->data.rw_done.p_data[2], p_evt->data.rw_done.len - 2);
            }
            else
            {
                NRF_LOG_WARNING("Invalid address: 0x%02X", reg_addr)
                break;
            }
        } break;

        case DRV_I2C_HID_EVT_TYPE_ERROR:
        {
            APP_ERROR_CHECK(p_evt->data.error.err_code);
        } break;

        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

static const i2c_hid_req_mapping_t * i2c_hid_validate_req_opcode(uint8_t const * p_val)
{
    const i2c_hid_req_mapping_t * p_ret;
    i2c_hid_req_opcode_filter_t const * p_opcode_filter;

    p_opcode_filter = (i2c_hid_req_opcode_filter_t const *) p_val;

    p_ret = NULL;
    for(int i = 0; i < ARRAY_SIZE(m_req_mappings); ++i)
    {
        if (m_req_mappings[i].opcode == p_opcode_filter->opcode)
        {
            p_ret = &m_req_mappings[i];
            break;
        }
    }

    return p_ret;
}

static ret_code_t i2c_hid_evt_issue(
    i2c_hid_evt_type_t    evt_type,
    uint8_t               rep_id,
    i2c_hid_report_type_t rep_type,
    uint8_t const       * p_rep_value,
    size_t                rep_length,
    i2c_hid_set_power_t   power)
{
    i2c_hid_evt_t * p_evt;

    p_evt = nrf_balloc_alloc(&m_evt_pool);
    if (p_evt == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    p_evt->type = evt_type;
    switch (evt_type)
    {
        case I2C_HID_EVT_OUTPUT_REP:
            /* Fall through */
        case I2C_HID_EVT_TYPE_REQ_SET_REPORT:
            p_evt->report.id         = rep_id;
            p_evt->report.type       = rep_type;
            p_evt->report.rep_length = rep_length;
            memcpy(p_evt->report.rep_value, p_rep_value, rep_length);
            break;
        case I2C_HID_EVT_TYPE_REQ_GET_REPORT:
            p_evt->report.id         = rep_id;
            p_evt->report.type       = rep_type;
            p_evt->report.rep_length = 0;
            break;
        case I2C_HID_EVT_TYPE_REQ_SET_POWER:
            p_evt->power             = power;
            break;
        default:
            break;
    }

    APP_ERROR_CHECK(nrf_queue_push(&m_evt_queue, &p_evt));
    nrfx_swi_trigger(m_swi_inst, 0);

    return NRF_SUCCESS;
}

static i2c_hid_internal_cmd_type_t i2c_hid_cmd_fetch_set_pending(i2c_hid_internal_cmd_type_t cmd)
{
    i2c_hid_internal_cmd_type_t pending_cmd;

    pending_cmd = nrf_atomic_u32_fetch_store(&m_pending_cmd, cmd);
    if (pending_cmd != I2C_HID_CMD_TYPE_NONE)
    {
        NRF_LOG_WARNING("Command already pending: %d (%d)", cmd, pending_cmd);
    }

    if(cmd != I2C_HID_CMD_TYPE_NONE)
    {
        drv_i2c_hid_interrupt_deassert();
    }

    return pending_cmd;
}

static void i2c_hid_reg_write_hid_descriptor(uint8_t const * p_val, size_t len)
{
    // I2C write to 'HID Desciptor' register
    i2c_hid_descriptor_t * p_hid_desc;

    i2c_hid_cmd_fetch_set_pending(I2C_HID_CMD_TYPE_GET_HID_DESCRIPTOR);

    NRF_LOG_DEBUG("i2c_hid_reg_write_hid_descriptor");

    p_hid_desc = (i2c_hid_descriptor_t *) m_i2c_buf_generic;
    uint16_encode(sizeof(i2c_hid_descriptor_t), p_hid_desc->wHIDDescLength);
    uint16_encode(0x0100, p_hid_desc->bcdVersion);
    uint16_encode(sizeof(m_hid_report_descriptor), p_hid_desc->wReportDescLength);
    uint16_encode(I2C_HID_REGISTER_ADDR_REPORT_DESCRIPTOR, p_hid_desc->wReportDescRegister);
    uint16_encode(I2C_HID_REGISTER_ADDR_INPUT, p_hid_desc->wInputRegister);
    uint16_encode(I2C_HID_MAX_SIZE_INPUT_REPORT + 3, p_hid_desc->wMaxInputLength);
    uint16_encode(I2C_HID_REGISTER_ADDR_OUTPUT, p_hid_desc->wOutputRegister);
    uint16_encode(MAX(I2C_HID_MAX_SIZE_OUTPUT_REPORT, I2C_HID_MAX_SIZE_FEATURE_REPORT), p_hid_desc->wMaxOutputLength);
    uint16_encode(I2C_HID_REGISTER_ADDR_COMMAND, p_hid_desc->wCommandRegister);
    uint16_encode(I2C_HID_REGISTER_ADDR_DATA, p_hid_desc->wDataRegister);
    uint16_encode(I2C_HID_VENDOR_ID, p_hid_desc->wVendorID);
    uint16_encode(I2C_HID_PRODUCT_ID, p_hid_desc->wProductID);
    uint16_encode(I2C_HID_VERSION_ID, p_hid_desc->wVersionID);
    uint32_encode(0x00000000, p_hid_desc->RESERVED);

    nrf_atomic_flag_set(&m_cmd_reply_buf_set);
}

static void i2c_hid_reg_write_report_descriptor(uint8_t const * p_val, size_t len)
{
    // I2C write to 'Report Desciptor' register
    NRF_LOG_DEBUG("i2c_hid_reg_write_report_descriptor");

    i2c_hid_cmd_fetch_set_pending(I2C_HID_CMD_TYPE_GET_REPORT_DESCRIPTOR);
    memcpy(m_i2c_buf_generic, m_hid_report_descriptor, sizeof(m_hid_report_descriptor));
    nrf_atomic_flag_set(&m_cmd_reply_buf_set);
}

static void i2c_hid_reg_write_input(uint8_t const * p_val, size_t len)
{
    // I2C write to 'Input' register: Currently handled in other command parsing functions
    NRF_LOG_DEBUG("i2c_hid_reg_write_input");
}

static void i2c_hid_reg_write_output(uint8_t const * p_val, size_t len)
{
    uint8_t  rep_id;
    uint16_t rep_length;

    // I2C write to 'Output' register
    NRF_LOG_DEBUG("i2c_hid_reg_write_output");

    if (len < 4)
    {
        NRF_LOG_WARNING("Received Output Report of invalid length: %d", len);
        return;
    }

    rep_length = uint16_decode(&p_val[0]);
    rep_id     = p_val[2];

    APP_ERROR_CHECK(i2c_hid_evt_issue(I2C_HID_EVT_OUTPUT_REP, rep_id, I2C_HID_REPORT_TYPE_OUTPUT, &p_val[3], (rep_length - 3), 0));
}

static void i2c_hid_reg_write_command(uint8_t const * p_val, size_t len)
{
    // I2C write to 'Command' register
    const i2c_hid_req_mapping_t * p_req_mapping;

    NRF_LOG_DEBUG("i2c_hid_reg_write_command");
    APP_ERROR_CHECK_BOOL(len >= 2);

    p_req_mapping = i2c_hid_validate_req_opcode(p_val);

    if (p_req_mapping != NULL)
    {
        p_req_mapping->process_func(p_val, len);
    }
    else
    {
        NRF_LOG_WARNING("No mapping found for command (hex): %02X:%02X", p_val[0], p_val[1]);
    }
}

static void i2c_hid_reg_write_data(uint8_t const * p_val, size_t len)
{
    // I2C write to select 'Data' register: Currently handled in other command parsing functions
    NRF_LOG_DEBUG("i2c_hid_reg_write_data");
}

static void i2c_hid_req_reset(uint8_t const * p_val, size_t len)
{
    i2c_hid_reset_req_t * p_reset_req;

    NRF_LOG_DEBUG("i2c_hid_req_reset");

    p_reset_req = (i2c_hid_reset_req_t *) p_val;
    if (p_reset_req->RESERVED1 != 0 || p_reset_req->RESERVED2 != 0)
    {
        NRF_LOG_WARNING("Unexpected format of RESET command. Resetting anyway");
    }

    i2c_hid_cmd_fetch_set_pending(I2C_HID_CMD_TYPE_RESET);

    // Clear Input Report queue. Host expects "Reset" report in response
    nrf_queue_reset(&m_input_rep_queue);
    nrf_balloc_init(&m_input_rep_pool);

    drv_i2c_hid_interrupt_deassert();

    APP_ERROR_CHECK(i2c_hid_evt_issue(I2C_HID_EVT_TYPE_REQ_RESET, 0, 0, 0, 0, 0));
}

static void i2c_hid_req_get_report(uint8_t const * p_val, size_t len)
{
    i2c_hid_get_report_req_t * p_get_report_req;
    i2c_hid_report_type_t      rep_type;
    uint16_t                   data_reg_addr;

    NRF_LOG_DEBUG("i2c_hid_req_get_report");

    if (len < 4)
    {
        NRF_LOG_WARNING("Unexpected length of GET_REPORT request.");
        return;
    }

    p_get_report_req = (i2c_hid_get_report_req_t *) p_val;
    if (p_get_report_req->RESERVED1 != 0 || p_get_report_req->RESERVED2 != 0)
    {
        NRF_LOG_WARNING("Unexpected format of GET_REPORT request");
        return;
    }

    switch (p_get_report_req->report_type)
    {
        case 0b10:
            rep_type = I2C_HID_REPORT_TYPE_OUTPUT;
            break;
        case 0b11:
            rep_type = I2C_HID_REPORT_TYPE_FEATURE;
            break;
        default:
            NRF_LOG_WARNING("Unexpected report type: 0x%02X", p_get_report_req->report_type);
            return;
    }

    data_reg_addr = uint16_decode(&p_val[sizeof(i2c_hid_get_report_req_t)]);
    if (data_reg_addr != I2C_HID_REGISTER_ADDR_DATA)
    {
        NRF_LOG_WARNING("Unexpected data register address: 0x%04X", data_reg_addr);
        return;
    }

    i2c_hid_cmd_fetch_set_pending(I2C_HID_CMD_TYPE_GET_REPORT);

    APP_ERROR_CHECK(i2c_hid_evt_issue(I2C_HID_EVT_TYPE_REQ_GET_REPORT, p_get_report_req->report_id, rep_type, 0, 0, 0));
}

static void i2c_hid_req_set_report(uint8_t const * p_val, size_t len)
{
    i2c_hid_set_report_req_t * p_set_report_req;
    i2c_hid_report_type_t      rep_type;
    uint16_t                   data_reg_addr;
    uint16_t                   rep_length;
    uint8_t                    rep_id;

    NRF_LOG_DEBUG("i2c_hid_req_set_report");

    if (len < 6)
    {
        NRF_LOG_WARNING("Unexpected length of SET_REPORT request.");
        return;
    }

    p_set_report_req = (i2c_hid_set_report_req_t *) p_val;
    if (p_set_report_req->RESERVED1 != 0 || p_set_report_req->RESERVED2 != 0)
    {
        NRF_LOG_WARNING("Unexpected format of SET_REPORT request");
        return;
    }

    switch (p_set_report_req->report_type)
    {
        case 0b10:
            rep_type = I2C_HID_REPORT_TYPE_OUTPUT;
            break;
        case 0b11:
            rep_type = I2C_HID_REPORT_TYPE_FEATURE;
            break;
        default:
            NRF_LOG_WARNING("Unexpected report type: 0x%02X", p_set_report_req->report_type);
            return;
    }

    data_reg_addr = uint16_decode(&p_val[sizeof(i2c_hid_set_report_req_t)]);
    if (data_reg_addr != I2C_HID_REGISTER_ADDR_DATA)
    {
        NRF_LOG_WARNING("Unexpected data register address: 0x%04X", data_reg_addr);
        return;
    }

    rep_length = uint16_decode(&p_val[sizeof(i2c_hid_set_report_req_t) + sizeof(data_reg_addr)]);
    if (rep_length != (len - (sizeof(i2c_hid_set_report_req_t) + sizeof(data_reg_addr))))
    {
        NRF_LOG_WARNING("Unexpected report length: 0x%04X", rep_length);
        return;
    }

    rep_id = p_val[sizeof(i2c_hid_set_report_req_t) + sizeof(data_reg_addr) + sizeof(rep_length)];
    if (rep_id != p_set_report_req->report_id)
    {
        NRF_LOG_WARNING("Descriptor without Report ID not yet supported");
        return;
    }

    rep_length -= (sizeof(rep_length) + sizeof(rep_id));

    APP_ERROR_CHECK(
        i2c_hid_evt_issue(I2C_HID_EVT_TYPE_REQ_SET_REPORT,
                          p_set_report_req->report_id,
                          rep_type,
                          &p_val[sizeof(i2c_hid_set_report_req_t) + sizeof(data_reg_addr) + sizeof(rep_length) + sizeof(rep_id)],
                          rep_length,
                          0));
}

static void i2c_hid_req_set_power(uint8_t const * p_val, size_t len)
{
    i2c_hid_set_power_req_t * p_power_req;
    i2c_hid_set_power_t       power_state;

    NRF_LOG_DEBUG("i2c_hid_req_set_power");

    APP_ERROR_CHECK_BOOL(len == 2);

    p_power_req = (i2c_hid_set_power_req_t *) p_val;
    switch (p_power_req->power_state)
    {
        case 0x00:
            power_state = I2C_HID_SET_POWER_ON;
            break;

        case 0x01:
            power_state = I2C_HID_SET_POWER_SLEEP;
            break;

        default:
            NRF_LOG_WARNING("Unexpected power state: 0x%02X", p_power_req->power_state);
            return;
    }

    APP_ERROR_CHECK(i2c_hid_evt_issue(I2C_HID_EVT_TYPE_REQ_SET_POWER, 0, 0, 0, 0, power_state));
}

static void swi_irqhandler(nrfx_swi_t swi, nrfx_swi_flags_t flags)
{
    do
    {
        i2c_hid_evt_t * p_evt;

        APP_ERROR_CHECK(nrf_queue_pop(&m_evt_queue, &p_evt));
        m_evt_handler(p_evt);
        nrf_balloc_free(&m_evt_pool, p_evt);

    } while (!nrf_queue_is_empty(&m_evt_queue));
}

ret_code_t i2c_hid_validate_report_ids(void)
{
    uint8_t input_rep_ids[] =
    {
#define X(_rep_id, _rep_length) _rep_id,
        I2C_HID_INPUT_REPORT_LIST
#undef X
    };

    uint8_t output_rep_ids[] =
    {
#define X(_rep_id, _rep_length) _rep_id,
        I2C_HID_OUTPUT_REPORT_LIST
#undef X
    };

    uint8_t feature_rep_ids[] =
    {
#define X(_rep_id, _rep_length) _rep_id,
        I2C_HID_FEATURE_REPORT_LIST
#undef X
    };

    uint8_t * p_rep_id_lists[3] = {input_rep_ids, output_rep_ids, feature_rep_ids};
    uint8_t   rep_list_lens[3]  = {ARRAY_SIZE(input_rep_ids), ARRAY_SIZE(output_rep_ids), ARRAY_SIZE(feature_rep_ids)};

    for (int i = 0; i < ARRAY_SIZE(p_rep_id_lists); ++i)
    {
        for (int j = 0; j < rep_list_lens[i]; ++j)
        {
            uint8_t rep_id;

            rep_id = p_rep_id_lists[i][j];
            if (rep_id == 0 || rep_id > 0x0F)
            {
                NRF_LOG_ERROR("Only Report IDs in range [1, 15] currently supported");
                return NRF_ERROR_NOT_SUPPORTED;
            }
        }
    }

    return NRF_SUCCESS;
}

ret_code_t i2c_hid_init(i2c_hid_evt_handler_t evt_handler)
{
    ret_code_t err_code;

    if (evt_handler == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    err_code = i2c_hid_validate_report_ids();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    m_evt_handler             = evt_handler;
    m_pending_cmd             = I2C_HID_CMD_TYPE_NONE;
    m_cmd_reply_buf_set       = 0;
    m_i2c_read_buf_waiting    = 0;

    nrf_queue_reset(&m_input_rep_queue);
    nrf_queue_reset(&m_evt_queue);
    nrf_balloc_init(&m_input_rep_pool);
    nrf_balloc_init(&m_evt_pool);

    err_code = nrfx_swi_alloc(&m_swi_inst, swi_irqhandler, I2C_HID_EVT_INT_PRIO);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    NRF_LOG_INFO("Input Report count:   %d", I2C_HID_INPUT_REPORT_COUNT);
    NRF_LOG_INFO("Output Report count:  %d", I2C_HID_OUTPUT_REPORT_COUNT);
    NRF_LOG_INFO("Feature Report count: %d", I2C_HID_FEATURE_REPORT_COUNT);
    NRF_LOG_INFO("Maximum report size:  %d bytes", I2C_HID_REPORT_MAX_SIZE);

    return drv_i2c_hid_init(drv_i2c_hid_evt_handler);
}

ret_code_t i2c_hid_req_reset_reply(void)
{
    i2c_hid_input_report_msg_t * p_reset_rep;

    if (m_pending_cmd == I2C_HID_CMD_TYPE_RESET)
    {
        // No chip reset applied: queue should be cleared already
        APP_ERROR_CHECK_BOOL(nrf_queue_is_empty(&m_input_rep_queue));
    }
    else if (m_pending_cmd == I2C_HID_CMD_TYPE_NONE)
    {
        APP_ERROR_CHECK_BOOL(i2c_hid_cmd_fetch_set_pending(I2C_HID_CMD_TYPE_RESET) == I2C_HID_CMD_TYPE_NONE);

        // Chip reset probably applied: make sure queues are empty
        // NOTE: Application responsibility to not send Input Reports if a chip reset is performed upon RESET request from Host
        nrf_queue_reset(&m_input_rep_queue);
        nrf_balloc_init(&m_input_rep_pool);
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }

    p_reset_rep = nrf_balloc_alloc(&m_input_rep_pool);
    APP_ERROR_CHECK_BOOL(p_reset_rep != NULL);

    p_reset_rep->size = 2;
    memset(p_reset_rep->value, 0, sizeof(p_reset_rep->value)); // Two-byte pseudo-Input Report with sentinel value of {0x00, 0x00}

    DRV_I2C_HID_CRITICAL_SECTION_BEGIN();
    APP_ERROR_CHECK(nrf_queue_push(&m_input_rep_queue, &p_reset_rep));
    drv_i2c_hid_interrupt_assert();
    DRV_I2C_HID_CRITICAL_SECTION_END();

    return NRF_SUCCESS;
}

ret_code_t i2c_hid_get_report_reply(uint8_t * p_rep_value, uint16_t length, uint8_t rep_id)
{
    ret_code_t err_code;

    if (p_rep_value == NULL || length == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (m_pending_cmd != I2C_HID_CMD_TYPE_GET_REPORT)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint16_encode((length + sizeof(uint16_t) + sizeof(uint8_t)), &m_i2c_buf_generic[0]);
    m_i2c_buf_generic[2] = rep_id;
    memcpy(&m_i2c_buf_generic[3], p_rep_value, length);

    err_code = NRF_SUCCESS;

    DRV_I2C_HID_CRITICAL_SECTION_BEGIN();
    if (nrf_atomic_flag_clear_fetch(&m_i2c_read_buf_waiting) == 0)
    {
        // I2C read buffer request not yet received
        nrf_atomic_flag_set(&m_cmd_reply_buf_set);
    }
    else
    {
        // I2C read buffer request already pending
        err_code = drv_i2c_hid_read_buf_set(m_i2c_buf_generic, sizeof(m_i2c_buf_generic));
    }
    DRV_I2C_HID_CRITICAL_SECTION_END();

    return err_code;
}

ret_code_t i2c_hid_input_report_send(uint8_t * p_rep_value, uint16_t length, uint8_t rep_id)
{
    i2c_hid_input_report_msg_t * p_inp_rep;

    if (m_pending_cmd == I2C_HID_CMD_TYPE_RESET)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    p_inp_rep = nrf_balloc_alloc(&m_input_rep_pool);
    if (p_inp_rep == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }

    p_inp_rep->size     = length + 3;
    p_inp_rep->value[0] = ((p_inp_rep->size & 0x00FF) >> 0);
    p_inp_rep->value[1] = ((p_inp_rep->size & 0xFF00) >> 8);
    p_inp_rep->value[2] = rep_id;
    memcpy(&p_inp_rep->value[3], p_rep_value, length);

    DRV_I2C_HID_CRITICAL_SECTION_BEGIN();
    APP_ERROR_CHECK(nrf_queue_push(&m_input_rep_queue, &p_inp_rep));
    drv_i2c_hid_interrupt_assert();
    DRV_I2C_HID_CRITICAL_SECTION_END();

    return NRF_SUCCESS;
}