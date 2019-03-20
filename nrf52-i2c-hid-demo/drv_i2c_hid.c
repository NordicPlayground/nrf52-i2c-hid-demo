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
#include "drv_i2c_hid.h"

#include "nrf_gpio.h"
#include "nrfx_twis.h"

#define NRF_LOG_MODULE_NAME drv_i2c_hid
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

static nrfx_twis_t m_twis_instance = NRFX_TWIS_INSTANCE(DRV_I2C_HID_I2C_INSTANCE);

static drv_i2c_hid_evt_handler_t m_evt_handler;
static uint8_t *                 mp_i2c_rx_buf;
static uint8_t const *           mp_i2c_tx_buf;

static void twis_event_handler(nrfx_twis_evt_t const * p_event)
{
    switch (p_event->type)
    {
        case NRFX_TWIS_EVT_READ_REQ:
        {
            NRF_LOG_DEBUG("NRFX_TWIS_EVT_READ_REQ");
            if (p_event->data.buf_req)
            {
                drv_i2c_hid_evt_t evt = 
                {
                    .type = DRV_I2C_HID_EVT_TYPE_READ_BUF_REQ,
                };

                m_evt_handler(&evt);
            }
        } break;

        case NRFX_TWIS_EVT_READ_DONE:
        {
            drv_i2c_hid_evt_t evt = 
            {
                .type = DRV_I2C_HID_EVT_TYPE_READ_DONE,
                .data.rw_done.p_data = mp_i2c_tx_buf,
                .data.rw_done.len    = p_event->data.tx_amount,
            };

            NRF_LOG_DEBUG("NRFX_TWIS_EVT_READ_DONE");

            m_evt_handler(&evt);

            mp_i2c_tx_buf = NULL;
        } break;

        case NRFX_TWIS_EVT_READ_ERROR:
        {
            NRF_LOG_DEBUG("NRFX_TWIS_EVT_READ_ERROR");
        } break;

        case NRFX_TWIS_EVT_WRITE_REQ:
        {
            NRF_LOG_DEBUG("NRFX_TWIS_EVT_WRITE_REQ");
            if (p_event->data.buf_req)
            {
                drv_i2c_hid_evt_t evt = 
                {
                    .type = DRV_I2C_HID_EVT_TYPE_WRITE_BUF_REQ,
                };

                m_evt_handler(&evt);
            }
        } break;

        case NRFX_TWIS_EVT_WRITE_DONE:
        {
            drv_i2c_hid_evt_t evt = 
            {
                .type = DRV_I2C_HID_EVT_TYPE_WRITE_DONE,
                .data.rw_done.p_data = mp_i2c_rx_buf,
                .data.rw_done.len    = p_event->data.rx_amount,
            };
            NRF_LOG_DEBUG("NRFX_TWIS_EVT_WRITE_DONE");

            m_evt_handler(&evt);

            mp_i2c_rx_buf = NULL;
        } break;

        case NRFX_TWIS_EVT_WRITE_ERROR:
        {
            drv_i2c_hid_evt_t evt = 
            {
                .type = DRV_I2C_HID_EVT_TYPE_ERROR,
                .data.error.err_code = p_event->data.error,
            };
            NRF_LOG_WARNING("NRFX_TWIS_EVT_WRITE_ERROR");

            m_evt_handler(&evt);
            
        } break;

        case NRFX_TWIS_EVT_GENERAL_ERROR:
        {

            switch (p_event->data.error)
            {
                case NRFX_TWIS_ERROR_OVERFLOW:
                    NRF_LOG_WARNING("NRFX_TWIS_ERROR_OVERFLOW");
                    break;

                case NRFX_TWIS_ERROR_OVERREAD:
                    NRF_LOG_WARNING("NRFX_TWIS_ERROR_OVERREAD");
                    break;

                case NRFX_TWIS_ERROR_DATA_NACK:
                    NRF_LOG_WARNING("NRFX_TWIS_ERROR_DATA_NACK");
                    break;

                case NRFX_TWIS_ERROR_UNEXPECTED_EVENT:
                    NRF_LOG_WARNING("NRFX_TWIS_ERROR_UNEXPECTED_EVENT");
                    break;

                default:
                    APP_ERROR_CHECK_BOOL(false);
                    break;
            }

            // TODO: Determine if error should be propagated as event

            // drv_i2c_hid_evt_t evt = 
            // {
            //     .type = DRV_I2C_HID_EVT_TYPE_ERROR,
            //     .data.error.err_code = p_event->data.error,
            // };
            // ///< Error that happens not inside WRITE or READ transaction.
            // NRF_LOG_WARNING("NRFX_TWIS_EVT_GENERAL_ERROR");

            // m_evt_handler(&evt);
        } break;

        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

ret_code_t drv_i2c_hid_init(drv_i2c_hid_evt_handler_t evt_handler)
{
    ret_code_t err_code;

    if (evt_handler == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_evt_handler = evt_handler;
    mp_i2c_rx_buf = NULL;
    mp_i2c_tx_buf = NULL;

    NRF_LOG_INFO("I2C Address:  0x%02X", DRV_I2C_HID_I2C_ADDR);
    NRF_LOG_INFO("I2C irq pri:  %d", DRV_I2C_HID_I2C_INTERRUPT_PRIORITY);
    NRF_LOG_INFO("I2C GPIO SDA: P%d.%d", DRV_I2C_HID_GPIO_SDA > 31 ? 1 : 0, DRV_I2C_HID_GPIO_SDA % 32);
    NRF_LOG_INFO("I2C GPIO SCL: P%d.%d", DRV_I2C_HID_GPIO_SCL > 31 ? 1 : 0, DRV_I2C_HID_GPIO_SCL % 32);
    NRF_LOG_INFO("I2C GPIO INT: P%d.%d", DRV_I2C_HID_GPIO_INT > 31 ? 1 : 0, DRV_I2C_HID_GPIO_INT % 32);

    nrfx_twis_config_t twis_config = 
    {
        .addr               = {DRV_I2C_HID_I2C_ADDR, 0},
        .scl                = DRV_I2C_HID_GPIO_SCL,
        .sda                = DRV_I2C_HID_GPIO_SDA,
        .scl_pull           = NRF_GPIO_PIN_NOPULL,
        .sda_pull           = NRF_GPIO_PIN_NOPULL,
        .interrupt_priority = DRV_I2C_HID_I2C_INTERRUPT_PRIORITY,
    };

    nrf_gpio_cfg_output(DRV_I2C_HID_GPIO_INT);
    nrf_gpio_pin_set(DRV_I2C_HID_GPIO_INT);

    err_code = nrfx_twis_init(&m_twis_instance, &twis_config, twis_event_handler);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("nrfx_twis_init: 0x%08X", err_code);
        return err_code;
    }

    nrfx_twis_enable(&m_twis_instance);

    return NRF_SUCCESS;
}

ret_code_t drv_i2c_hid_write_buf_set(void * p_buf, size_t size)
{
    ret_code_t err_code;

    DRV_I2C_HID_CRITICAL_SECTION_BEGIN();
    err_code = nrfx_twis_rx_prepare(&m_twis_instance, p_buf, size);
    if (err_code == NRF_SUCCESS)
    {
        mp_i2c_rx_buf = p_buf;
    }
    DRV_I2C_HID_CRITICAL_SECTION_END();

    return err_code;
}

ret_code_t drv_i2c_hid_read_buf_set(void const * p_buf, size_t size)
{
    ret_code_t err_code;

    DRV_I2C_HID_CRITICAL_SECTION_BEGIN();
    err_code = nrfx_twis_tx_prepare(&m_twis_instance, p_buf, size);
    if (err_code == NRF_SUCCESS)
    {
        mp_i2c_tx_buf = p_buf;
    }
    DRV_I2C_HID_CRITICAL_SECTION_END();

    return err_code;
}

void drv_i2c_hid_interrupt_assert(void)
{
    nrf_gpio_pin_clear(DRV_I2C_HID_GPIO_INT);
}

void drv_i2c_hid_interrupt_deassert(void)
{
    nrf_gpio_pin_set(DRV_I2C_HID_GPIO_INT);
}
