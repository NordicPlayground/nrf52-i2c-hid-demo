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
#include <stdbool.h>
#include <stdint.h>

#include "app_timer.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_clock.h"
#include "nrfx_gpiote.h"
#include "nordic_common.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "i2c_hid.h"

#define CHIP_RESET_UPON_HOST_RESET_REQUEST 0 // Do full chip reset when I2C Host sends "RESET" request?
#define RESET_MAGIC_VALUE 0x000000AB

#define MOUSE_REPORT_INTERVAL_MS    10
#define KEYBOARD_REPORT_INTERVAL_MS 100

APP_TIMER_DEF(m_mouse_inp_rep_timer);
APP_TIMER_DEF(m_keyboard_inp_rep_timer);

#if CHIP_RESET_UPON_HOST_RESET_REQUEST
static bool m_flush_logs_and_reset = false;

static void check_for_reset_request(void)
{
    if ((NRF_POWER->GPREGRET & POWER_GPREGRET_GPREGRET_Msk) == RESET_MAGIC_VALUE)
    {
        NRF_LOG_INFO("Reply to RESET from host");
        i2c_hid_req_reset_reply();
    }
    else
    {
        NRF_LOG_INFO("No magic value: 0x%08X", NRF_POWER->GPREGRET);
    }
    NRF_POWER->GPREGRET = 0;
}
#endif /* CHIP_RESET_UPON_HOST_RESET_REQUEST */

static void i2c_hid_evt_handler(i2c_hid_evt_t const * p_evt)
{
    // Using APP_ERROR_CHECK_BOOL() for testing and validation purposes. Normally ignore invalid requests from host?

    switch (p_evt->type)
    {
        case I2C_HID_EVT_OUTPUT_REP:
            APP_ERROR_CHECK_BOOL(p_evt->report.type == I2C_HID_REPORT_TYPE_OUTPUT);
            APP_ERROR_CHECK_BOOL(p_evt->report.id == REPORT_ID_KEYBOARD);
            APP_ERROR_CHECK_BOOL(p_evt->report.rep_length == REPORT_SIZE_KEYBOARD_OUT);

            NRF_LOG_INFO("I2C_HID_EVT_OUTPUT_REP: 0x%02X", p_evt->report.rep_value[0]);
            break;

        case I2C_HID_EVT_TYPE_REQ_RESET:
            NRF_LOG_INFO("I2C_HID_EVT_TYPE_REQ_RESET");
#if CHIP_RESET_UPON_HOST_RESET_REQUEST
            // NOTE 1: i2c_hid_req_reset_reply() can be called immediately instead of doing chip reset
            //        This example illustrates full chip reset as this is the "most extreme" case
            //
            // NOTE 2: It is generally a good idea to have an external pull-up on the interrupt GPIO.
            //         Otherwise this can be floating during chip reset
            NRF_POWER->GPREGRET    = RESET_MAGIC_VALUE;
            m_flush_logs_and_reset = true;
#else
            // Send reply to RESET request right away
            i2c_hid_req_reset_reply();
#endif /* CHIP_RESET_UPON_HOST_RESET_REQUEST*/
            break;

        case I2C_HID_EVT_TYPE_REQ_SET_REPORT:
            NRF_LOG_INFO("I2C_HID_EVT_TYPE_REQ_SET_REPORT");

            APP_ERROR_CHECK_BOOL(p_evt->report.type == I2C_HID_REPORT_TYPE_FEATURE);
            APP_ERROR_CHECK_BOOL(p_evt->report.id == REPORT_ID_USER_CONFIG);
            APP_ERROR_CHECK_BOOL(p_evt->report.rep_length == REPORT_SIZE_USER_CONFIG);

            NRF_LOG_INFO("SET_REPORT: 0x%02X-0x%02X-0x%02X-0x%02X-0x%02X", p_evt->report.rep_value[0], p_evt->report.rep_value[1], p_evt->report.rep_value[2], p_evt->report.rep_value[3], p_evt->report.rep_value[4]);
            break;

        case I2C_HID_EVT_TYPE_REQ_GET_REPORT:
            NRF_LOG_INFO("I2C_HID_EVT_TYPE_REQ_GET_REPORT");

            switch (p_evt->report.type)
            {
                case I2C_HID_REPORT_TYPE_INPUT:
                    if (p_evt->report.id == REPORT_ID_MOUSE)
                    {
                        uint8_t rep_val[REPORT_SIZE_MOUSE] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E};

                        APP_ERROR_CHECK(i2c_hid_get_report_reply(rep_val, sizeof(rep_val), REPORT_ID_MOUSE));
                    }
                    else if (p_evt->report.id == REPORT_ID_KEYBOARD)
                    {
                        uint8_t rep_val[REPORT_SIZE_KEYBOARD] = {0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x10, 0x11, 0x12};

                        APP_ERROR_CHECK(i2c_hid_get_report_reply(rep_val, sizeof(rep_val), REPORT_ID_KEYBOARD));
                    }
                    else
                    {
                        APP_ERROR_CHECK_BOOL(false);
                    }
                    break;
                case I2C_HID_REPORT_TYPE_FEATURE:
                    {
                        uint8_t rep_val[REPORT_SIZE_USER_CONFIG] = {0x2A, 0x2B, 0x2C, 0x2D, 0x2E};

                        APP_ERROR_CHECK_BOOL(p_evt->report.id == REPORT_ID_USER_CONFIG);
                        APP_ERROR_CHECK(i2c_hid_get_report_reply(rep_val, sizeof(rep_val), REPORT_ID_USER_CONFIG));
                    }
                    break;
                case I2C_HID_REPORT_TYPE_OUTPUT:
                    /* Fall-through */
                default:
                    APP_ERROR_CHECK_BOOL(false);
                    break;
            }
            break;

        case I2C_HID_EVT_TYPE_REQ_SET_POWER:
            NRF_LOG_INFO("I2C_HID_EVT_TYPE_REQ_SET_POWER: %s", p_evt->power == I2C_HID_SET_POWER_ON ? "ON" : "SLEEP");
            break;

        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

static void mouse_report_encode(uint8_t * p_rep_value, int16_t x, int16_t y)
{
    uint8_t x_buff[2];
    uint8_t y_buff[2];

    uint16_encode(x, x_buff);
    uint16_encode(y, y_buff);

    p_rep_value[0] = 0x00; // Buttons
    p_rep_value[1] = 0x00; // Wheel
    p_rep_value[2] = x_buff[0];
    p_rep_value[3] = (y_buff[0] << 4) | (x_buff[1] & 0x0f);
    p_rep_value[4] = (y_buff[1] << 4) | (y_buff[0] >> 4);
}

static void keyboard_report_encode(uint8_t * p_rep_value, uint8_t key, uint8_t modifier)
{
    memset(p_rep_value, 0, REPORT_SIZE_KEYBOARD);

    p_rep_value[0] = modifier;
    p_rep_value[2] = key;
}

static void keyboard_input_report_send(void)
{
    static const uint8_t m_key_codes[]          = {0x0B, 0x08, 0x0F, 0x0F, 0x12, 0x36, 0x2C, 0x1A, 0x12, 0x15, 0x0F, 0x07, 0x1E, 0x2C};
    static const uint8_t m_key_code_modifiers[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00};
    static size_t m_key_code_idx = 0;

    STATIC_ASSERT(ARRAY_SIZE(m_key_codes) == ARRAY_SIZE(m_key_code_modifiers));

    ret_code_t err_code;
    uint8_t    keyboard_input_rep[REPORT_SIZE_KEYBOARD];

    keyboard_report_encode(keyboard_input_rep, m_key_codes[m_key_code_idx], m_key_code_modifiers[m_key_code_idx]); // Key press
    err_code = i2c_hid_input_report_send(keyboard_input_rep, sizeof(keyboard_input_rep), REPORT_ID_KEYBOARD);
    if (err_code == NRF_SUCCESS)
    {
        keyboard_report_encode(keyboard_input_rep, 0x00, 0x00);
        err_code = i2c_hid_input_report_send(keyboard_input_rep, sizeof(keyboard_input_rep), REPORT_ID_KEYBOARD); // Key release
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("Failed to send keyboard Input Report because 0x%08X", err_code);
        }
        m_key_code_idx += 1;
        m_key_code_idx %= ARRAY_SIZE(m_key_codes);
    }
    else
    {
        NRF_LOG_WARNING("Failed to send keyboard Input Report because 0x%08X", err_code);
    }
}

static void mouse_input_report_send(void)
{
    static const int16_t m_x_deltas[] = {10, 9, 8, 7, 5, 3, 0, -1, -4, -6, -8, -9, -9, -9, -9, -8, -6, -4, -1, 0, 3, 5, 7, 8, 9};
    static const int16_t m_y_deltas[] = {0, 2, 4, 6, 8, 9, 9, 9, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -9, -9, -9, -8, -6, -4, -2};
    static size_t m_delta_idx = 0;

    STATIC_ASSERT(ARRAY_SIZE(m_x_deltas) == ARRAY_SIZE(m_y_deltas));

    ret_code_t err_code;
    uint8_t    mouse_input_rep[REPORT_SIZE_MOUSE];

    mouse_report_encode(mouse_input_rep, m_x_deltas[m_delta_idx], m_y_deltas[m_delta_idx]);
    err_code = i2c_hid_input_report_send(mouse_input_rep, sizeof(mouse_input_rep), REPORT_ID_MOUSE);
    if (err_code == NRF_SUCCESS)
    {
        m_delta_idx += 1;
        m_delta_idx %= ARRAY_SIZE(m_x_deltas);
    }
    else
    {
        NRF_LOG_WARNING("Failed to send mouse Input Report because 0x%08X", err_code);
    }
}

static void button_evt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    static bool m_keyboard_timer_running = false;
    static bool m_mouse_timer_running = false;



    if (action == NRF_GPIOTE_POLARITY_LOTOHI)
    {
        // Ignore release if configured
        return;
    }

    switch (pin)
    {
        case BUTTON_1:
            mouse_input_report_send();
            break;

        case BUTTON_2:
            keyboard_input_report_send();
            break;

        case BUTTON_3:
            if (m_keyboard_timer_running)
            {
                APP_ERROR_CHECK(app_timer_stop(m_keyboard_inp_rep_timer));
                m_keyboard_timer_running = false;
            }
            else
            {
                APP_ERROR_CHECK(app_timer_start(m_keyboard_inp_rep_timer, APP_TIMER_TICKS(KEYBOARD_REPORT_INTERVAL_MS), (void*) keyboard_input_report_send));
                m_keyboard_timer_running = true;
            }
            break;

        case BUTTON_4:
            if (m_mouse_timer_running)
            {
                APP_ERROR_CHECK(app_timer_stop(m_mouse_inp_rep_timer));
                m_mouse_timer_running = false;
            }
            else
            {
                APP_ERROR_CHECK(app_timer_start(m_mouse_inp_rep_timer, APP_TIMER_TICKS(MOUSE_REPORT_INTERVAL_MS), (void*) mouse_input_report_send));
                m_mouse_timer_running = true;
            }
            break;

        default:
            NRF_LOG_WARNING("Unknown event for pin %d", pin);
            break;
    }
}

static void inp_rep_send_timeout_func(void * p_context)
{
    ((void (*)(void))p_context)();
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void buttons_init(void)
{
    nrfx_gpiote_pin_t       button_gpios[] = BUTTONS_LIST;
    nrfx_gpiote_in_config_t gpiote_pin_cfg =
    {
        .is_watcher      = false,
        .hi_accuracy     = false,
        .pull            = NRF_GPIO_PIN_PULLUP,
        .sense           = NRF_GPIOTE_POLARITY_HITOLO,
        .skip_gpio_setup = false,
    };

    APP_ERROR_CHECK(nrfx_gpiote_init());

    for (int i = 0; i < ARRAY_SIZE(button_gpios); ++i)
    {
        APP_ERROR_CHECK(nrfx_gpiote_in_init(button_gpios[i], &gpiote_pin_cfg, button_evt_handler));
        nrfx_gpiote_in_event_enable(button_gpios[i], true);
    }
}

static void timers_init(void)
{
    nrf_clock_lf_src_set(NRF_CLOCK_LFCLK_Xtal);
    nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);

    APP_ERROR_CHECK(app_timer_init());

    APP_ERROR_CHECK(app_timer_create(&m_mouse_inp_rep_timer, APP_TIMER_MODE_REPEATED, inp_rep_send_timeout_func));
    APP_ERROR_CHECK(app_timer_create(&m_keyboard_inp_rep_timer, APP_TIMER_MODE_REPEATED, inp_rep_send_timeout_func));
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    log_init();
    buttons_init();
    timers_init();

    NRF_LOG_INFO("I2C HID example");
    NRF_LOG_INFO("Press button 1 on nRF52-DK to send single mouse Input Report");
    NRF_LOG_INFO("Press button 2 on nRF52-DK to send single keyboard Input Report");
    NRF_LOG_INFO("Press button 3 on nRF52-DK to send keyboard Input Reports at %d Hz", (1000 / KEYBOARD_REPORT_INTERVAL_MS));
    NRF_LOG_INFO("Press button 4 on nRF52-DK to send mouse Input Reports at %d Hz", (1000 / MOUSE_REPORT_INTERVAL_MS));
    NRF_LOG_FLUSH();

    APP_ERROR_CHECK(i2c_hid_init(i2c_hid_evt_handler));

#if CHIP_RESET_UPON_HOST_RESET_REQUEST
    check_for_reset_request();
#endif /* CHIP_RESET_UPON_HOST_RESET_REQUEST */

    while (true)
    {
#if CHIP_RESET_UPON_HOST_RESET_REQUEST
        if (m_flush_logs_and_reset)
        {
            NRF_LOG_FINAL_FLUSH();
            NVIC_SystemReset();
        }
#endif /* CHIP_RESET_UPON_HOST_RESET_REQUEST */

        // Do nothing.
        while (NRF_LOG_PROCESS())
        {
            // Process logs
        }

        // CPU sleep
        __WFE();
        __SEV();
        __WFE();
    }
}
/** @} */
