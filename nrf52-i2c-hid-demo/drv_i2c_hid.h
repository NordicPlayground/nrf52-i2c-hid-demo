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
#ifndef __DRV_I2C_HID_H__
#define __DRV_I2C_HID_H__

#include <stdint.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "app_error.h"

#define DRV_I2C_HID_I2C_INSTANCE_DEFAULT           0
#define DRV_I2C_HID_I2C_ADDR_DEFAULT               (0x0A)
#define DRV_I2C_HID_I2C_INTERRUPT_PRIORITY_DEFAULT 7
#ifdef BOARD_PCA10056
#define DRV_I2C_HID_GPIO_SDA_DEFAULT               NRF_GPIO_PIN_MAP(1, 14)
#define DRV_I2C_HID_GPIO_SCL_DEFAULT               NRF_GPIO_PIN_MAP(1, 15)
#define DRV_I2C_HID_GPIO_INT_DEFAULT               NRF_GPIO_PIN_MAP(1, 13)
#else
#define DRV_I2C_HID_GPIO_SDA_DEFAULT               NRF_GPIO_PIN_MAP(0, 24)
#define DRV_I2C_HID_GPIO_SCL_DEFAULT               NRF_GPIO_PIN_MAP(0, 25)
#define DRV_I2C_HID_GPIO_INT_DEFAULT               NRF_GPIO_PIN_MAP(0, 23)
#endif

#ifndef DRV_I2C_HID_GPIO_SDA
#define DRV_I2C_HID_GPIO_SDA DRV_I2C_HID_GPIO_SDA_DEFAULT
#endif /* DRV_I2C_HID_GPIO_SDA */

#ifndef DRV_I2C_HID_GPIO_SCL
#define DRV_I2C_HID_GPIO_SCL DRV_I2C_HID_GPIO_SCL_DEFAULT
#endif /* DRV_I2C_HID_GPIO_SCL */

#ifndef DRV_I2C_HID_GPIO_INT
#define DRV_I2C_HID_GPIO_INT DRV_I2C_HID_GPIO_INT_DEFAULT
#endif /* DRV_I2C_HID_GPIO_INT */

#ifndef DRV_I2C_HID_I2C_INSTANCE
#define DRV_I2C_HID_I2C_INSTANCE DRV_I2C_HID_I2C_INSTANCE_DEFAULT
#endif /* DRV_I2C_HID_I2C_INSTANCE */

#ifndef DRV_I2C_HID_I2C_ADDR
#define DRV_I2C_HID_I2C_ADDR DRV_I2C_HID_I2C_ADDR_DEFAULT
#endif /* DRV_I2C_HID_I2C_ADDR */

#ifndef DRV_I2C_HID_I2C_INTERRUPT_PRIORITY
#define DRV_I2C_HID_I2C_INTERRUPT_PRIORITY DRV_I2C_HID_I2C_INTERRUPT_PRIORITY_DEFAULT
#endif /* DRV_I2C_HID_I2C_INTERRUPT_PRIORITY */

#if defined(NRF52810_XXAA)
# if DRV_I2C_HID_I2C_INSTANCE == 0
#  define DRV_I2C_HID_IRQn TWIM0_TWIS0_IRQn
# else
#  error Invalid TWIS instance
# endif /* DRV_I2C_HID_I2C_INSTANCE */
#elif defined(NRF52832_XXAA) || defined (NRF52832_XXAB) || defined(NRF52840_XXAA)
# if DRV_I2C_HID_I2C_INSTANCE == 0
#  define DRV_I2C_HID_IRQn SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn
# elif DRV_I2C_HID_I2C_INSTANCE == 1
#  define DRV_I2C_HID_IRQn SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn
# else
#  error Invalid TWIS instance
# endif /* DRV_I2C_HID_I2C_INSTANCE */
#else
    #error "Unknown device."
#endif

#define DRV_I2C_HID_CRITICAL_SECTION_BEGIN() NVIC_DisableIRQ(DRV_I2C_HID_IRQn)
#define DRV_I2C_HID_CRITICAL_SECTION_END() NVIC_EnableIRQ(DRV_I2C_HID_IRQn)

typedef enum
{
    DRV_I2C_HID_EVT_TYPE_WRITE_BUF_REQ,
    DRV_I2C_HID_EVT_TYPE_READ_BUF_REQ,
    DRV_I2C_HID_EVT_TYPE_WRITE_DONE,
    DRV_I2C_HID_EVT_TYPE_READ_DONE,
    DRV_I2C_HID_EVT_TYPE_ERROR
} drv_i2c_hid_evt_type_t;

typedef struct
{
    drv_i2c_hid_evt_type_t type;
    union
    {
        struct
        {
            size_t          len;
            uint8_t const * p_data;
        } rw_done;
        struct
        {
            uint32_t err_code;
        } error;
    } data;
} drv_i2c_hid_evt_t;

typedef void (*drv_i2c_hid_evt_handler_t)(drv_i2c_hid_evt_t const * p_evt);

ret_code_t drv_i2c_hid_init(drv_i2c_hid_evt_handler_t evt_handler);
ret_code_t drv_i2c_hid_write_buf_set(void * p_buf, size_t size);
ret_code_t drv_i2c_hid_read_buf_set(void const * p_buf, size_t size);
bool drv_i2c_hid_interrupt_asserted(void);
void drv_i2c_hid_interrupt_assert(void);
void drv_i2c_hid_interrupt_deassert(void);

#endif /* __DRV_I2C_HID_H__ */
