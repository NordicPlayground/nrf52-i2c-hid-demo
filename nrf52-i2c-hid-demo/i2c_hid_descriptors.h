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
#ifndef __I2C_HID_DESCRIPTORS_H__
#define __I2C_HID_DESCRIPTORS_H__

#define REPORT_SIZE_MOUSE        5 /* bytes */
#define REPORT_SIZE_MOUSE_BOOT   3 /* bytes */
#define REPORT_SIZE_KEYBOARD     9 /* bytes */
#define REPORT_SIZE_KEYBOARD_OUT 1 /* bytes */
#define REPORT_SIZE_USER_CONFIG  5 /* bytes */

#define REPORT_ID_MOUSE          0x01
#define REPORT_ID_KEYBOARD       0x02
#define REPORT_ID_USER_CONFIG    0x03

#define USAGE_PAGE_MOUSE_XY	     0x01
#define USAGE_PAGE_MOUSE_WHEEL   0x01
#define USAGE_PAGE_KEYBOARD      0x07
#define USAGE_PAGE_LEDS	         0x08
#define USAGE_PAGE_MOUSE_BUTTONS 0x09

/*****************************/
/*     API defines below     */
/*****************************/

#define I2C_HID_VENDOR_ID  0x1915 /* wVendorID  */
#define I2C_HID_PRODUCT_ID 0xABCD /* wProductID */
#define I2C_HID_VERSION_ID 0x0001 /* wVersionID */

// Note about report lists:
// If there is only a single report in a list and the Report Descriptor does not specify a Report ID, the Report ID can be omitted according to HID over I2C specification.
// This is not supported yet in this code. Always specify Report ID in the range of 1-15 in the Report Descriptor and in the lists below

#define I2C_HID_INPUT_REPORT_LIST   \
    X(REPORT_ID_MOUSE,    REPORT_SIZE_MOUSE) \
    X(REPORT_ID_KEYBOARD, REPORT_SIZE_KEYBOARD)

#define I2C_HID_OUTPUT_REPORT_LIST  \
    X(REPORT_ID_KEYBOARD, REPORT_SIZE_KEYBOARD_OUT)

#define I2C_HID_FEATURE_REPORT_LIST \
    X(REPORT_ID_USER_CONFIG, REPORT_SIZE_USER_CONFIG)

#define I2C_HID_REPORT_DESCRIPTOR                              \
    /* Usage page */                                           \
    0x05, 0x01,     /* Usage Page (Generic Desktop) */         \
    0x09, 0x02,     /* Usage (Mouse) */                        \
    0xA1, 0x01,     /* Collection (Application) */             \
                                                               \
    /* Report: Mouse */                                        \
    0x09, 0x01,       /* Usage (Pointer) */                    \
    0xA1, 0x00,       /* Collection (Physical) */              \
    0x85, REPORT_ID_MOUSE,                                     \
                                                               \
    0x05, USAGE_PAGE_MOUSE_BUTTONS,                            \
    0x19, 0x01,         /* Usage Minimum (1) */                \
    0x29, 0x08,         /* Usage Maximum (8) */                \
    0x15, 0x00,         /* Logical Minimum (0) */              \
    0x25, 0x01,         /* Logical Maximum (1) */              \
    0x75, 0x01,         /* Report Size (1) */                  \
    0x95, 0x08,         /* Report Count (8) */                 \
    0x81, 0x02,         /* Input (Data, Variable, Absolute) */ \
                                                               \
    0x05, USAGE_PAGE_MOUSE_WHEEL,                              \
    0x09, 0x38,         /* Usage (Wheel) */                    \
    0x15, 0x81,         /* Logical Minimum (-127) */           \
    0x25, 0x7F,         /* Logical Maximum (127) */            \
    0x75, 0x08,         /* Report Size (8) */                  \
    0x95, 0x01,         /* Report Count (1) */                 \
    0x81, 0x06,         /* Input (Data, Variable, Relative) */ \
                                                               \
    0x05, USAGE_PAGE_MOUSE_XY,                                 \
    0x09, 0x30,         /* Usage (X) */                        \
    0x09, 0x31,         /* Usage (Y) */                        \
    0x16, 0x01, 0xF8,   /* Logical Maximum (2047) */           \
    0x26, 0xFF, 0x07,   /* Logical Minimum (-2047) */          \
    0x75, 0x0C,         /* Report Size (12) */                 \
    0x95, 0x02,         /* Report Count (2) */                 \
    0x81, 0x06,         /* Input (Data, Variable, Relative) */ \
                                                               \
    0xC0,             /* End Collection (Physical) */          \
                                                               \
    /* Report: Configuration feature report */                 \
    0x05, 0x01,       /* Usage Page (Generic Desktop) */       \
    0x85, REPORT_ID_USER_CONFIG,                               \
    0x09, 0x05,       /* Usage (Vendor Defined) */             \
    0x15, 0x00,       /* Logical Minimum (0) */                \
    0x26, 0xFF, 0x00, /* Logical Maximum (255) */              \
    0x75, 0x08,       /* Report Size (8) */                    \
    0x95, REPORT_SIZE_USER_CONFIG, /* Report Count */          \
    0xB1, 0x02,       /* Feature (Data, Variable, Absolute) */ \
    0xC0,           /* End Collection (Application) */         \
                                                               \
    /* Usage page - Keyboard */                                \
    0x05, 0x01,     /* Usage Page (Generic Desktop) */         \
    0x09, 0x06,     /* Usage (Mouse) */                        \
                                                               \
    0xA1, 0x01,     /* Collection (Application) */             \
                                                               \
    /* Report: Keyboard */                                     \
    0x85, REPORT_ID_KEYBOARD,                                  \
                                                               \
    /* Keyboard - Modifiers */                                 \
    0x05, USAGE_PAGE_KEYBOARD,                                 \
    0x19, 0xe0,       /* Usage Minimum (Left Ctrl) */          \
    0x29, 0xe7,       /* Usage Maximum (Right GUI) */          \
    0x15, 0x00,       /* Logical Minimum (0) */                \
    0x25, 0x01,       /* Logical Maximum (1) */                \
    0x75, 0x01,       /* Report Size (1) */                    \
    0x95, 0x08,       /* Report Count (8) */                   \
    0x81, 0x02,       /* Input (Data, Variable, Absolute) */   \
                                                               \
    /* Keyboard - Reserved */                                  \
    0x75, 0x08,       /* Report Size (8) */                    \
    0x95, 0x01,       /* Report Count (1) */                   \
    0x81, 0x01,       /* Input (Constant) */                   \
                                                               \
    /* Keyboard - Keys */                                      \
    0x05, USAGE_PAGE_KEYBOARD,                                 \
    0x19, 0x00,       /* Usage Minimum (0) */                  \
    0x29, 0x65,       /* Usage Maximum (101) */                \
    0x15, 0x00,       /* Logical Minimum (0) */                \
    0x25, 0x65,       /* Logical Maximum (101) */              \
    0x75, 0x08,       /* Report Size (8) */                    \
    0x95, 0x06,       /* Report Count (6) */                   \
    0x81, 0x00,       /* Input (Data, Array) */                \
                                                               \
    /* Keyboard - LEDs */                                      \
    0x05, USAGE_PAGE_LEDS,                                     \
    0x19, 0x01,       /* Usage Minimum (1) */                  \
    0x29, 0x05,       /* Usage Maximum (5) */                  \
    0x95, 0x05,       /* Report Count (5) */                   \
    0x75, 0x01,       /* Report Size (1) */                    \
    0x91, 0x02,       /* Output (Data, Variable, Absolute) */  \
                                                               \
    /* Keyboard - LEDs padding */                              \
    0x95, 0x01,       /* Report Count (1) */                   \
    0x75, 0x03,       /* Report Size (3) (padding) */          \
    0x91, 0x01,       /* Output (Data, Variable, Absolute) */  \
                                                               \
    0xC0,           /* End Collection (Application) */         \

#endif /* __I2C_HID_DESCRIPTORS_H__*/