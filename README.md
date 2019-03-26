nrf52-i2c-hid-demo
=============================
This demo implements the Peripheral-side HID over I2C protocol as specified in [HID Over I2C Protocol Specification](https://technet.microsoft.com/en-us/windows/dn642101(v=vs.60)#MainContent). 

HID over I2C is a specification that lets an I2C device appear as a HID Device such as a mouse and keyboard to a Host device.
This demo uses a Report Descriptor that enumerates as a mouse + keyboard composite device. 

Requirements
------------
- nRF5 SDK version 15.3.0
- nRF52-DK (PCA10040) or nRF52840-DK (PCA10056)
- HID over I2C Host such as [HiKey (LeMaker)](https://www.96boards.org/product/hikey/) or [Raspberry Pi 3](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/)


Firmware setup
--------------
Clone or download the git repo, and place the nrf52-i2c-hid-demo folder in the following SDK subfolder: **nRF5_SDK_15.3.0_59ac345\examples\peripheral**.

Build the firmware using either GCC or SES, and flash the firmware using for example nrfjprog.

Example of using GCC to build the PCA10040 (nRF52832) target:
```
$ cd nRF5_SDK_15.3.0_59ac345\examples\peripheral\nrf52-i2c-hid-demo\pca10040\blank\armgcc
$ make nrf52832_xxaa
$ nrfjprog --program _build\nrf52832_xxaa.hex --chiperase --reset
```


Hardware setup
--------------
HID over I2C uses three GPIOs: SCL, SDA, and INTERRUPT.
The default nRF52 GPIO configuration is as follows:

| Board    | SDA   | SCL   | INTERRUPT |
| -------- |:-----:|:-----:|:---------:|
| PCA10040 | P0.24 | P0.25 | P0.23     |
| PCA10056 | P1.14 | P1.15 | P1.13     |

Connect the GPIOs to the corresponding host pins. Note that long wires or too strong pull resistors could cause transmission errors or limit the SCL speed.

This repo contains an example for building and configuring **AOSP** as the HID over I2C Host [here](Android/HiKey_AOSP_HID-Over-I2C_README.md), and an example for configure a **Rasperry Pi 3** as the HID over I2C Host [here](Raspbian/Raspbian_HID-Over-I2C_README.md).


Firmware behavior
-----------------
The firmware uses UART printouts to provide debugging information. 
PCA10040 and PCA10056 will enumerate as a COM port when connected to USB. Use any serial port reader (for example Putty) to connect to this COM port with 115200/8-N-1, and no flow control (this is the default Putty configuration).

When the Host enumerates HID Devices on the I2C bus, you will typically see the following printout:
```
<info> app: I2C_HID_EVT_TYPE_REQ_SET_POWER: ON
<info> app: I2C_HID_EVT_TYPE_REQ_RESET
```

The demo uses Buttons 1 to 4 on the DK to do the following:

| Button   | Function                                                                 |
|:--------:| ------------------------------------------------------------------------ |
| Button 1 | Send one Mouse Input Report                                              |
| Button 2 | Send one Keyboard Input Report                                           |
| Button 3 | Toggle continous transmission of Keyboard Input Reports (default: 10 Hz) |
| Button 4 | Toggle continous transmission of Mouse Input Reports (default: 100 Hz)   |

The maximum transmission rate of Input Reports depends on the maximum report length and the I2C bus speed.
With this Report Descriptor and 400 kHz bus speed, the maximum report rate is well above 1000 reports per second


SoftDevice version
------------------

This demo does not use a SoftDevice. There are no limitations for adding a SoftDevice by the user. 


HID Descriptors
---------------
The HID Descriptor and Report Descriptor is defined in [i2c_hid_descriptors.h](nrf52-i2c-hid-demo/i2c_hid_descriptors.h).

To use a different Report Descriptor, the **I2C_HID_REPORT_DESCRIPTOR** define should be updated.

Furthermore, **I2C_HID_INPUT_REPORT_LIST**, **I2C_HID_OUTPUT_REPORT_LIST**, and **I2C_HID_FEATURE_REPORT_LIST** must be manually populated with the Report IDs and Report sizes that are defined in the report descriptor. Leave the list empty if there are no Reports of a given type. 


Notes and limitations
---------------------
- The nRF52 firmware will by default configure the SDA and SCL pins without a pull resistor, assuming that the I2S bus has pull-up resistors already. 
- The Report Descriptor is limited to 255 bytes in length on the nRF52832. The nRF52810 and nRF52840 SoCs can have larger TWIS buffers, but for simplicity this demo enforces the 255 byte limit for all hardware targets. 
- All Reports must have an explicit Report ID in the Report Descriptor, even if there is only one Report of a given type. The Report ID must be in the range of 1-15. In other words, the third ID byte in HID over I2C Specification is not currently supported. 


About this project
------------------
This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty. 

However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub. 

The application is built to be used with the official nRF5 SDK, that can be downloaded from developer.nordicsemi.com

Please post any questions about this project on [devzone](https://devzone.nordicsemi.com)