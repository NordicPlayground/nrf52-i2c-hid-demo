# HID over I2C on Raspberry Pi 3
This is an example demonstrating HID over I2C on Raspberry Pi 3 running Raspbian Stretch.

## Hardware Requirement
- [Raspberry Pi 3](https://www.raspberrypi.org/products/)

## How to Build
The HID I2C Kernel module must be compiled and loaded in order to enable HID over I2C.
The Kernel can be built either locally on the Raspberry Pi, or cross-compiled on a separate computer.
The following instructions illustrates a cross-compile build using 64-bit Ubuntu, based on [these original instructions](https://www.raspberrypi.org/documentation/linux/kernel/building.md).
An overlay file (found in this repo) from [this thread](https://www.raspberrypi.org/forums/viewtopic.php?t=152803) is used as reference.


1. Make a note of which Kernel version your Raspberry Pi is using by running `uname -r` command. For example, output from this command on Raspbian Stretch is currently `4.14.98-v7+`.
2. On the Ubuntu computer, install required packages: 
```
sudo apt-get update && sudo apt-get install -y git bison flex libssl-dev make libncurses-dev device-tree-compiler
```
3. Download the toolchain to your home folder: 
```
git clone https://github.com/raspberrypi/tools ~/tools
```
4. Create PATH variable for the tools folder (NOTE: this is assuming 64-bit Ubuntu):
```
echo PATH=\$PATH:~/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin >> ~/.bashrc
source ~/.bashrc
```
5. Download Kernel sources using the version found in step 1 and generate default config. This example uses v4.14.98. NOTE: The *_defconfig command is slightly different if you are using Pi 1, Pi Zero, Pi Zero W, or Compute Module. See [here](https://www.raspberrypi.org/documentation/linux/kernel/building.md) for details.
```
git clone --depth=1 https://github.com/raspberrypi/linux --branch rpi-4.14.y
cd linux
KERNEL=kernel7
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- bcm2709_defconfig
```
6. Copy *i2c_hid-bcm2708-overlay.dts* from this repo into the downloaded kernel tree and manually compile it:
```
cp i2c_hid-bcm2708-overlay.dts arch/arm/boot/dts/overlays/
dtc -@ -I dts -O dtb -o arch/arm/boot/dts/overlays/i2c_hid-bcm2708.dtbo arch/arm/boot/dts/overlays/i2c_hid-bcm2708-overlay.dts
```
7. Run menuconfig (`make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- menuconfig`) and navigate to the "**Device Drivers -> HID support -> I2C HID support**" option. Enable "**HID over I2C transport layer**" module option by clicking 'M' key. The option should now have an 'M' next to it. Save and exit menuconfig.
8. Build the kernel: 
```
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage modules dtbs
```
9. Follow the steps for **Install directly onto the SD card** as described [here](https://www.raspberrypi.org/documentation/linux/kernel/building.md).
10. Boot the Raspberry Pi using the updated SD card, and enable I2C and I2C-HID by making the following modifications to **/boot/config.txt** on the Raspberry Pi:
```
sudo -- sh -c 'echo "dtparam=i2c_arm=on" >> /boot/config.txt'
sudo -- sh -c 'echo "dtparam=i2c_arm_baudrate=400000" >> /boot/config.txt'
sudo -- sh -c 'echo "dtoverlay=i2c_hid-bcm2708" >> /boot/config.txt'
```
11. Reboot Raspberry Pi to make changes take effect. **NOTE: The I2C HID Slave must be connected to the I2C bus (SDA = GPIO2, SCL = GPIO3, INTERRUPT = GPIO27) when the I2C HID module is loaded (normally at boot) for the I2C HID device to be detected**. See [GPIO layout](https://www.raspberrypi.org/documentation/usage/gpio/) for details.
