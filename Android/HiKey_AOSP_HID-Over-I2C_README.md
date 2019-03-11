# HID over I2C on HiKey AOSP
This is an example demonstrating HID over I2C on HiKey Android reference board.

## Hardware Requirement
- [HiKey (LeMaker)](https://www.96boards.org/product/hikey/)

## How to Build

1. Initialize your environment as described in
   http://source.android.com/source/initializing.html
2. git clone https://android.googlesource.com/kernel/hikey-linaro
3. cd hikey-linaro
4. git checkout -b android-hikey-linaro-4.14 origin/android-hikey-linaro-4.14
5. make ARCH=arm64 hikey_defconfig
6. Enable HID over I2C by setting CONFIG_I2C_HID=y
7. Copy device tree patch to kernel root directory and apply patch
    * patch -p1 < ./android-hikey-linaro-4.14_hid-over-i2c.patch
8. make ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- -j24
9. Copy output to the AOSP hikey kernel directory (/kernel/hikey-linaro):
    * Copy hi6220-hikey.dtb (arch/arm64/boot/dts/hisilicon/hi6220-hikey.dtb) to the hikey-kernel directory as file hi6220-hikey.dtb-4.14.
    * Copy the Image file (arch/arm64/boot/Image.gz-dtb) to the hikey-kernel directory as file Image.gz-dtb-4.14.
10. Compiling Android userspace image as described in https://source.android.com/setup/build/devices#620userspace