| Supported Targets | ESP32 |
| ----------------- | ----- |

#Custom BLE Capable HID Macropad
Code heavily modified from ESP-IDF BLE HID Device Demo

Introduction
========================
Implementation of BLE HID profile for a keyboard and consumer device. Possible addition of a BLE SPP. 

This macropad implements a ESP32 as a BLE capable HID interface that is connected to a 3x3 keyboard matrix and a rotary encoder. 

An ATmega16U2 is also implemented as a USB controller for when the device is connected through USB. This allow dual operation mode, BLE + Battery wireless mode and a tethered USB + Rail power mode.

This codebase heavily modifies the demo code provided by Espressif in their BLE HID Device Demo. The modification covers code refactoring to be more descriptive of the functions and attributes. Also, simplified the various different source files and header files to reduce cross-reference (my god was this a headache).

The main.c contains core hardware control, while the hid_dev.c contains the core HID interfacing. hid_device_le_prf.c (that name will be changed) contains the lower level HID profile and descriptors.