# ESP gateway from temperature and humidity sensor and Apple HomeKit
## Introduction

[HomeKit](https://developer.apple.com/homekit/) is a framework developed by Apple for communicating with and controlling connected accessories in a user’s home using iOS devices.
ESP HomeKit SDK has been developed in-house by Espressif to build Apple HomeKit compatible accessories using ESP32/ESP32-S2/ESP32-C3/ESP8266 SoCs.

Features of this gateway:

* Support for Xiaomi 2 sensor by BLE (LYWSD03MMC)"
* Support for DHT11/12 sensor by GPIO"
* Support for DHT22/21 sensor by GPIO"

## Get Started

### Set up Host environment

Set up the host environment and ESP IDF (**master** branch) as per the steps given [here](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html).

Please clone this repository using the below command:

```text
git clone https://github.com/worknd/hap-gateway.git
```

### Compile and Flash

You can use esp-homekit-sdk with any ESP32 or ESP32-C3 board (I have tested only with the ESP32-DevKit-C and ESP32-C3-DevKit).
I use hard-coded Wi-Fi credentials, so please set the ssid and passphrase by navigating to `idf.py menuconfig -> HAP-Gateway Configuration -> WiFi SSID/Password`
Compile and flash as below:

```text
$ cd /path/to/hap-gateway
$ export ESPPORT=/dev/tty.SLAB_USBtoUART #Set your board's serial port here
$ idf.py set-target <esp32/esp32c3>
$ idf.py flash monitor
```

As the device boots up and it has found your sensor, you will see QR code for HomeKit.

### Add acccessory in the Home app

Open the Home app on your iPhone/iPad and follow these steps:

- Tap on "Add Accessory" and scan the QR code mentioned above.
- Tap on the accessory in the list of Nearby Accessories.
- Select the "Add Anyway" option for the "Uncertified Accessory" prompt.
- Enter 11122333 as the Setup code.
