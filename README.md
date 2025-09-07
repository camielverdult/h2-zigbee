| Supported Targets | ESP32-H2 | ESP32-C6 |
| ----------------- | -------- | -------- |

# Temperature Sensor Example

This project demonstrates how to expose BME280 sensor data from a Zigbee end device.

## Hardware Required

* One 802.15.4 enabled development board (e.g., ESP32-H2 or ESP32-C6) running this example.
* One Zigbee modem or pre-existing network

## Preparations

Clone the [esp-idf-bmx280](https://github.com/utkumaden/esp-idf-bmx280) repository into the `components` directory.

## Configure the project

Before project configuration and build, make sure to set the correct chip target using `idf.py set-target TARGET` command.

## Erase the NVRAM

Before flash it to the board, it is recommended to erase NVRAM if user doesn't want to keep the previous examples or other projects stored info
using `idf.py -p PORT erase-flash`

## Build and Flash

Build the project, flash it to the board, and start the monitor tool to view the serial output by running `idf.py -p PORT flash monitor`.

(To exit the serial monitor, type ``Ctrl-]``.)

## Application Functions

- When the program starts, the board will attempt to detect an available Zigbee network every **1 second** until one is found.

```
I (420) main_task: Calling app_main()
I (440) phy: phy_version: 211,0, 5857fe5, Nov  1 2023, 11:31:09
I (440) phy: libbtbb version: ce629d6, Nov  1 2023, 11:31:19
I (450) main_task: Returned from app_main()
I (580) ESP_ZB_TEMP_SENSOR: ZDO signal: ZDO Config Ready (0x17), status: ESP_FAIL
I (580) ESP_ZB_TEMP_SENSOR: Initialize Zigbee stack
I (580) temperature_sensor: Range [-10°C ~ 80°C], error < 1°C
I (590) gpio: GPIO[9]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:2
I (600) ESP_ZB_TEMP_SENSOR: Device started up in  factory-reset mode
I (600) ESP_ZB_TEMP_SENSOR: Start network steering
I (3720) ESP_ZB_TEMP_SENSOR: ZDO signal: NLME Status Indication (0x32), status: ESP_OK
I (8730) ESP_ZB_TEMP_SENSOR: Joined network successfully (Extended PAN ID: 74:4d:bd:ff:fe:60:2d:57, PAN ID: 0xbcc7, Channel:13, Short Address: 0xe6d0)
```

- If the board is on a network, it acts as a Zigbee End Device with the `Home Automation Temperature Sensor` function

- The board updates the temperature attribute of the `Temperature Measurement` cluster based on the actual temperature sensor value of the chip.
- The board updates the temperature attribute of the `Humidity Measurement` cluster based on the actual humidity sensor value of the chip.
