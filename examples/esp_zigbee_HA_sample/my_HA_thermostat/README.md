| Supported Targets | ESP32-H2 | ESP32-C6 | ESP32-C5 |
| ----------------- | -------- | -------- | -------- |

# Thermostat (Light + Temperature Sensor) Example

This example combines a Home Automation on/off light endpoint and a temperature sensor endpoint on a Zigbee end device.

## Hardware Required

* One 802.15.4 enabled development board (e.g., ESP32-H2 or ESP32-C6) running this example.
* A second board running as a Zigbee coordinator (see [HA_on_off_switch](../HA_on_off_switch/) example).

## Configure the project

Before project configuration and build, make sure to set the correct chip target using `idf.py set-target TARGET` command.

## Erase the NVRAM

Before flash it to the board, it is recommended to erase NVRAM if user doesn't want to keep the previous examples or other projects stored info
using `idf.py -p PORT erase-flash`.

## Build and Flash

Build the project, flash it to the board, and start the monitor tool to view the serial output by running `idf.py -p PORT flash monitor`.

(To exit the serial monitor, type ``Ctrl-]``.)

## Application Functions

- The device exposes two endpoints:
  - On/Off Light (endpoint 10)
  - Temperature Sensor (endpoint 11)
- On startup, the device joins a Zigbee network and begins reporting temperature measurements.
- When the device receives an `On/Off` command, the LED on the board toggles accordingly.

## Troubleshooting

For any technical queries, please open an [issue](https://github.com/espressif/esp-zigbee-sdk/issues) on GitHub. We will get back to you soon.
