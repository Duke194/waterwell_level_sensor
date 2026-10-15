# ESP32-S3 DevKitC-1

This page is the wiring and build for the `esp32-s3` environment in `platformio.ini`. MQTT, settings, and the status LED colors are in the [main README](../README.md).

## Hardware

* [ESP32-S3-DevKitC-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html)
* SR04M-2 waterproof ultrasonic module (JSN-SR04M-2, the same family as the AJ-SR04M)
* Four Dupont jumpers, 0.14 mm², about 0.2 m

USB powers the DevKit. The module takes 3.3 V from that board. Its supply range is 3.0–5.5 V and its working current is under 8 mA, so the DevKit regulator is enough. The supply pin is labeled `5V`. Fed at 3.3 V, the echo output stays at 3.3 V, so no divider is needed.

Leave R19 open. The firmware sends a 20 µs trigger pulse, which is mode 1 of this module. The 300 kΩ powersave resistor in the main README belongs to the battery build.

GPIO26 is not on the DevKitC-1 headers. It stays inside the module for the flash. The S3 build moves the trigger to GPIO4 with `-DTRIG_PIN=4` in `platformio.ini`. Echo stays on GPIO5. The `esp-wroom-32` build keeps GPIO26.

## Wiring

J1 pin numbers follow the [Espressif v1.1 user guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html): antenna at the top, USB sockets at the bottom, J1 on the left. The harness drawing shows the front view under the ESP32 pin table, with the four wires marked in the cable colors.

The SR04M-2 header is labeled `5V`, `RX`, `TX`, `GND`. In this pulse mode, `RX` is the trigger input and `TX` is the echo output.

| DevKit J1 | Signal | Color | SR04M-2 |
|---|---|---|---|
| Pin 1 | 3V3 | Red | 5V |
| Pin 4 | GPIO4, trigger | Yellow | RX |
| Pin 5 | GPIO5, echo | Green | TX |
| Pin 22 | GND | Black | GND |

![ESP32-S3 DevKitC-1 to SR04M-2](../wiring/esp32-s3.png)

The on-board RGB LED is not part of this harness. Early DevKitC-1 boards use GPIO48 for it, newer ones use GPIO38. The colors are listed in the main README.

## Build

`pio run` without an environment still builds `esp-wroom-32`. For this board:

```
pio run -e esp32-s3
pio run -t upload -e esp32-s3
```

Copy `include/settings.h.example` to `include/settings.h` and fill it in as described in the main README.

## Status LED

The firmware shows what the device is doing with its on-board LED. What you see depends on the board you build for:

  * `esp-wroom-32`: the plain LED on GPIO2. It blinks while the device connects to Wi-Fi, blinks for about 5 seconds after a successful OTA flash, and is off in every other state.
  * `esp32-s3`: the on-board RGB LED of the ESP32-S3 DevKitC-1. It is enabled by the build flag `-DSTATUS_LED_RGB` in `platformio.ini` and shows the colors below.

| Color | Meaning |
|---|---|
| Blue, blinking | Connecting to Wi-Fi |
| Red | No Wi-Fi connection |
| Cyan | Wi-Fi is up, connecting to the MQTT broker |
| Orange | Wi-Fi is up, but the MQTT broker is not reachable |
| Yellow | New firmware (OTA) is not confirmed yet. Open the web monitor so it can be confirmed, otherwise the previous firmware returns |
| White, short flash | A measurement is running |
| Magenta | DEV mode is on and the device is online |
| Green, blinking | Firmware image was accepted. The device reboots after about 5 seconds |
| Green | Everything is fine |
| Off | Deep sleep (battery mode) |

If several states apply at once, the one higher in the table wins, except for blue, the green blink before reboot, and white, which only show for a moment.

Notes:

  * The RGB LED runs at 10% brightness. Change it with `-DSTATUS_LED_BRIGHTNESS_PERCENT=<0-100>` in `build_flags`.
  * The LED is switched off before deep sleep, because an RGB LED keeps its last color otherwise and would drain the batteries.
  * The RGB pin comes from `RGB_BUILTIN` in the board definition. If your board revision has the LED on another pin and it stays dark, check that definition. Early DevKitC-1 revisions use GPIO48, newer ones use GPIO38.
  * The LED logic lives in `src/status_led.cpp`. To change a color, edit `colorFor()` there.
  * The Home Assistant entity "Status" is a remote copy of this LED. It shows the same text (`Online`, `Messung`, `Schlaeft`, `Firmware unbestaetigt`, and the other states). `Messung` stays up for at least a second so it is visible. States that only happen while the broker is unreachable (`WLAN verbindet`, `WLAN Fehler`, `MQTT verbindet`, `MQTT Fehler`) show up after the connection returns. The Wi-Fi blink and the green blink before reboot stay on the lamp and are not sent.

