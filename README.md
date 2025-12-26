<p align="center"><img src="./app/assets/icon.svg" width="256" /></p>

# motoheato

This project uses an ESP32C6 to automatically control the heated grips and heated seat of a motorcycle.

It monitors the oil pressure switch and/or voltage to only run when the engine is running, with a short delay before heating start to give the engine and battery a moment to stabilize after start. When using the oil pressure switch it will disable heating instantaneously when the engine stops for any reason to avoid any drain on the battery.

A temperature sensor is used to monitor either the air or the handlebar teemperature to decide the required preheat period and power output.

All parameters can be monitored and configured over BLE using the flutter app in the app folder.

# Building

## Firmware

```sh
./setup.sh

# Just building
./build.sh

# Flash
./build.sh flash
```

## App

Install flutter, android-studio and android-sdk and make sure `ANDROID_HOME` is set. Then connect your phone with USB debugging enabled:

```sh
flutter run --release
```

# Connections

Current connections on the ESP32C6:

pin    | description
-------|------------
GPIO18 | grip left (active high)
GPIO20 | grip right (active high)
GPIO19 | heated seat (active high)
GPIO17 | DS18B20 external temperature sensor
GPIO0  | voltage sense through 33k/6.8k voltage divider
GPIO1  | oil pressure sense (active low with pullup, protected by diode)

3.3V is supplied through a small buck converter, which is powered by the accessory line.

# Vibe coding notes

This project was mostly coded by gemini-3-flash-preview. Some notes:

1. Gemini is terrible at adhering to any kind of code style, in the end gave up and stuck to clang-format/dart format.
2. It's allergic to readability: It randomly removes comments (both yours and its own) and sometimes gets so allergic to newlines that it rewrites entire code blocks to be super long lines. This is despite working above the compression threshold.
3. It insists on using the `write_file` tool, overwriting any changes made by yourself even if you tell it that you changed the file, and will insist on undoing your changes on every future edit even though you explicitly informed it of its mistake.
4. In using `write_file`, it often makes accidental and erroneous edits to entirely unrelated parts of the file which it seems unaware of, likely an error when recalling the current file content "from memory".
5. Can still get entirely derailed to the point of complete failure by trivial tasks like fixing an app icon.

It helped throw stuff together, but definitely still not something that can be left to its own devices.
