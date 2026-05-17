# SatAlign ESP32 V3

**SatAlign ESP32 V3** is an ESP32-based controller for manual and semi-automatic alignment of a Selfsat satellite antenna while travelling.

The project combines elevation motor control, DiSEqC-based azimuth movement, sensor feedback, RF signal evaluation, a local TFT display, a mobile web interface, and OTA updates into one compact DIY system. It is intended for hobbyists and makers who want to understand, modify, and improve their own mobile satellite alignment setup.

## Project idea

The idea behind SatAlign came from a practical problem: when travelling with a van or camper, aligning a satellite antenna by hand can be slow and frustrating. Commercial automatic systems exist, but this project was built as a personal DIY solution adapted to the author’s own hardware, travel setup, and learning process.

The system is not an industrial product and not a ready-made kit. It is a personal open DIY project that grew over several years through testing, rebuilding, debugging, documenting, and improving.

The project was inspired in part by existing antenna-rotor and motor control concepts, then developed into a dedicated ESP32-based version for a Selfsat flat antenna.

## Main functions

- Manual azimuth movement to the east and west through a DiSEqC-compatible satellite motor setup
- Manual elevation / angle adjustment
- Center alignment using Hall sensors
- Search routine for satellite signal detection
- RF-based signal evaluation using an AD8317 RF detector
- Candidate confirmation workflow using PLUS / MINUS buttons
- TFT display for local operation
- Mobile web UI for control, diagnosis, and troubleshooting
- OTA update support over Wi-Fi
- GitHub-safe configuration using a local `secrets.h` file

## Hardware overview

Typical hardware used in this project:

- ESP32 development board
- Selfsat flat satellite antenna
- DiSEqC-compatible satellite motor for azimuth movement
- Elevation linear actuator for angle adjustment
- Motor driver for the elevation actuator
- A3144 Hall sensors for Center, East limit, and West limit
- MPU6050 / GY-521 sensor for elevation angle measurement
- AD8317 1M–10GHz 60dB RF detector
- SAT splitter
- DC blocker, F-plug to F-socket
- F-plug to SMA adapter / 50 Ohm RG316 extension cable
- ST7735 1.44 inch TFT display
- Push buttons for local operation
- Satellite receiver

> **Important:** The DC blocker must be installed before the AD8317 RF detector. It protects the RF detector from the 13 V / 18 V LNB supply voltage. Without the DC blocker, the AD8317 can be damaged.

## Azimuth and elevation control

SatAlign uses two different movement concepts.

### Azimuth

Azimuth movement is handled by a DiSEqC-compatible satellite motor.

SatAlign does not directly drive a separate DC motor for azimuth movement. The azimuth axis depends on the connected DiSEqC motor setup and must be wired, configured, and tested carefully.

### Elevation

Elevation movement is controlled separately through the elevation actuator. The current elevation angle is measured with the MPU6050 / GY-521 sensor.

## RF signal logic

The RF detector output is evaluated inversely:

- lower ADC / voltage value = stronger RF signal
- higher ADC / voltage value = weaker or no usable RF signal

The project uses practical threshold values from outdoor testing to classify the signal roughly as weak, usable, good, or very good.

These values are not universal. They depend on the specific receiver, LNB, splitter, cable path, attenuation, and detector setup.

The current version uses RF values for signal detection, comparison, and candidate evaluation during the search workflow. It does not include a separate automatic signal optimization function.

## Basic operation

1. Switch on the satellite receiver.
2. Roughly point the antenna toward the south.
3. Power on the SatAlign controller.
4. Use the initial angle window to roughly set the elevation angle.
5. Use **Align / Center** to establish the azimuth center reference.
6. Start **Search**.
7. When a satellite candidate is found, check the TV picture.
8. Press **PLUS** if it is the correct satellite.
9. If it is the wrong satellite, press **MINUS** and continue searching.
10. Final validation and any mechanical fine adjustment remain the user's responsibility.

## Web UI

The web interface provides access to:

- Main menu
- Align / Center
- Search
- Manual control
- Status / Diagnosis
- Troubleshooting
- RF signal display
- Reset function

The web UI is designed for mobile use and mirrors the practical operation of the TFT display where appropriate.

## Local operation

The local TFT and buttons can be used without the web interface.

Typical button logic:

- **PLUS**: move/select/increase, depending on menu
- **MINUS**: move/select/decrease, depending on menu
- **MODE short**: confirm / start selected function
- **MODE long**: cancel / return, depending on state

## Configuration and secrets

Private data is not stored directly in the GitHub version.

Before compiling, copy:

```text
secrets.example.h
```

to:

```text
secrets.h
```

Then enter your local Wi-Fi and OTA data in `secrets.h`.

Example:

```cpp
#pragma once

static const char* WIFI_SSID = "YOUR_WIFI_NAME";
static const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
static const char* WIFI_HOSTNAME = "sat-tracker";

static const char* OTA_PASSWORD = "YOUR_OTA_PASSWORD";
```

`secrets.h` is intentionally ignored by Git and should not be uploaded to GitHub.

## Documentation

Additional documentation can be stored in the `docs/` folder, for example:

- complete project documentation
- quick-start guide
- wiring notes
- troubleshooting guide

Test and utility sketches should be placed in `tools/`, not in `docs/`.

## Install test sketch

A separate install test sketch can be used before commissioning the full project. It is intended to test the hardware elements individually, such as:

- buttons
- TFT display
- MPU6050 angle sensor
- RF detector
- DiSEqC azimuth movement
- Hall sensors
- center alignment
- elevation motor
- web / network availability

Recommended path:

```text
tools/SatAlign_ESP32_V3_InstallTest/
```

## Status

The current V3 project state has passed offline testing for:

- web UI navigation
- TFT synchronization
- manual motor control
- Hall sensor display and coloring
- align / center menu
- search without RF signal
- weak-signal red RF display
- RF status display
- reset and start window
- OTA / network diagnosis

Outdoor live testing with real satellite signal is still the key step for validating the final RF thresholds and the practical search behavior.

## License and reuse

This is a personal DIY project. You are welcome to study, modify, adapt, and improve the code for your own experiments and setups.

Use at your own risk. Always check your wiring, motor direction, DiSEqC motor behavior, RF path, DC blocker placement, and mechanical limits before operating the system.

## Credits

Project idea, practical testing, hardware decisions, and requirements: **Hans-Peter Voß**

Programming support, comments, and documentation assistance: **ChatGPT GPT-5.5**
