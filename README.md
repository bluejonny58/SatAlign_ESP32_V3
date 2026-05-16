# SatAlign ESP32 V3

**SatAlign ESP32 V3** is an ESP32-based controller for manual and semi-automatic alignment of a Selfsat satellite antenna while travelling.

The project combines motor control, sensor feedback, RF signal evaluation, a local TFT display, a mobile web interface, and OTA updates into one compact DIY system. It is intended for hobbyists and makers who want to understand, modify, and improve their own mobile satellite alignment setup.

## Project idea

The idea behind SatAlign came from a practical problem: when travelling with a van or camper, aligning a satellite antenna by hand can be slow and frustrating. Commercial automatic systems exist, but this project was built as a personal DIY solution adapted to the author’s own hardware, travel setup, and learning process.

The system is not an industrial product and not a ready-made kit. It is a personal open DIY project that grew over several years through testing, rebuilding, debugging, documenting, and improving.

The project was inspired in part by existing antenna-rotor and stepper-motor control concepts, then developed into a dedicated ESP32-based version for a Selfsat flat antenna.

## Main functions

- Manual azimuth movement to the east and west
- Manual elevation / angle adjustment
- Center alignment using Hall sensors
- Search routine for satellite signal detection
- RF-based signal evaluation using an AD8317 RF detector
- Signal optimization after user confirmation
- TFT display for local operation
- Mobile web UI for control, diagnosis, and troubleshooting
- OTA update support over Wi-Fi
- GitHub-safe configuration using a local `secrets.h` file

## Basic operation

1. Switch on the satellite receiver.
2. Roughly point the antenna toward the south.
3. Power on the SatAlign controller.
4. Use the initial angle window to roughly set the elevation angle.
5. Use **Align / Center** to establish the azimuth center reference.
6. Start **Search**.
7. When a satellite candidate is found, check the TV picture.
8. Press **PLUS** if it is the correct satellite.
9. The system then starts signal optimization.
10. If it is the wrong satellite, press **MINUS** and continue searching.

## Web UI

The web interface provides access to:

- Main menu
- Align / Center
- Search
- Manual control
- Status / Diagnosis
- Troubleshooting
- Signal optimization display
- Reset function

The web UI is designed for mobile use and mirrors the practical operation of the TFT display where appropriate.

## Documentation

Additional documentation can be stored in the `docs/` folder, for example:

- complete project documentation
- quick-start guide
- wiring notes
- troubleshooting guide

## Install test sketch

A separate install test sketch can be used before commissioning the full project. It is intended to test the hardware elements individually, such as:

- buttons
- TFT display
- MPU6050 angle sensor
- RF detector
- azimuth motor
- Hall sensors
- center alignment
- elevation motor
- web / network availability

## Status

The current V3 project state has passed offline testing for:

- web UI navigation
- TFT synchronization
- manual motor control
- Hall sensor display and coloring
- align / center menu
- search without RF signal
- weak-signal red RF display
- optimization page display
- reset and start window
- OTA / network diagnosis


## License and reuse

This is a personal DIY project. You are welcome to study, modify, adapt, and improve the code for your own experiments and setups.

Use at your own risk. Always check your wiring, motor direction, RF path, DC blocker placement, and mechanical limits before operating the system.

## Credits

Project idea, practical testing, hardware decisions, and requirements: **Hans-Peter Voss**

Programming support, comments, and documentation assistance: **ChatGPT GPT-5.5**
