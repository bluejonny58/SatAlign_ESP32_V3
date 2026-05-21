# SatAlign ESP32 V3 - KiCad PCB Documentation

This document shows the current KiCad-based PCB concept for SatAlign ESP32 V3.

The PCB version is now the active hardware basis for further development. The earlier breadboard/test wiring remains useful as a reference, but the firmware pinout must now follow the KiCad PCB assignment.

## Schematic overview

![KiCad schematic overview](images/kicad_schematic_overview.png)

The schematic shows the main electrical structure of the SatAlign controller:

- ESP32 mini development board as the central controller
- 12 V input with protected 12 V rail for the elevation motor driver supply
- Mini360 buck converter for the 5 V system supply
- 3.3 V rail from the ESP32 for logic-level pullups and sensors
- MPU6050 / GY-521 connection for elevation angle measurement
- AD8317 / AD8318 RF detector input for satellite signal evaluation
- Hall sensor connector for azimuth reference and limit detection
- Optocoupler interface for azimuth east/west control signals
- TFT display connector
- local button connector for MODE, PLUS and MINUS
- L298N connectors for elevation actuator control and motor power

The schematic intentionally reflects the PCB pinout. Some ESP32 pins differ from the earlier breadboard/test setup because the routing on the PCB is cleaner and mechanically more practical with this assignment.

## PCB layout overview

![KiCad PCB layout overview](images/kicad_pcb_layout_overview.png)

The PCB layout groups the main functional blocks in a way that keeps the wiring practical during assembly:

- ESP32 module in the central area
- display connector on the right side
- Hall sensor and MPU6050 connectors close to their external wiring direction
- RF detector connector near the analog input path
- L298N motor driver connectors separated from the ESP32 logic area
- 12 V input and protected 12 V path kept near the power section
- button connector placed near the control input section
- mounting holes placed at the board corners

The large keep-out area under the ESP32 module is intentional and should remain free of copper where required by the ESP32 module footprint and antenna/USB/mechanical clearance.

## Active PCB pinout note

The current PCB version uses the KiCad pin assignment as the active development basis. The firmware file `pins.h` must match this PCB layout.

Important PCB-related pin changes compared with the earlier breadboard/test wiring:

| Function | PCB GPIO | Note |
|---|---:|---|
| MPU SCL | GPIO26 | changed for PCB routing |
| Elevation IN1 | GPIO21 | changed for PCB routing |
| TFT CS | GPIO5 | changed for PCB routing |

The old breadboard/test pinout should only be kept as documentation. It should not be used as the active firmware basis for the PCB version.

## Manufacturing status

Before ordering the PCB, always run the following checks in KiCad:

- Electrical Rules Checker (ERC)
- Design Rules Checker (DRC)
- zone refill before DRC
- Gerber viewer inspection
- drill file inspection
- polarity/orientation check for diode, electrolytic capacitors and connectors
- final comparison between `pins.h`, schematic and PCB

The PCB files should only be considered production-ready after these checks have passed and the current hardware revision has been reviewed.
