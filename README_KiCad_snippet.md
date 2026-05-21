## KiCad PCB overview

The current SatAlign hardware development is based on a dedicated KiCad PCB. The PCB replaces the earlier breadboard/test wiring and is now the active basis for the firmware pinout.

![KiCad schematic overview](docs/images/kicad_schematic_overview.png)

![KiCad PCB layout overview](docs/images/kicad_pcb_layout_overview.png)

The PCB combines the ESP32 controller, RF detector input, MPU6050 angle sensor, Hall sensor inputs, TFT display connector, local buttons, elevation motor driver interface and optocoupler-based azimuth control interface.

Some ESP32 pins were intentionally changed compared with the earlier test wiring to make the PCB routing cleaner and mechanically more practical. The active firmware pinout is defined in `pins.h`.

More details are available in:

```text
docs/kicad_pcb_documentation.md
```
