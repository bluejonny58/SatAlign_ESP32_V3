# SatAlign ESP32 V3 - GitHub Notes

## Release-Stand

**Aktuell empfohlene Version: V3.1.4**

Dieses Repository entspricht dem praktisch getesteten stabilen Stand ohne experimentelle Peak-Rueckfahrt. Die Firmwareversion wird in `config.h` ueber `FIRMWARE_VERSION = "3.1.4"` gefuehrt.

## Wichtig fuer Arduino

Der Ordnername und die Hauptdatei muessen identisch sein:

```text
SatAlign_ESP32_V3/
SatAlign_ESP32_V3.ino
```

Die Release-Version wird nicht ueber den Dateinamen, sondern ueber `FIRMWARE_VERSION` verwaltet.

## Was sich gegenueber dem alten GitHub-Stand geaendert hat

- Stable Baseline von v3.0.0 auf **V3.1.4** aktualisiert.
- feste GeniusBulli-IP `192.168.4.15` dokumentiert.
- mDNS aus dem Projekt entfernt; IP-basierter Aufruf ist der vorgesehene Weg.
- ElegantOTA (`/update`) und ArduinoOTA parallel dokumentiert.
- fruehere automatische Funktion `Signal optimieren` aus Bedienung/Dokumentation entfernt.
- RF-Auswertung fuer Anzeige und AUTO-Suche vereinheitlicht.
- RF-Anzeige auf max. 100 % begrenzt.
- AUTO-Kandidatenschwelle auf 80 % gesetzt.
- `RF_FILTER_ALPHA` auf 0.50 eingestellt.
- strukturiertes, bereinigtes AUTO-Serial-Logging dokumentiert.
- `AZPOS` als Diagnosewert, nicht als Encoderposition, klargestellt.

## GitHub-sichere Dateien

`secrets.h` ist lokal und darf **nicht** in GitHub landen. Im Repository liegt nur:

```text
secrets.example.h
```

Vor dem Build lokal kopieren und anpassen.

## Tools / InstallTest

Der Hardware-Diagnosesketch bleibt bewusst im Repository:

```text
tools/SatAlign_ESP32_V3_InstallTest/SatAlign_ESP32_V3_InstallTest.ino
```

Er dient der Erstinbetriebnahme und der Fehlersuche an Hardwarepfaden. Der InstallTest ist nicht die produktive Firmware und ersetzt `SatAlign_ESP32_V3.ino` nicht. Nach einem Test muss die Hauptfirmware wieder geflasht werden.

Typische Einsatzfaelle: Taster, TFT, Hall-Sensoren, MPU6050, RF-Detector, Azimut-Richtungen, STOP-Funktion und Elevationsantrieb einzeln pruefen, bevor ein Fehler in der AUTO-Suche vermutet wird.

## AUTO-Log fuer Fehleranalyse

Bei Problemen mit der automatischen Suche am besten den kompletten Bereich von

```text
# AUTO_LOG_BEGIN
```

bis

```text
# AUTO_LOG_END
```

kopieren. Dieser Block enthaelt die relevanten RF-, Richtungs-, Hall-, Elevations- und Kandidateninformationen, ohne die fruehere Flut redundanter periodischer Statuszeilen.

## Bewusst nicht enthalten

Die experimentellen Peak-Such-/Peak-Rueckfahrvarianten wurden nach Feldtests verworfen und gehoeren **nicht** zum stabilen Release. Sie sollten deshalb weder in `main` noch in der aktuellen Dokumentation als Funktion aufgefuehrt werden.
