# SatAlign ESP32 V3 - GitHub Notes

## Release-Stand

**Aktueller Projektstand: V3.1.5**

Die bewaehrte AUTO-Basis entspricht dem praktisch getesteten Stand ohne experimentelle Peak-Rueckfahrt. V3.1.5 erweitert darauf aufbauend die manuelle Web-Feinjustierung. Die Firmwareversion wird in `config.h` ueber `FIRMWARE_VERSION = "3.1.5"` gefuehrt.

## Wichtig fuer Arduino

Der Ordnername und die Hauptdatei muessen identisch sein:

```text
SatAlign_ESP32_V3/
SatAlign_ESP32_V3.ino
```

Die Release-Version wird nicht ueber den Dateinamen, sondern ueber `FIRMWARE_VERSION` verwaltet.

## Was sich gegenueber dem alten GitHub-Stand geaendert hat

- Aktueller Projektstand auf **V3.1.5** aktualisiert; die bewaehrte AUTO-Basis von V3.1.4 bleibt unveraendert.
- feste GeniusBulli-IP `192.168.4.15` dokumentiert.
- mDNS aus dem Projekt entfernt; IP-basierter Aufruf ist der vorgesehene Weg.
- ElegantOTA (`/update`) und ArduinoOTA parallel dokumentiert.
- fruehere automatische Funktion `Signal optimieren` aus Bedienung/Dokumentation entfernt.
- RF-Auswertung fuer Anzeige und AUTO-Suche vereinheitlicht; V3.1.5 bewertet die Nutzerqualitaet prozentbasiert: <80 % schwach, 80-<85 % brauchbar, 85-<95 % gut, >=95 % sehr gut.
- RF-Anzeige auf max. 100 % begrenzt.
- AUTO-Kandidatenschwelle auf 80 % gesetzt.
- `RF_FILTER_ALPHA` auf 0.50 eingestellt.
- strukturiertes, bereinigtes AUTO-Serial-Logging dokumentiert.
- `AZPOS` als Diagnosewert, nicht als Encoderposition, klargestellt.
- Elevationskorrektur in `Mitte einstellen` dokumentiert: PLUS/MINUS erzeugen kurze 250-ms-Pulse; kein 10-Sekunden-Bootfenster.
- Manuelle Web-UI um separate AZ-/EL-Feinschritte erweitert: AZ 50 ms, EL 50 ms bei PWM 90; Pulszeiten zentral in `settings.cpp` parametrierbar, normale Fahrbuttons bleiben Dauerfahrt bis STOP.
- In V3.1.5 stehen `RF` und gespreiztes `OPT` direkt **zwischen** den beiden Feinbuttons; die relative Positionsanzeige folgt unmittelbar darunter. RF 80-100 % wird fuer `OPT` auf 0-100 % gespreizt. Die separate Signalbewertungs-/Diagnosekarte wurde von der manuellen Bedienseite entfernt.
- Relative Feinpositionsanzeige: Beim Oeffnen von `Manuell` wird die Referenz automatisch gesetzt. AZ zeigt `0 -> +/-n Schritte`; EL zeigt `Referenzwinkel -> aktueller Winkel (Aenderung)`. Ein separater Reset-Button ist nicht mehr erforderlich. Der technische Status ist auf Motorzustand AZ/EL, Hall-Sensoren sowie EL-Winkel mit Softlimits reduziert; doppelte Web-/Live-Zeilen entfallen.

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

- Die Seite `Suchen` zeigt fuer Nutzer primaer den echten RF-Prozentwert und die Qualitaetsstufe statt Spannung/ADC; Rohwerte bleiben auf Diagnose-/Feinjustierseiten verfuegbar.
