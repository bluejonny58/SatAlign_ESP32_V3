# SatAlign ESP32 V3

**ESP32-basierte manuelle und automatische Ausrichthilfe fuer eine mobile Selfsat-Flachantenne.**

SatAlign verbindet Azimut- und Elevationssteuerung, Hall-Sensorik, MPU6050-Winkelmessung, RF-Signalbewertung, TFT-Bedienung, mobile Web-UI sowie ArduinoOTA und ElegantOTA in einem modularen ESP32-Projekt.

## Aktueller stabiler Stand

**Aktueller Stand: V3.1.5**

V3.1.5 ist der aktuelle Projektstand fuer die Web-Feinjustierung. Die bewaehrte AUTO-Suchlogik aus V3.1.4 bleibt dabei unveraendert. Die Suchlogik basiert auf den praktisch getesteten AUTO-Abläufen ohne nachtraegliche Peak-Rueckfahrt. Experimentelle Peak-Suchvarianten aus spaeteren Tests wurden bewusst nicht uebernommen, weil die Rueckfahrt zum RF-Maximum mit der vorhandenen Mechanik/Positionsschaetzung nicht ausreichend reproduzierbar war.

### Wesentliche Merkmale von V3.1.5

- gemeinsame RF-Prozentbewertung fuer TFT, Web-UI, Diagnose und AUTO-Suche
- `RF_FILTER_ALPHA = 0.50` nach reproduzierbarem RF-/Azimut-Test
- normale AUTO-Kandidatenschwelle: **80 %**
- RF-Anzeige auf maximal **100 %** begrenzt
- dynamische RF-Referenz aus der Centerfahrt bleibt aktiv, darf die 80-%-Mindestgrenze aber nicht unterschreiten
- kompakter, kopierbarer AUTO-Diagnoseblock im seriellen Monitor
- feste IP **192.168.4.15** im Netzwerk `GeniusBulli`
- andere bekannte WLANs per DHCP
- ArduinoOTA und ElegantOTA parallel
- ElegantOTA ueber `/update`
- kein mDNS
- fruehere automatische Funktion **Signal optimieren** aus der Bedienlinie entfernt
- `BADSAT`-Logik zum Sperren bereits verworfener Kandidatenbereiche

## AUTO-Diagnose im seriellen Monitor

Bei einem vollstaendigen AUTO-Lauf erzeugt V3.1.5 einen strukturierten Block bei **115200 Baud**:

```text
# AUTO_LOG_BEGIN
# T_MS;STATE;DIR;RAW;FILTER;SIGNAL;DROP;ZERO_ADC;BEST_ADC;AZPOS;EZ;H_C;H_E;H_W;MIN_RF;CENTER_MIN;BLOCKED
AUTO_LOG;...
...
# AUTO_LOG_RESULT;...
# AUTO_LOG_END
```

Waehrend AUTO werden redundante periodische `RF_DIAG`, grosse `MODE=...`-Statuszeilen und doppelte RF-Diagnosen unterdrueckt. Wichtige Ereignisse - Kandidat, `BADSAT`, Richtungswechsel, Hall-/Limit-Ereignisse, Fehler und Abschluss - bleiben sichtbar.

`AZPOS` ist dabei ein Diagnosewert und kein Encoderwert. Er darf nicht als absolut reproduzierbare mechanische Position verstanden werden.

## RF-Signalbewertung

Beim verwendeten AD8317/AD8318-Aufbau gilt:

- kleinerer ADC-/Spannungswert = staerkeres RF-Signal
- groesserer Prozentwert = staerkeres RF-Signal

Aktuelle zentrale Werte:

```cpp
RF_FILTER_ALPHA = 0.50f;
RF_WEAK_REFERENCE_ADC = 1883.0f;
RF_STRONG_REFERENCE_ADC = 700.0f;
AUTO_RF_MIN_CANDIDATE_PERCENT = 80.0f;
```

Die gemeinsame RF-Prozentbewertung wird auf den normalen Bereich von **0 bis 100 %** begrenzt. Werte, die rechnerisch oberhalb der starken Referenz liegen, werden daher als **100 %** dargestellt. Die 80-%-Grenze bleibt die operative Mindestgrenze fuer normale AUTO-Kandidaten; ein Wert unterhalb davon soll nicht als normaler AUTO-Kandidat akzeptiert werden.

Seit V3.1.5 basiert auch die fuer Nutzer sichtbare Qualitaetsstufe konsequent auf diesem Prozentwert: **< 80 % = schwach**, **80 bis < 85 % = brauchbar**, **85 bis < 95 % = gut**, **ab 95 % = sehr gut**. Auf der Seite **Suchen** stehen deshalb Prozentwert und Qualitaet im Vordergrund; ADC und Spannung bleiben Diagnosewerte fuer Status/Feinjustierung.

## Lokale Grundeinstellung / Elevation

Nach dem Einschalten startet SatAlign im Hauptmenue. Fuer die grobe Elevationskorrektur wird **Mitte einstellen / Ausrichten** geoeffnet. **PLUS/MINUS** bewegen die Elevation dort nicht dauerhaft, sondern jeweils in einem kurzen Puls von etwa **250 ms**. Wird eine Taste laenger als etwa **800 ms** festgehalten, stoppt die Bewegung und die Tastensperre bleibt bis zum Loslassen aktiv. Ein frueheres 10-Sekunden-Bootfenster fuer die Elevationskorrektur ist im aktuellen V3.1.5-Code nicht mehr vorhanden. **MODE** startet anschliessend die Mittenfahrt.

## Suchablauf

Der automatische Ablauf arbeitet praxisorientiert:

1. Center-/Mittenreferenz herstellen bzw. pruefen.
2. RF waehrend der Mittenfahrt beobachten.
3. Bei einem stabilen brauchbaren Kandidaten anhalten und zur Benutzerpruefung anbieten.
4. Wird kein Kandidat gefunden, die vorgesehene Ost-/West-Suche fortsetzen.
5. **PLUS** bestaetigt den richtigen Satelliten.
6. **MINUS** verwirft den Kandidaten; der Bereich wird fuer die weitere Suche gesperrt (`BADSAT`).

Eine nachtraegliche automatische Peak-Rueckfahrt ist im stabilen Stand V3.1.4 **nicht** enthalten.

## WLAN und OTA

### GeniusBulli

Im Fahrzeug verwendet SatAlign:

- IP: `192.168.4.15`
- Gateway: `192.168.4.1`
- DNS: `192.168.4.1`
- Subnetz: `255.255.255.0`

Bei anderen bekannten WLANs wird DHCP verwendet. Die Reihenfolge in `WIFI_NETWORKS[]` bestimmt die Verbindungsprioritaet.

### Firmware-Updates

Zwei Updatewege sind vorgesehen:

- **ArduinoOTA** fuer Uploads direkt aus der Arduino IDE (Port 3232)
- **ElegantOTA** fuer Browser-Updates ueber `http://192.168.4.15/update`

mDNS wird nicht verwendet. Fuer Web-UI und OTA sollte die IP-Adresse genutzt werden.

## Konfiguration und Zugangsdaten

Private Zugangsdaten gehoeren nicht in das Repository. Vor dem Kompilieren:

1. `secrets.example.h` nach `secrets.h` kopieren.
2. WLAN- und OTA-Daten lokal eintragen.
3. `secrets.h` nicht committen.

Die `.gitignore` ist darauf ausgelegt, lokale Zugangsdaten vom GitHub-Repository fernzuhalten.

## Hardware-Uebersicht

Typischer Projektaufbau:

- ESP32 Development Board
- Selfsat-Flachantenne
- DiSEqC-kompatibler Azimut-Antrieb / Tastensimulation
- Elevations-Linearantrieb mit L298N
- MPU6050 / GY-521
- A3144 Hall-Sensoren fuer Center, Ost und West
- AD8317/AD8318 RF-Detector
- Sat-Splitter / Messabzweig
- **DC-Blocker vor dem RF-Detector**
- ST7735 1,44-Zoll-TFT
- MODE / PLUS / MINUS Taster
- Sat-Receiver

> **Wichtig:** Der DC-Blocker muss vor dem RF-Detector sitzen. Die 13-V-/18-V-LNB-Versorgung des Receivers darf nicht am RF-Detector anliegen.

## Projektstruktur

```text
SatAlign_ESP32_V3/
├── SatAlign_ESP32_V3.ino
├── config.h
├── pins.h
├── settings.cpp / settings.h
├── types.h
├── control_state.cpp / .h
├── azimuth_control.cpp / .h
├── elevation_control.cpp / .h
├── sensor_mpu6050.cpp / .h
├── rf_detector.cpp / .h
├── display_ui.cpp / .h
├── live_runtime.cpp / .h
├── web_server.cpp / .h
├── wifi_manager.cpp / .h
├── ota_manager.cpp / .h
├── wifi_config.h
├── secrets.example.h
├── tools/
│   └── SatAlign_ESP32_V3_InstallTest/
│       └── SatAlign_ESP32_V3_InstallTest.ino
├── docs/
├── images/
└── STL/
```

Arduino-Anforderung: Der Projektordner und die Hauptdatei muessen denselben Namen tragen. Im Repository sind das deshalb `SatAlign_ESP32_V3/` und `SatAlign_ESP32_V3.ino`. Die Firmwareversion steht unabhaengig davon in `FIRMWARE_VERSION` und ist aktuell `3.1.5`.

## InstallTest / Hardwarediagnose

Im Repository bleibt der eigenstaendige Hardwaretest unter

```text
tools/SatAlign_ESP32_V3_InstallTest/SatAlign_ESP32_V3_InstallTest.ino
```

erhalten. Der InstallTest ist **nicht** die normale SatAlign-Firmware, sondern ein Diagnosewerkzeug fuer Erstinbetriebnahme und Hardwareverdacht. Er eignet sich insbesondere, um Taster, TFT, Hall-Sensoren, MPU6050, RF-Detector sowie die Azimut- und Elevationsausgaenge getrennt von der AUTO-Logik zu pruefen.

Wichtig: Auch beim InstallTest gilt die Arduino-Regel, dass Sketch-Datei und Sketch-Ordner denselben Namen tragen. Deshalb liegt die Datei im Unterordner `SatAlign_ESP32_V3_InstallTest/`. Nach dem Hardwaretest muss fuer den normalen Betrieb wieder `SatAlign_ESP32_V3.ino` geflasht werden.

## Web-UI

Die Weboberflaeche bietet unter anderem:

- Hauptmenue
- Ausrichten / Center
- automatische Suche
- manuelle Steuerung mit zusaetzlichen AZ-/EL-Feinschritten fuer die letzte RF-Optimierung
- Status / Diagnose
- Firmware Update (ElegantOTA)

Die Versionsnummer wird auf der Hauptseite angezeigt.

### Manuelle Feinkorrektur

Unter den normalen AZ-/EL-Fahrbuttons stehen zusaetzliche **Fein - / Fein +**-Buttons. Sie starten keine Dauerfahrt, sondern nur sehr kurze Einzelpulse fuer die letzte Optimierung des RF-Maximums. Der aktuelle Arbeitsstand verwendet:

- Azimut-Feinschritt: `WEB_AZ_FINE_PULSE_MS = 50 ms`
- Elevations-Feinschritt: `WEB_EL_FINE_PULSE_MS = 50 ms`
- Elevations-Fein-PWM: `WEB_EL_FINE_PWM = 90`

Bei jeder Achse steht der **aktuelle RF-/OPT-Wert direkt zwischen den beiden Feinbuttons**. Beim Oeffnen der Seite **Manuell** wird automatisch eine neue Referenz fuer die Feinjustierung gesetzt. Bei AZ ist die Referenz `0 Schritte`; darunter wird die relative Position als `0 -> +/-n Schritte` angezeigt. Bei EL wird der aktuelle MPU-Winkel als Referenz gespeichert und danach `Referenz -> aktueller Winkel (Aenderung)` angezeigt. Separate Reset-Buttons und die Puls-/PWM-Werte werden in der Bedienansicht nicht mehr gezeigt.

OPT spreizt den relevanten RF-Bereich von 80 bis 100 % linear auf 0 bis 100 % und dient ausschliesslich als Anzeige-Lupe. Eine separate Signalbewertungs-/Diagnosekarte wurde von der manuellen Bedienseite entfernt; die Qualitaetsgrenzen und technischen RF-Rohwerte bleiben in Dokumentation bzw. Diagnosekontext erhalten.

Auch der technische Status wurde reduziert: doppelte Angaben fuer **Web-Zustand** und **Live-Zustand** entfallen. Die Diagnosekarte zeigt nur noch den aktuellen Motorzustand von AZ/EL, die Hall-Sensoren sowie den aktuellen EL-Winkel zusammen mit den Softlimits. Die Feinpulszeiten sind zentral in `settings.cpp` parametrierbar; V3.1.5 startet wegen der mechanischen Untersetzung bewusst mit sehr kleinen 50-ms-Schritten auf beiden Achsen.

## Sicherheit und Grenzen

- Manuelle Steuerung ist ein Override und muss beobachtet werden.
- Softwarelimits ersetzen keine mechanischen Endbegrenzungen.
- Hall-Sensoren muessen vor der ersten Nutzung einzeln geprueft werden.
- RF-Schwellen sind auf den realen Aufbau kalibriert und koennen sich bei anderer Verkabelung, Daempfung oder anderem RF-Detector verschieben.
- Der Sat-Receiver muss eingeschaltet sein, damit der RF-Messzweig ein verwertbares Signal sieht.
- `AZPOS` ist nur eine softwareseitige Diagnoseposition und kein echter Encoder.

## Versionshinweise

### V3.1.5
- Manuelle Web-UI fuer die Feinjustierung neu angeordnet.
- RF und gespreizter OPT-Wert stehen direkt zwischen `Fein -` und `Fein +`.
- Relative Positionsanzeige steht unmittelbar unter den Feinbuttons: AZ zeigt `0 -> +/-n Schritte`, EL zeigt `Referenzwinkel -> aktueller Winkel (Aenderung)`.
- Die Referenz wird beim Oeffnen der manuellen Seite automatisch neu gesetzt; Reset-Button sowie Puls-/PWM-Angaben entfallen aus der Bedienansicht.
- Technischen Status weiter verdichtet: keine doppelten Web-/Live-Zeilen mehr; nur Motorzustand AZ/EL, Hall-Sensoren sowie EL-Winkel mit Softlimits bleiben sichtbar.
- AUTO-Suchlogik, 80-%-Kandidatenschwelle, RF-Normierung und Motor-Richtungszuordnung unveraendert.

### V3.1.4
- RF-Filter von 0.75 auf **0.50** geaendert.
- RF-Obergrenze von **95 %** auf **100 %** angehoben.
- 80-%-AUTO-Mindestgrenze beibehalten.
- gemeinsame RF-Prozentbewertung beibehalten.
- kompaktes AUTO-Protokoll fuer kopierbare Feldtests eingefuehrt und anschliessend bereinigt.
- keine Aenderung der bewaehrten Suchlogik durch die Protokollbereinigung.
- Web-UI um sehr kleine AZ-/EL-Feinschritte und eine Live-RF-Anzeige fuer die abschliessende Signaloptimierung erweitert.

### V3.1.3
- operative AUTO-Mindestgrenze auf **80 %** angehoben.

### V3.1.2
- RF-Kalibrierung weiter gespreizt; starker Referenzwert auf **700 ADC** gesetzt.
- nutzbare Prozentanzeige auf **95 %** gedeckelt.

### V3.1.1
- ElegantOTA zusaetzlich zu ArduinoOTA.
- RF-Prozentbewertung fuer Anzeige und Suchlogik vereinheitlicht.

## Lizenz / Nutzung

Persoenliches Open-DIY-Projekt. Anpassung, Weiterentwicklung und eigene Versuche sind ausdruecklich erwuenscht. Nutzung auf eigenes Risiko; keine Gewaehrleistung fuer Verdrahtung, RF-Anschluss, Motorbewegung oder mechanische Sicherheit.

## Credits

Projektidee, Aufbau, praktische Tests und Anforderungen: **Hans-Peter Voss**  
Programmierung, Kommentare und Dokumentationsunterstuetzung: **ChatGPT**
