# SatAlign ESP32 V3

**ESP32-basierte manuelle und automatische Ausrichthilfe fuer eine mobile Selfsat-Flachantenne.**

SatAlign verbindet Azimut- und Elevationssteuerung, Hall-Sensorik, MPU6050-Winkelmessung, RF-Signalbewertung, TFT-Bedienung, mobile Web-UI sowie ArduinoOTA und ElegantOTA in einem modularen ESP32-Projekt.

## Aktueller stabiler Stand

**Stable Release: V3.1.4**

V3.1.4 ist der aktuell empfohlene Projektstand. Die Suchlogik basiert auf den praktisch getesteten AUTO-Abläufen ohne nachtraegliche Peak-Rueckfahrt. Experimentelle Peak-Suchvarianten aus spaeteren Tests wurden bewusst nicht uebernommen, weil die Rueckfahrt zum RF-Maximum mit der vorhandenen Mechanik/Positionsschaetzung nicht ausreichend reproduzierbar war.

### Wesentliche Merkmale von V3.1.4

- gemeinsame RF-Prozentbewertung fuer TFT, Web-UI, Diagnose und AUTO-Suche
- `RF_FILTER_ALPHA = 0.50` nach reproduzierbarem RF-/Azimut-Test
- normale AUTO-Kandidatenschwelle: **80 %**
- RF-Anzeige auf maximal **95 %** begrenzt
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

Bei einem vollstaendigen AUTO-Lauf erzeugt V3.1.4 einen strukturierten Block bei **115200 Baud**:

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

Die Prozentanzeige wird bewusst bei **95 %** gedeckelt. Die 80-%-Grenze ist die operative Mindestgrenze fuer normale AUTO-Kandidaten; ein Wert unterhalb davon soll nicht als normaler AUTO-Kandidat akzeptiert werden.

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

Arduino-Anforderung: Der Projektordner und die Hauptdatei muessen denselben Namen tragen. Im Repository sind das deshalb `SatAlign_ESP32_V3/` und `SatAlign_ESP32_V3.ino`. Die Firmwareversion steht unabhaengig davon in `FIRMWARE_VERSION` und ist aktuell `3.1.4`.

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
- manuelle Steuerung
- Status / Diagnose
- Firmware Update (ElegantOTA)

Die Versionsnummer wird auf der Hauptseite angezeigt.

## Sicherheit und Grenzen

- Manuelle Steuerung ist ein Override und muss beobachtet werden.
- Softwarelimits ersetzen keine mechanischen Endbegrenzungen.
- Hall-Sensoren muessen vor der ersten Nutzung einzeln geprueft werden.
- RF-Schwellen sind auf den realen Aufbau kalibriert und koennen sich bei anderer Verkabelung, Daempfung oder anderem RF-Detector verschieben.
- Der Sat-Receiver muss eingeschaltet sein, damit der RF-Messzweig ein verwertbares Signal sieht.
- `AZPOS` ist nur eine softwareseitige Diagnoseposition und kein echter Encoder.

## Versionshinweise

### V3.1.4
- RF-Filter von 0.75 auf **0.50** geaendert.
- 80-%-AUTO-Mindestgrenze beibehalten.
- gemeinsame RF-Prozentbewertung beibehalten.
- kompaktes AUTO-Protokoll fuer kopierbare Feldtests eingefuehrt und anschliessend bereinigt.
- keine Aenderung der bewaehrten Suchlogik durch die Protokollbereinigung.

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
