# SatAlign ESP32 V3 - GitHub-sichere Projektversion

Diese Version ist fuer GitHub vorbereitet.

## Was bleibt im Projekt?

Alle technischen Mess-, Kalibrier- und Projektwerte bleiben im Code, zum Beispiel:

- RF-Grenzwerte aus den Aussentests
- Signaloptimierungs-Schrittwerte
- Winkel-/Offsetwerte
- Softlimits
- Pinbelegung
- Web-UI- und TFT-Logik

## Was wird nicht veroeffentlicht?

Private Zugangsdaten werden nicht im Repository gespeichert:

- WLAN-SSID
- WLAN-Passwort
- OTA-Passwort
- personenbezogene User-Daten
- feste private IP-Adressen

## Lokale Einrichtung

1. `secrets.example.h` kopieren.
2. Kopie in `secrets.h` umbenennen.
3. Eigene Daten eintragen.
4. `secrets.h` nicht committen. Die Datei ist in `.gitignore` eingetragen.

## Hinweis

Vor einem Push kann man lokal pruefen:

```bash
grep -R "projektspezifisches OTA-Passwort" .
grep -R "WIFI_PASSWORD" .
grep -R "192.168" .
```

Die Beispielplatzhalter in `secrets.example.h` sind absichtlich neutral.


## Lokaler Compile

Diese ZIP enthaelt eine neutrale `secrets.h`, damit der Sketch grundsaetzlich kompiliert.
Vor dem Upload auf den ESP32 muessen dort die eigenen WLAN- und OTA-Daten eingetragen werden.

`WEB_SERVER_PORT` ist kein Geheimnis und steht in `config.h` auf Port 80.
