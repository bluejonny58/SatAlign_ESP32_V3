#pragma once

/*
  SatAlign V3 - lokale Zugangsdaten / Secrets
  ------------------------------------------------------------
  Diese Beispieldatei darf auf GitHub liegen.

  Vorgehen:
  1. Diese Datei kopieren.
  2. Die Kopie in secrets.h umbenennen.
  3. Eigene WLAN- und OTA-Daten eintragen.

  WICHTIG:
  secrets.h enthaelt private Zugangsdaten und soll nicht auf GitHub hochgeladen
  werden. Die Datei ist in .gitignore eingetragen.
*/

static const char* WIFI_SSID = "DEIN_WLAN_NAME";
static const char* WIFI_PASSWORD = "DEIN_WLAN_PASSWORT";
static const char* WIFI_HOSTNAME = "sat-tracker";

static const char* OTA_PASSWORD = "DEIN_OTA_PASSWORT";
