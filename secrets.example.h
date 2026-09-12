#pragma once

/*
  SatAlign V3 - Beispiel fuer lokale Zugangsdaten
  ------------------------------------------------------------
  Diese Datei darf auf GitHub liegen, weil sie nur Platzhalter enthaelt.

  Fuer den echten Betrieb:
  1. Diese Datei kopieren.
  2. Die Kopie in secrets.h umbenennen.
  3. Eigene WLAN- und OTA-Daten in secrets.h eintragen.

  WICHTIG:
  secrets.h bleibt lokal und wird durch .gitignore nicht zu GitHub hochgeladen.
*/

struct WifiCredential {
  const char* ssid;
  const char* password;
};

// Bekannte Netzwerke in fester Prioritaetsreihenfolge.
// Der erste sichtbare Eintrag wird verwendet. Dadurch kann z. B. das
// GeniusBulli-Netz vor einem gleichzeitig sichtbaren Test-WLAN bevorzugt werden.
static const WifiCredential WIFI_NETWORKS[] = {
  { "GeniusBulli", "BULLI_PASSWORT_EINTRAGEN" },
  { "GeniusHome",  "HOME_PASSWORT_EINTRAGEN" },
  { "Genius Home", "HOME_PASSWORT_EINTRAGEN" },
  { "WLAN_NAME_4", "WLAN_PASSWORT_4" }
};

static const int WIFI_NETWORK_COUNT =
  sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// OTA-Hostname fuer ArduinoOTA. mDNS/Web-Aufruf per .local wird im Projekt nicht verwendet.
static const char* WIFI_HOSTNAME = "sat-tracker";

// OTA-Passwort fuer Updates ueber die Arduino IDE / Netzwerk-Port.
static const char* OTA_PASSWORD = "OTA_PASSWORT_EINTRAGEN";
