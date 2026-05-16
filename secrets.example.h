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

// V3: Zwei bekannte Netzwerke.
// Die Anlage scannt die sichtbaren WLANs und verbindet sich mit dem bekannten
// Netzwerk mit der besseren Signalstaerke. Bei gleicher Signalstaerke bleibt
// diese Reihenfolge die Prioritaet.
static const WifiCredential WIFI_NETWORKS[] = {
  { "WLAN_NAME_1", "WLAN_PASSWORT_1" },
  { "WLAN_NAME_2", "WLAN_PASSWORT_2" }
};

static const int WIFI_NETWORK_COUNT =
  sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// Hostname im lokalen Netzwerk, z. B. http://sat-tracker.local
static const char* WIFI_HOSTNAME = "sat-tracker";

// OTA-Passwort fuer Updates ueber die Arduino IDE / Netzwerk-Port.
static const char* OTA_PASSWORD = "OTA_PASSWORT_EINTRAGEN";
