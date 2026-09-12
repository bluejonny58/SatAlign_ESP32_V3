#pragma once

/*
  SatAlign V3 - WLAN-/OTA-Konfiguration
  ------------------------------------------------------------
  Diese Datei enthaelt keine echten Zugangsdaten.
  Die lokalen privaten Werte stehen in secrets.h.

  Fuer GitHub bleibt secrets.h in .gitignore ausgeschlossen.
  Fuer den lokalen Arduino-Compile muss secrets.h im Projektordner vorhanden sein.
*/

#include "secrets.h"

/*
  GeniusBulli-Netzwerk
  ------------------------------------------------------------
  Die Satellitensteuerung bekommt im Bulli eine feste Adresse, damit sie
  von der GeniusBulli-Zentrale und direkt im Browser immer unter derselben
  IP erreichbar ist.

  Fuer alle anderen bekannten WLANs (z. B. GeniusHome zum Testen) wird DHCP
  benutzt. Dadurch darf 192.168.4.15 niemals versehentlich im Heimnetz
  erzwungen werden, wenn dieses ein anderes IP-Subnetz verwendet.
*/
static const char* WIFI_BULLI_SSID = "GeniusBulli";

static const uint8_t WIFI_BULLI_IP[4]      = { 192, 168, 4, 15 };
static const uint8_t WIFI_BULLI_GATEWAY[4] = { 192, 168, 4, 1 };
static const uint8_t WIFI_BULLI_SUBNET[4]  = { 255, 255, 255, 0 };
static const uint8_t WIFI_BULLI_DNS[4]     = { 192, 168, 4, 1 };
