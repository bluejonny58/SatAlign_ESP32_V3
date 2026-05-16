#pragma once

#include <Arduino.h>

// Initialisiert die WLAN-Verbindung des ESP32.
//
// Typischer Ablauf intern:
// - WLAN-Modus setzen
// - Hostname setzen
// - Verbindung mit SSID und Passwort starten
// - auf erfolgreiche Verbindung warten
//
// Diese Funktion sollte einmal in setup() aufgerufen werden.
void wifiInit();

// Muss zyklisch in loop() aufgerufen werden.
//
// Aufgabe:
// - Verbindung überwachen
// - bei Verbindungsverlust Reconnect-Versuche auslösen
//
// Dadurch bleibt die Weboberfläche auch dann wieder erreichbar,
// wenn das WLAN kurzzeitig ausfällt.
void wifiLoop();

// Liefert true, wenn aktuell eine gültige WLAN-Verbindung besteht.
//
// Praktisch nützlich für:
// - Diagnose
// - spätere Statusanzeigen
// - eventuelle Web-/Netzwerklogik
bool wifiIsConnected();

// Liefert die aktuell zugewiesene IP-Adresse als Text.
// Wenn keine WLAN-Verbindung besteht, wird "keine Verbindung" geliefert.
String wifiGetIpString();

// Liefert den Namen des aktuell verbundenen WLANs oder einen Hinweis,
// falls keine Verbindung besteht.
String wifiGetConnectedSsid();

// Liefert die aktuelle WLAN-Signalstaerke als Text, z. B. "-55 dBm".
// Wenn keine Verbindung besteht, wird "-" geliefert.
String wifiGetRssiString();
