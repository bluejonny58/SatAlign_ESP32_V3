#pragma once

// Initialisiert ArduinoOTA.
//
// Voraussetzungen:
// - WLAN ist bereits initialisiert bzw. wifiInit() wurde aufgerufen
// - der ESP32 soll im lokalen Netz per OTA erreichbar sein
//
// OTA ist mit dem projektspezifischen Passwort aus secrets.h gesichert.
// Das ist nur im vertrauenswürdigen Heim-/Testnetz sinnvoll.
void otaInit();

// Muss zyklisch in loop() aufgerufen werden.
//
// Diese Funktion verarbeitet eingehende OTA-Events.
// Ohne diesen zyklischen Aufruf ist zwar OTA registriert,
// aber Uploads werden nicht angenommen.
void otaLoop();

// Liefert true, wenn OTA bereits initialisiert wurde.
bool otaIsReady();
