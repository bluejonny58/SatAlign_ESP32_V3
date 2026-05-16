/*
  SatAlign V3 - MPU6050 API
  ------------------------------------------------------------
  Oeffentliche Funktionen fuer Winkelmessung, Kalibrierung und Filterwerte.
*/

#pragma once

#include <stdint.h>

// Initialisiert den MPU6050.
//
// Aufgaben:
// - I2C starten
// - MPU aus dem Sleep holen
// - Messbereiche konfigurieren
// - interne Laufvariablen zurücksetzen
//
// Rückgabewert:
// - true  = MPU antwortet und wurde erfolgreich initialisiert
// - false = Initialisierung fehlgeschlagen
bool initMPU6050();

// Prüft, ob der MPU6050 auf dem I2C-Bus erreichbar ist.
//
// Diese Funktion ist eine reine Verfügbarkeitsprüfung
// und noch keine vollständige Messinitialisierung.
bool mpuIsConnected();

// Liest die Rohdaten des MPU6050 aus.
//
// Ausgegeben werden:
// - Beschleunigung: ax, ay, az
// - Gyro: gx, gy, gz
//
// Rückgabewert:
// - true  = Daten erfolgreich gelesen
// - false = Lesezugriff fehlgeschlagen
bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz);

// Berechnet aus den Beschleunigungswerten einen reinen
// Beschleunigungs-Winkel für die Elevation.
//
// Dieser Winkel ist relativ rauschanfällig,
// aber driftfrei im Vergleich zum Gyro.
// Deshalb dient er im Komplementärfilter als Langzeitreferenz.
float mpuGetAccelElevationAngleDeg(int16_t ax, int16_t ay, int16_t az);

// -----------------------------------------------------
// Filter
// -----------------------------------------------------

// Setzt den internen Komplementärfilter auf einen definierten Startwert.
//
// Sinnvoll:
// - nach erfolgreicher Gyro-Kalibrierung
// - beim Neustart des Winkelfilters
// - vor neuen Referenzfahrten
void mpuResetFilter(float initialAngleDeg = 0.0f);

// Führt einen neuen Filter-Update-Schritt aus.
//
// Intern werden dabei:
// - neue Rohdaten gelesen
// - Beschleunigungswinkel berechnet
// - Gyro-Drehgeschwindigkeit berücksichtigt
// - der Komplementärfilter fortgeschrieben
//
// Rückgabewert:
// - true  = Update erfolgreich
// - false = Sensorwert konnte nicht gelesen werden
bool mpuUpdateFilteredAngle();

// Liefert den aktuell gefilterten Winkel.
// Das ist der zentrale interne Winkelwert des Sensors.
float mpuGetFilteredAngleDeg();

// Liefert den für die Anzeige bestimmten Winkel.
//
// Im Projekt wird hier zusätzlich ein Anzeige-Offset
// (DISPLAY_ANGLE_OFFSET_DEG) berücksichtigt.
float mpuGetDisplayedAngleDeg();

// Liefert den zuletzt berechneten reinen Beschleunigungswinkel.
float mpuGetLastAccelAngleDeg();

// Liefert die zuletzt berechnete Gyro-Drehgeschwindigkeit
// in Grad pro Sekunde.
float mpuGetLastGyroRateDegPerSec();

// -----------------------------------------------------
// Gyro-Kalibrierung
// -----------------------------------------------------

// Führt eine Gyro-Nullpunktkalibrierung durch.
//
// Dabei werden mehrere Rohmessungen gesammelt und gemittelt,
// um den Ruheoffset des Gyros zu bestimmen.
//
// Parameter:
// - samples = Anzahl der Messungen
// - delayMs = Pause zwischen zwei Messungen
//
// Rückgabewert:
// - true  = Kalibrierung erfolgreich
// - false = Sensorlesen fehlgeschlagen
bool mpuCalibrateGyro(uint16_t samples = 1000, uint16_t delayMs = 2);

// Liefert den zuletzt bestimmten Gyro-Offset-Rohwert.
float mpuGetGyroOffsetXRaw();

// -----------------------------------------------------
// Alt-Kompatibilität / Referenzwinkel
// -----------------------------------------------------

// Setzt einen Referenzwinkel explizit von außen.
//
// Dieser Referenzwinkel dient im Projekt dazu,
// den späteren relativen Winkel zur aktuellen Startlage
// oder zu einer gewünschten Nullposition zu bestimmen.
void mpuSetReferenceAngleDeg(float referenceAngleDeg);

// Setzt den aktuell gefilterten Winkel als Referenz.
//
// Typischer Einsatz:
// - beim Start
// - nach einer bewussten Nullung
// - wenn die aktuelle Lage als neuer Bezugspunkt gelten soll
void mpuSetCurrentAngleAsReference();

// Liefert den aktuell gespeicherten Referenzwinkel.
float mpuGetReferenceAngleDeg();

// Liefert den relativen Winkel:
// aktueller gefilterter Winkel minus Referenzwinkel.
//
// Dieser Wert ist im Projekt für die eigentliche
// Elevationsregelung besonders wichtig.
float mpuGetRelativeAngleDeg();