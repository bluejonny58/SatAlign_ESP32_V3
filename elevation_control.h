/*
  SatAlign V3 - Winkel-/Elevations API
  ------------------------------------------------------------
  Oeffentliche Funktionen fuer Winkelbewegung, Stop und Pulsfahrt.
*/

#pragma once

#include "types.h"

// =====================================================
// Elevations-Steuerung
// =====================================================
//
// Dieses Modul kapselt die komplette Elevations-Achse:
//
// - Roh-Ansteuerung des L298N
// - semantische Richtungen "hoch" / "runter"
// - Einzelpulse für feine Bewegungen
// - zyklische Pulsverwaltung
// - Softlimit-Prüfung
// - einfache Zielanfahrten in Pulslogik
//
// WICHTIG:
// Die eigentliche übergeordnete AUTO-Strategie liegt nicht hier,
// sondern in live_runtime.cpp.

// Initialisiert die Elevations-Hardware.
//
// Aufgaben:
// - IN1 / IN2 / ENA als Ausgänge setzen
// - sicheren Stop-Zustand herstellen
void initElevation();

// -----------------------------------------------------
// Rohfunktionen entsprechend dem funktionierenden Test-Sketch
// -----------------------------------------------------
//
// Diese drei Funktionen sprechen den L298N direkt an.
//
// WICHTIG:
// "Raw" bedeutet hier wirklich nur elektrische Richtung,
// nicht die semantische Bedeutung für den angezeigten Winkel.
// Wegen der realen Mechanik ist "elektrisch hoch" im Projekt
// nicht zwingend dasselbe wie "angezeigter Winkel wird größer".

// Rohbewegung in eine Richtung mit vorgegebenem PWM-Wert.
void elevationUpRaw(int pwm);

// Rohbewegung in die Gegenrichtung mit vorgegebenem PWM-Wert.
void elevationDownRaw(int pwm);

// Schaltet die Elevationsausgänge stromlos / stoppt den Motor.
void elevationStopRaw();

// -----------------------------------------------------
// Semantische Funktionen
// -----------------------------------------------------
//
// Diese Funktionen bilden die im Projekt gewünschte fachliche Bedeutung ab:
//
// elevationUp()   -> angezeigter Winkel wird größer
// elevationDown() -> angezeigter Winkel wird kleiner
//
// Intern kann die Roh-Richtung dafür invertiert sein,
// je nach realem mechanischem Aufbau.

// Bewegt die Elevation semantisch "nach oben".
void elevationUp();

// Bewegt die Elevation semantisch "nach unten".
void elevationDown();

// Stoppt die Elevation semantisch korrekt.
void elevationStop();

// -----------------------------------------------------
// Nicht-blockierende Pulse
// -----------------------------------------------------
//
// Diese Funktionen starten eine zeitlich begrenzte Bewegung.
// Die Bewegung endet nicht automatisch im selben Funktionsaufruf,
// sondern wird später in elevationUpdate() abgeschlossen.
//
// Das ist wichtig für:
// - feine manuelle Korrekturen
// - schrittweise Zielanfahrten
// - AUTO-Strategie

// Startet einen semantischen Puls nach oben.
void elevationPulseUp(unsigned long pulseMs, int pwm);

// Startet einen semantischen Puls nach unten.
void elevationPulseDown(unsigned long pulseMs, int pwm);

// Muss zyklisch aufgerufen werden, damit aktive Pulse sauber enden.
void elevationUpdate();

// -----------------------------------------------------
// Status
// -----------------------------------------------------

// Liefert true, wenn aktuell ein Elevationspuls aktiv läuft.
bool elevationIsPulseActive();

// Prüft, ob ein Winkel innerhalb der softwareseitigen Softlimits liegt.
bool elevationWithinSoftLimits(float currentAngle);

// Liefert die aktuell semantisch aktive Bewegungsrichtung.
ElevationDirection elevationGetDirection();

// -----------------------------------------------------
// Zielregelung mit absolutem/angezeigtem Winkel
// -----------------------------------------------------
//
// Diese Funktionen erzeugen eine einfache pulsbasierte Zielregelung.
// Sie arbeiten schrittweise:
// - Winkel prüfen
// - ggf. passenden Puls auslösen
// - beim nächsten Aufruf weiter bewerten
//
// Rückgabewert:
// - true  = Ziel innerhalb der Toleranz erreicht
// - false = Ziel noch nicht erreicht oder Bewegung läuft noch

// Zielregelung mit Winkelbezug "absolut/anzeigeorientiert".
bool moveToElevationPulse(float currentAngleDeg,
                          float targetAngleDeg,
                          float toleranceDeg);

// V3: Sonderfunktion fuer die erste Standard-Elevation beim Einschalten.
//
// Diese Funktion ignoriert bewusst nur die softwareseitigen Elevations-
// Softlimits. Sie ist fuer den Boot-Start gedacht, wenn die Anlage zuerst grob
// auf den zentralen Standardwinkel fahren soll. Die mechanischen Endabschalter
// des Linearantriebs bleiben davon unberuehrt. Normale AUTO-/Suchbewegungen
// sollen weiterhin die Variante mit Softlimit-Pruefung verwenden.
bool moveToElevationPulseNoSoftLimits(float currentAngleDeg,
                                      float targetAngleDeg,
                                      float toleranceDeg);

// Kompatibilitäts-Name.
// Im aktuellen Projekt funktional identisch zur obigen Variante,
// aber semantisch passend für relative Elevationslogik.
bool moveToRelativeElevationPulse(float currentAngleDeg,
                                  float targetAngleDeg,
                                  float toleranceDeg);