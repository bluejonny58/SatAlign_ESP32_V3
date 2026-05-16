#pragma once

// =====================================================
// TestMode
// =====================================================
//
// Diese Enumeration stammt aus der früheren Teststruktur des Projekts.
//
// Sie wurde verwendet, um im Hauptsketch gezielt genau einen
// Testpfad zu aktivieren, z. B. nur Azimut, nur Elevation,
// nur MPU oder den kombinierten Web-/Manual-Test.
//
// Im aktuellen Live-System ist diese Struktur nicht mehr der
// zentrale Laufzeitpfad, kann aber als Rest/Archiv für ältere
// Teststände weiter im Projekt liegen.
enum TestMode {
  TEST_AZIMUTH,     // isolierter Azimut-Test
  TEST_ELEVATION,   // isolierter Elevations-Test
  TEST_MPU6050,     // isolierter MPU-/Winkel-Test
  TEST_WEB_MANUAL   // kombinierter Web-/Manual-Test
};

// =====================================================
// AzimuthDirection
// =====================================================
//
// Richtungsdefinition für die Azimut-Achse.
//
// Diese Werte werden projektweit verwendet für:
// - Azimut-Steuerung
// - Homing
// - Scan-Logik
// - Statusanzeigen
enum AzimuthDirection {
  AZ_DIR_NONE,  // keine Richtung / Stopp / undefiniert
  AZ_DIR_EAST,  // Bewegung nach Osten
  AZ_DIR_WEST   // Bewegung nach Westen
};

// =====================================================
// ElevationDirection
// =====================================================
//
// Semantische Richtungsdefinition für die Elevations-Achse.
//
// WICHTIG:
// Diese Werte beschreiben die fachliche / angezeigte Richtung,
// nicht zwingend direkt die elektrische Roh-Richtung am Treiber.
// Die tatsächliche Roh-Ansteuerung wird in elevation_control.cpp
// passend auf die reale Mechanik abgebildet.
enum ElevationDirection {
  EL_DIR_STOP,  // keine Bewegung
  EL_DIR_UP,    // angezeigter Winkel wird größer
  EL_DIR_DOWN   // angezeigter Winkel wird kleiner
};

// =====================================================
// ControlMode
// =====================================================
//
// Globaler Hauptmodus des Systems.
//
// Diese Werte beschreiben die grobe Betriebsart:
//
// - CONTROL_MAIN_MENU -> Start-/Hauptmenue nach dem Boot, keine Bewegung
// - CONTROL_CENTER    -> automatische Referenzfahrt zur Mitte
// - CONTROL_MANUAL    -> Benutzer steuert selbst
// - CONTROL_AUTO      -> AUTO-Strategie / Zustandsmaschine aktiv
enum ControlMode {
  CONTROL_MAIN_MENU,  // Start-/Hauptmenue, Motoren aus
  CONTROL_CENTER,     // Menuepunkt 1: automatische Referenzfahrt zur Mitte
  CONTROL_MANUAL,     // Menuepunkt 3: manueller Hauptmodus
  CONTROL_AUTO        // Menuepunkt 2: automatischer Hauptmodus
};