/*
  SatAlign V3 - Azimut API
  ------------------------------------------------------------
  Oeffentliche Funktionen fuer Azimutbewegung, Stop und Hall-Sensor-Status.
*/

#pragma once

#include "types.h"

// =====================================================
// Azimut-Steuerung
// =====================================================
//
// Dieses Modul kapselt die komplette Azimut-Achse:
//
// - EAST / WEST Ausgangssignale
// - Einzelpulse
// - gestufte Schrittbewegung
// - Referenzfahrt zur Mitte über Hall-Sensoren
// - Endanschlag-Logik Ost / West
// - Statusabfragen für Web, Display und AUTO-Strategie
//
// WICHTIG:
// Hier geht es nur um die Azimut-Achse.
// Die übergeordnete AUTO-Zustandsmaschine liegt in live_runtime.cpp.

// Initialisiert die Azimut-Hardware.
//
// Aufgaben:
// - Ausgänge für EAST / WEST vorbereiten
// - Hall-Sensor-Pins vorbereiten
// - internen Bewegungszustand zurücksetzen
void initAzimuth();

// Stoppt jede laufende Azimutbewegung sofort.
//
// Wirkung:
// - Ausgänge stromlos / LOW
// - Pulszustand zurückgesetzt
// - Homing-Suche beendet
void azimuthStop();


// -----------------------------------------------------
// Direkte logische Roh-Ansteuerung fuer Spezialroutinen
// -----------------------------------------------------
//
// Nutzt dieselbe zentrale EAST/WEST-Korrektur wie die normale
// Azimutsteuerung, startet aber keinen Puls- oder Step-Modus.
// Gedacht fuer Routinen mit eigener Zeitmessung, z. B. Mitte ausrichten.
// Rueckgabe: true = Ausgang wurde in diese logische Richtung eingeschaltet.
bool azimuthDriveRawLogical(AzimuthDirection direction);

// Direkte manuelle AZ-Ansteuerung ohne Endsensor-/Software-Sperre.
// Nur fuer den manuellen Tastermodus verwenden. AUTO/Mitte/Suche bleiben
// auf den abgesicherten Funktionen.
bool azimuthDriveManualOverride(AzimuthDirection direction);

// -----------------------------------------------------
// Direkte Roh-Ausgänge
// -----------------------------------------------------
//
// Diese Funktionen schalten die Azimut-Ausgänge direkt in eine Richtung.
// Sie sind vor allem für einfache manuelle / rohe Steuerfälle gedacht.
//
// WICHTIG:
// Sie prüfen trotzdem die jeweilige Bewegungsfreigabe
// über die Endanschlag-Sensorik.

// Schaltet Azimut direkt nach EAST.
void azimuthEastOn();

// Schaltet Azimut direkt nach WEST.
void azimuthWestOn();

// -----------------------------------------------------
// Einzelpulse
// -----------------------------------------------------
//
// Diese Funktionen erzeugen einen einzelnen zeitlich begrenzten Puls
// in die gewünschte Richtung.
//
// Typischer Einsatz:
// - manuelle Korrekturen
// - kleine Positionsänderungen
// - Web-/Taster-Kommandos

// Einzelpuls nach EAST mit frei vorgegebener Pulsdauer in Millisekunden.
void azimuthPulseEast(unsigned long pulseMs);

// Einzelpuls nach WEST mit frei vorgegebener Pulsdauer in Millisekunden.
void azimuthPulseWest(unsigned long pulseMs);

// -----------------------------------------------------
// Schrittbetrieb bis STOP
// -----------------------------------------------------
//
// Diese Funktionen starten den gestuften Dauerbetrieb:
// - erster Puls länger
// - danach wiederholte Schritte
// - läuft weiter, bis gestoppt oder blockiert
//
// Typischer Einsatz:
// - Suchbewegungen
// - längere Verfahrwege
// - Homing-nahe Routinen

// Startet gestuften Betrieb nach EAST.
void azimuthStartEastStepped();

// Startet gestuften Betrieb nach WEST.
void azimuthStartWestStepped();

// -----------------------------------------------------
// Referenzfahrt zur Mitte
// -----------------------------------------------------
//
// Die Referenzfahrt sucht mit Hilfe der Hall-Sensoren
// die definierte Mittelposition / Südrichtung.
//
// Typischer Ablauf:
// - zuerst bevorzugte Suchrichtung laut Settings
// - falls blockiert oder Limit erreicht, ggf. Gegenrichtung
// - Erfolg: Mitte erkannt
// - Fehler: keine Richtung mehr sinnvoll oder Mitte nicht gefunden

// Startet die automatische Suche nach der Mitte.
void azimuthStartHomingToCenter();

// Bricht die laufende Referenzfahrt ab.
void azimuthAbortHoming();

// Liefert true, solange die Referenzfahrt aktiv läuft.
bool azimuthIsHomingActive();

// Liefert true, wenn die Referenzfahrt erfolgreich abgeschlossen wurde.
bool azimuthIsHomed();

// Liefert true, wenn die Referenzfahrt in einen Fehler gelaufen ist.
bool azimuthDidHomingFail();

// -----------------------------------------------------
// Zyklische Aktualisierung
// -----------------------------------------------------
//
// Muss regelmäßig in loop() bzw. in der zentralen Runtime
// aufgerufen werden.
//
// Hier passieren:
// - Pulsende
// - Schrittfortschaltung
// - Endanschlag-Stop
// - Homing-Fortschritt
void azimuthUpdate();

// -----------------------------------------------------
// Status
// -----------------------------------------------------

// Liefert true, wenn gerade irgendeine Azimut-Puls-/Bewegungslogik aktiv ist.
bool azimuthIsPulseActive();

// Liefert true, wenn die Ausgangssignale aktuell eingeschaltet sind.
// Also nicht nur "Bewegung geplant", sondern Ausgang wirklich aktiv.
bool azimuthIsOutputOn();

// Liefert true, wenn aktuell gestufter Azimutlauf aktiv ist.
bool azimuthIsSteppedRunActive();

// Liefert die aktuell aktive Bewegungsrichtung.
// Typisch:
// - AZ_DIR_EAST
// - AZ_DIR_WEST
// - AZ_DIR_NONE
AzimuthDirection azimuthGetDirection();

// Gibt die Zahl der bereits abgeschlossenen Schritte/Pulse zurück.
// Nützlich für Debug, Telemetrie und Suchanalyse.
unsigned long azimuthGetStepCounter();

// Gibt die momentan verwendete Einschaltdauer des aktuellen Pulses zurück.
// Nützlich zur Diagnose von Einzelpuls vs. erstem Puls vs. Schrittpuls.
unsigned long azimuthGetCurrentOnTimeMs();

// -----------------------------------------------------
// Hall-Sensoren
// -----------------------------------------------------
//
// Diese Funktionen abstrahieren die drei Hall-Sensoren
// in semantische Zustände.

// Mitte / Südrichtung erkannt?
bool azimuthIsCenterDetected();

// Ost-Endanschlag erkannt?
// Liefert die LOGISCHE Ost-Seite nach Mittenreferenz.
// Die physische Pin-Zuordnung wird zentral in azimuth_control.cpp korrigiert.
bool azimuthIsEastLimitDetected();

// West-Endanschlag erkannt?
// Liefert die LOGISCHE West-Seite nach Mittenreferenz.
// Die physische Pin-Zuordnung wird zentral in azimuth_control.cpp korrigiert.
bool azimuthIsWestLimitDetected();

// Rohdiagnose der echten Hall-Pins ohne logische EAST/WEST-Korrektur.
// Nur fuer Serial-/Fehlersuche verwenden, nicht fuer Bewegungsfreigaben.
bool azimuthIsRawCenterPinDetected();
bool azimuthIsRawEastLimitPinDetected();
bool azimuthIsRawWestLimitPinDetected();

// -----------------------------------------------------
// Erlaubnislogik
// -----------------------------------------------------
//
// Diese Funktionen prüfen, ob die Achse aufgrund der
// Endanschlag-Sensorik in die jeweilige Richtung fahren darf.

// Darf aktuell nach EAST gefahren werden?
bool azimuthCanMoveEast();

// Darf aktuell nach WEST gefahren werden?
bool azimuthCanMoveWest();