#pragma once

#include <Arduino.h>
#include "types.h"

// Globale Modusvariable des Systems.
// Hier steht jederzeit der aktuell aktive Hauptmodus:
//
// - CONTROL_MANUAL
// - CONTROL_AUTO
//
// WICHTIG:
// Diese Variable beschreibt nur den globalen Hauptmodus.
// Feingranulare Unterzustände der AUTO-Strategie liegen separat
// in live_runtime.cpp.
extern ControlMode controlMode;

// Zeitstempel des letzten manuellen Eingriffs.
// Wird beim Wechsel in den manuellen Modus aktualisiert.
//
// Praktisch nützlich für:
// - spätere Timeout-Ideen
// - Protokollierung / Diagnose
// - manuelle Eingriffserkennung
extern unsigned long lastManualCommandTime;

// -----------------------------------------------------
// Alt-Kompatibilität
// -----------------------------------------------------
// Historische Hilfsfunktion aus älteren Entwicklungsständen.
// Setzt das System auf MANUELL zurück.
//
// Der Name stammt aus der früheren Idee eines "Manual Override".
// Intern wird heute einfach enterManualMode() verwendet.
void setManualOverride();

// Prüft, ob AUTO aktuell erlaubt ist.
//
// In der jetzigen Logik ist das bewusst einfach gehalten:
// AUTO ist genau dann erlaubt, wenn controlMode auf AUTO steht.
//
// Die Funktion bleibt erhalten, damit ältere oder alternative
// Codepfade weiterhin kompatibel auf diese Abfrage zugreifen können.
bool isAutoAllowed();

// -----------------------------------------------------
// Klare Modusumschaltung
// -----------------------------------------------------

// Schaltet das System explizit in das Hauptmenue.
// In diesem Zustand laufen keine manuellen Motorbefehle.
void enterMainMenuMode();

// Schaltet das System in den speziellen Modus "Grundeinstellung / Mitte referenzieren".
// In diesem Modus laeuft nur die Referenzfahrt zur Mitte.
void enterCenterMode();

// Schaltet das System explizit in den manuellen Modus.
//
// Typische Verwendung:
// - Web-Button MANUELL
// - langer MODE-Druck aus AUTO heraus
// - manueller Eingriff während des Betriebs
void enterManualMode();

// Schaltet das System explizit in den AUTO-Modus.
//
// WICHTIG:
// Diese Funktion setzt nur den globalen Hauptmodus.
// Die eigentliche AUTO-Zustandsmaschine und Suchlogik liegen
// in live_runtime.cpp.
void enterAutoMode();

// Hilfsfunktion: true, wenn das System aktuell im Hauptmenue steht.
bool isMainMenuMode();

// Hilfsfunktion: true, wenn das System aktuell die Mitte einstellt.
bool isCenterMode();

// Hilfsfunktion: true, wenn das System aktuell im manuellen Modus ist.
bool isManualMode();

// Hilfsfunktion: true, wenn das System aktuell im AUTO-Modus ist.
bool isAutoMode();