/*
  SatAlign ESP32 - control_state.cpp
  ---------------------------------------------------------------------------
  Verwaltet den groben Betriebsmodus des Systems: Hauptmenue, Ausrichten,
  AUTO und Manuell. Die Detailzustandsmaschine liegt in live_runtime.cpp.
*/
#include <Arduino.h>

#include "control_state.h"
#include "config.h"

// Globale Hauptmodus-Variable.
// Startet mit dem in den Settings / der Konfiguration
// definierten Default-Modus.
ControlMode controlMode = DEFAULT_CONTROL_MODE;

// Zeitstempel des letzten manuellen Eingriffs.
// Wird beim Wechsel in den manuellen Modus gesetzt.
// So lässt sich später nachvollziehen, wann zuletzt
// bewusst manuell eingegriffen wurde.
unsigned long lastManualCommandTime = 0;

// Schaltet das System explizit in das Hauptmenue.
// In diesem Zustand sind Motoren nicht aktiv; Bewegungen starten erst
// nach bewusster Auswahl von MANUELL oder AUTO.
void enterMainMenuMode() {
  controlMode = CONTROL_MAIN_MENU;
}

// Schaltet das System in den Modus fuer Menuepunkt 1: Ausrichten / Mitte referenzieren.
void enterCenterMode() {
  controlMode = CONTROL_CENTER;
}

// Schaltet das System explizit in den manuellen Modus.
//
// Zusätzlich wird der Zeitstempel aktualisiert,
// damit spätere Logik oder Diagnosefunktionen wissen,
// wann zuletzt ein manueller Eingriff stattgefunden hat.
void enterManualMode() {
  controlMode = CONTROL_MANUAL;
  lastManualCommandTime = millis();
}

// Schaltet das System explizit in den AUTO-Modus.
//
// WICHTIG:
// Diese Funktion aktiviert nur den globalen Hauptmodus.
// Die eigentliche AUTO-Ablaufsteuerung liegt woanders,
// aktuell in live_runtime.cpp.
void enterAutoMode() {
  controlMode = CONTROL_AUTO;
}

// Hilfsfunktion für bessere Lesbarkeit im restlichen Code.
// Liefert true, wenn aktuell das Hauptmenue aktiv ist.
bool isMainMenuMode() {
  return controlMode == CONTROL_MAIN_MENU;
}

// Hilfsfunktion für bessere Lesbarkeit im restlichen Code.
// Liefert true, wenn aktuell die Mitte eingestellt wird.
bool isCenterMode() {
  return controlMode == CONTROL_CENTER;
}

// Hilfsfunktion für bessere Lesbarkeit im restlichen Code.
// Liefert true, wenn aktuell der MANUELL-Modus aktiv ist.
bool isManualMode() {
  return controlMode == CONTROL_MANUAL;
}

// Hilfsfunktion für bessere Lesbarkeit im restlichen Code.
// Liefert true, wenn aktuell der AUTO-Modus aktiv ist.
bool isAutoMode() {
  return controlMode == CONTROL_AUTO;
}

// -----------------------------------------------------
// Alt-Kompatibilität
// -----------------------------------------------------
//
// Historische Wrapper-Funktion aus älteren Entwicklungsständen.
// Bedeutet heute einfach:
// "wechsle in MANUELL"
void setManualOverride() {
  enterManualMode();
}

// Prüft, ob AUTO aktuell erlaubt ist.
//
// In der jetzigen Projektlogik gibt es keinen automatischen
// Rücksprung oder impliziten AUTO-Timeout mehr.
// Deshalb ist AUTO genau dann erlaubt, wenn controlMode
// explizit auf CONTROL_AUTO steht.
bool isAutoAllowed() {
  return isAutoMode();
}