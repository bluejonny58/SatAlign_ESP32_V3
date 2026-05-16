/*
  SatAlign V3 - RF-Detector API
  ------------------------------------------------------------
  Stellt RF-Rohwert, gefilterte Bewertung und Hilfswerte fuer Anzeige und
  Suche bereit.
*/

#pragma once

#include <stdint.h>

// Initialisiert den RF-Detektor.
//
// Aufgaben dieser Funktion:
// - ADC-Pin als Eingang vorbereiten
// - ADC-Auflösung und Attenuation setzen
// - interne Laufvariablen zurücksetzen
//
// Diese Funktion sollte einmal in setup() aufgerufen werden,
// bevor rfUpdate() zyklisch verwendet wird.
void initRFDetector();

// Führt eine neue RF-Messung durch.
//
// Typischer Ablauf intern:
// - mehrere ADC-Messungen mitteln
// - gefilterten Wert berechnen
// - Min/Max-Werte aktualisieren
//
// Diese Funktion ist für den zyklischen Aufruf in loop() gedacht.
void rfUpdate();

// Liefert true, wenn der RF-Detektor erfolgreich initialisiert wurde.
//
// Praktisch nützlich für:
// - Display-Info
// - Diagnose
// - Absicherung, dass noch keine ungültigen RF-Werte verwendet werden
bool rfIsReady();

// Gibt den zuletzt gemessenen ungefilterten ADC-Rohwert zurück.
//
// Bereich typischerweise:
// 0 ... 4095 bei 12 Bit ADC-Auflösung
//
// Nützlich für:
// - Diagnose
// - Serielle Ausgabe
// - Vergleich mit dem gefilterten Signal
uint16_t rfGetRawAdc();

// Gibt den zuletzt berechneten gefilterten ADC-Wert zurück.
//
// Dieser Wert liegt noch im ADC-Rohwertbereich,
// also ebenfalls ungefähr 0 ... 4095, aber geglättet.
//
// Nützlich für:
// - stabilere Auswertung
// - Debug
float rfGetFilteredAdc();

// Wandelt den gefilterten ADC-Wert in eine Spannung um
// und gibt diese zurück.
//
// Diese Spannung ist im Projekt aktuell die wichtigste Größe
// für die RF-Auswertung.
//
// WICHTIG:
// Im jetzigen Projekt gilt näherungsweise:
// kleinere Spannung = stärkeres Signal
float rfGetPinVoltage();

// Kleinster bisher beobachteter Roh-ADC-Wert seit dem letzten Reset.
// Kann als Verlaufshilfe / Diagnosewert nützlich sein.
uint16_t rfGetMinRawAdc();

// Größter bisher beobachteter Roh-ADC-Wert seit dem letzten Reset.
// Ebenfalls hilfreich für Verlauf und Diagnose.
uint16_t rfGetMaxRawAdc();

// Setzt die bisher gemerkten Min/Max-Werte zurück.
//
// Sinnvoll z. B.:
// - vor einem neuen Testlauf
// - vor einer gezielten Suchfahrt
// - bei Diagnosezwecken
void rfResetMinMax();

// Misst ein aktuelles Kein-/Schwachsignal-Rauschband als Vergleichsbasis.
//
// Diese Funktion speichert nichts im NVS, sondern setzt nur RAM-Werte
// fuer die laufende Session. Sie wird beim Boot nach initRFDetector()
// aufgerufen.
void rfCalibrateNoSignalBaseline(int sampleBlocks, int delayMs);

// Misst ein RF-Fenster, ohne die aktuelle RF-Zero-/Baseline zu veraendern.
// meanAdc/minAdc/maxAdc duerfen als Pointer uebergeben werden; bei nullptr
// wird der jeweilige Wert nicht zurueckgegeben.
bool rfMeasureAdcWindow(int sampleBlocks, int delayMs, float* meanAdc, float* minAdc, float* maxAdc);

// Setzt die RF-Zero-/No-Signal-Baseline aus einem vorher gemessenen Fenster.
// Schreibt bewusst nichts in NVS/Flash.
void rfSetNoSignalBaselineAdc(float meanAdc, float minAdc, float maxAdc);

// true, sobald fuer die aktuelle Session ein RF-Zero gesetzt wurde.
bool rfHasNoSignalBaseline();

// Normierter Signalwert 0.0 ... 1.0.
// Im aktuellen AD8317/AD8318-Aufbau gilt: kleinerer ADC-Wert = staerkeres Signal.
float rfGetSignalNorm();

// Signalwert in Prozent 0 ... 100 fuer serielle Diagnose und Anzeige.
float rfGetSignalPercent();

// Differenz zwischen Baseline und aktuellem gefilterten ADC-Wert.
// Positiver Wert bedeutet: Signal wurde gegenueber der Baseline staerker.
float rfGetDropAdc();

// true, wenn der DROP_ADC gross genug ist, um von einem verwertbaren
// RF-Signal bzw. aktivem Signalweg auszugehen.
bool rfHasValidSignalDrop();
