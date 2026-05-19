/*
  SatAlign V3 - Schnittstelle zu settings.cpp
  ------------------------------------------------------------
  Deklariert die zentralen Einstellwerte, damit Runtime, Web-UI, Display,
  RF-Auswertung und Motorsteuerung dieselben Parameter verwenden.
*/

#pragma once


// Initialisiert das zentrale Settings-System.
//
// No-NVS-Version:
// - setzt alle Runtime-Werte auf die Code-Defaults aus settings.cpp
// - lädt keine Werte aus Preferences/NVS
// - schreibt keine Werte in den ESP32-Flash
//
// Rückgabewert:
// - false = es wurden Code-Defaults verwendet
bool initSettings();

// Kompatibilitätsfunktion für frühere Projektstände.
//
// No-NVS-Version:
// Diese Funktion schreibt bewusst nichts in den Flash.
// Falls später wieder persistente Werte gewünscht sind, wird das
// gezielt neu entschieden und nicht automatisch reaktiviert.
void saveAllSettings();

// Setzt alle Settings wieder auf die in settings.cpp definierten
// Code-Defaultwerte zurück.
//
// No-NVS-Version:
// Der Reset wirkt nur auf die Runtime-Werte im RAM.
// Es wird nichts in den Flash geschrieben.
void resetSettingsToDefaults();

// Gibt alle aktuell aktiven Settings über die serielle Schnittstelle aus.
//
// Diese Funktion ist rein diagnostisch:
// - hilfreich nach dem Boot
// - hilfreich zum Prüfen der aktiven Code-Defaultwerte
// - hilfreich zum Vergleichen von Default- und Live-Werten
void printSettingsToSerial();

// V3: RF-Bewertung aus Aussentest.
// Diese Werte dienen als Ampel fuer schwach/brauchbar/gut/sehr gut.
// Sie blockieren PLUS nicht; der Nutzer darf bei sichtbarem TV-Bild
// weiterhin den Kandidaten bestaetigen.
extern float RF_TV_USABLE_MAX_ADC;
extern float RF_TV_GOOD_MAX_ADC;
extern float RF_TV_STRONG_MAX_ADC;

// V3_01: Mindest-Prozentwert fuer die Kandidatenerkennung waehrend
// der ersten AUTO-Centerfahrt. Diese Schwelle ist bewusst strenger
// als die normale Ost-/West-Suche: Nur wirklich gute Signale sollen
// die Centerfahrt stoppen. 0 % = schwach/kein Signal, 100 % = sehr gut.
extern float AUTO_CENTER_RF_MIN_GOOD_SIGNAL_PERCENT;

// V3_01: Zentrale RF-Referenz- und AUTO-Schwellwerte.
// Definition und Defaultwerte liegen in settings.cpp.
// Diese Werte koennen bei Aussentests manuell angepasst werden, ohne
// live_runtime.cpp oder rf_detector.cpp durchsuchen zu muessen.
extern float RF_WEAK_REFERENCE_ADC;
extern float RF_STRONG_REFERENCE_ADC;
extern float RF_VALID_DROP_ADC;

extern float AUTO_RF_CANDIDATE_DROP_ADC;
extern float AUTO_RF_CANDIDATE_EXIT_ADC;

extern float AUTO_RF_DYNAMIC_MIN_ENTER_V;
extern float AUTO_RF_DYNAMIC_MIN_EXIT_V;
extern float AUTO_RF_DYNAMIC_ENTER_FACTOR;
extern float AUTO_RF_DYNAMIC_EXIT_FACTOR;

extern int AUTO_CANDIDATE_CONFIRM_COUNT;
extern unsigned long AUTO_CANDIDATE_CONFIRM_INTERVAL_MS;

extern float BADSAT_MIN_TRIGGER_V;
extern float BADSAT_WEAK_MAX_V;
extern int   BADSAT_WEAK_RADIUS_STEPS;
extern float BADSAT_STRONG_MAX_V;
extern int   BADSAT_STRONG_RADIUS_STEPS;
extern int   BADSAT_RESUME_MARGIN_STEPS;
extern int   BADSAT_MERGE_GAP_STEPS;


// V3_01 Cleanup: Die alten Parameter der entfernten Signaloptimierung
// wurden geloescht, weil diese Funktion nicht mehr Teil der Bedienlogik ist.
