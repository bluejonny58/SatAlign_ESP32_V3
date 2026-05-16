/*
  SatAlign V3 - Schnittstelle zu settings.cpp
  ------------------------------------------------------------
  Deklariert die zentralen Einstellwerte, damit Runtime, Web-UI, Display,
  RF-Auswertung und Motorsteuerung dieselben Parameter verwenden.
*/

#pragma once

// V3: Zeitfenster fuer den manuellen Winkel-Startbildschirm beim Einschalten.
// Definition und Defaultwert liegen in settings.cpp.
extern unsigned long BOOT_MANUAL_ELEVATION_WINDOW_MS;


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
// weiterhin die Signaloptimierung starten.
extern float RF_TV_USABLE_MAX_ADC;
extern float RF_TV_GOOD_MAX_ADC;
extern float RF_TV_STRONG_MAX_ADC;


// V3: Signaloptimierung nach PLUS-Bestaetigung eines Satelliten.
// Definition und Defaultwerte liegen in settings.cpp.
extern unsigned long SIGNAL_OPT_AZ_STEP_MS;
extern unsigned long SIGNAL_OPT_AZ_SETTLE_MS;
extern unsigned long SIGNAL_OPT_EL_STEP_MS;
extern unsigned long SIGNAL_OPT_EL_SETTLE_MS;
extern float SIGNAL_OPT_RF_IMPROVE_ADC;
extern float SIGNAL_OPT_RF_WORSE_ADC;
extern float SIGNAL_OPT_RETURN_EL_TOLERANCE_DEG;
extern int SIGNAL_OPT_AZ_MAX_STEPS_PER_DIR;
extern int SIGNAL_OPT_EL_MAX_STEPS_PER_DIR;
