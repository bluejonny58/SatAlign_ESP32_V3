/*
  SatAlign V3 - einstellbare Betriebswerte
  ------------------------------------------------------------
  Hier liegen die zentralen Test- und Kalibrierwerte des Projekts:
  - Standard-Winkel beim Start
  - Startfenster fuer manuelle Winkeleinstellung
  - RF-Grenzwerte aus den Aussentests
  - reservierte Altwerte der entfernten Signaloptimierung

  Die Werte werden bewusst im Sketch gehalten und nicht dauerhaft in NVS
  gespeichert. Wenn im Test bessere Werte gefunden werden, werden sie hier
  nachvollziehbar geaendert und kommentiert.
*/

/*
  SatAlign ESP32 - settings.cpp
  ---------------------------------------------------------------------------
  Zentrale Code-Defaults des Projekts.

  Wichtig: Diese Version nutzt bewusst keine NVS-/Preferences-Speicherung.
  Aenderungen an Zielwinkeln, Limits oder RF-Parametern sollen bewusst hier
  bzw. im Sketch erfolgen, damit der Code die verbindliche Quelle bleibt.
*/
#include <Arduino.h>
#include "settings.h"
#include "config.h"

// =====================================================
// Globale Runtime-Werte
// =====================================================
//
// WICHTIG:
// Diese Datei ist die zentrale Stelle für alle globalen,
// veränderbaren Systemwerte, die nicht mehr als harte
// Konstanten im Code stehen sollen.
//
// Die Variablen werden hier:
//
// 1. definiert
// 2. mit Defaultwerten versehen
// 3. beim Start immer aus den Code-Defaults aufgebaut
// 4. nicht automatisch in Preferences/NVS gespeichert
//
// config.h enthält dazu nur die extern-Deklarationen.

// -----------------------------------------------------
// Allgemein
// -----------------------------------------------------

// Serielle Baudrate für Diagnose / Monitor
unsigned long SERIAL_BAUDRATE = 115200;

// Abstand zwischen den Heartbeat-Ausgaben im Hauptloop
unsigned long HEARTBEAT_MS = 2000;

// -----------------------------------------------------
// Betriebsmodi
// -----------------------------------------------------

// Startmodus des Systems nach dem Einschalten.
// Der Live-Code setzt später oft explizit den gewünschten Modus,
// aber der globale Startwert bleibt trotzdem hier zentral definiert.
ControlMode DEFAULT_CONTROL_MODE = CONTROL_MAIN_MENU;

// Historischer/ergänzender Timeout-Wert für manuelle Eingriffe.
// Aktuell ist er nicht das zentrale Steuerinstrument,
// bleibt aber als Systemparameter verfügbar.
unsigned long MANUAL_OVERRIDE_TIMEOUT_MS = 15000;

// -----------------------------------------------------
// Azimut
// -----------------------------------------------------

// Standardpuls für einfache manuelle Azimutkommandos
unsigned long AZ_PULSE_MS = 250;

// Zusätzlicher Intervallwert aus früheren Tests / Hilfsroutinen
unsigned long AZ_TEST_INTERVAL_MS = 2500;

// Allgemeine Azimut-Einschwingzeit
unsigned long AZ_SETTLE_MS = 300;

// Erster längerer Puls im gestuften Azimutlauf.
//
// FUER DEN AKTUELLEN ERSTEN PRAXISTEST bewusst deutlich erhoeht,
// damit der langsame Azimutmotor beim Anfahren moeglichst lange
// am Stueck durchlaufen kann, statt sofort in kurze Suchschritte
// zerlegt zu werden.
unsigned long AZ_FIRST_PULSE_ON_MS = 3000;

// Normale Schritt-Pulsdauer im gestuften Azimutbetrieb.
//
// Ebenfalls fuer den ersten Test stark vergroessert, damit die
// Azimutbewegung sich fast wie ein Durchlauf anfuellt.
// Die eigentliche Feinstufigkeit soll spaeter erst nahe am Ziel
// bzw. bei echter RF-Reaktion kommen.
unsigned long AZ_STEP_PULSE_ON_MS = 1200;

// Pause zwischen zwei Azimut-Schritten.
//
// Fuer den ersten Test stark reduziert, damit der Motor zwischen
// zwei Schritten kaum stehen bleibt und praktisch fast durchlaeuft.
unsigned long AZ_STEP_PULSE_OFF_MS = 20;

//
// Hall-Sensor-Logik:
//
// Die A3144-Ausgänge werden hier mit externen 10k Pull-ups auf 3,3V betrieben.
// Dadurch gilt:
// - ohne Magnetkontakt -> Eingang liegt auf HIGH
// - bei erkanntem Magneten -> Hall-Sensor zieht nach GND -> LOW
//
// Deshalb ist die Hall-Logik active-low.
//
bool AZ_HALL_ACTIVE_LOW = true;

//
// Die Hall-Sensoren hängen jetzt an GPIO34 / GPIO36 / GPIO39.
// Diese Eingänge bekommen ihre Pull-ups extern auf der Platine,
// deshalb sollen die internen Pull-ups des ESP32 hier NICHT verwendet werden.
//
bool AZ_HALL_USE_INTERNAL_PULLUP = false;

// Historischer Richtungsparameter fuer alte Homing-Funktionen.
// Die aktuelle Mittenfunktion verwendet ihre eigene getestete Start-/Richtungslogik.
AzimuthDirection AZ_HOME_FIRST_SEARCH_DIRECTION = AZ_DIR_EAST;

// -----------------------------------------------------
// Elevation
// -----------------------------------------------------

// Allgemeiner Elevations-Bewegungswert aus älteren Tests
unsigned long EL_MOVE_MS = 2000;

// Allgemeiner Elevations-Pausenwert
unsigned long EL_PAUSE_MS = 2000;

// EZ-Softlimits fuer den normalen Astra-19.2E-Betrieb in Deutschland.
// Astra liegt in Deutschland typischerweise grob im Bereich ca. 26 bis 34 Grad Elevation.
// Die Grenzen sind bewusst enger gesetzt, damit AUTO keine unplausiblen Hoehenbereiche absucht.
// Fuer andere Laender oder stark abweichende Standorte muessen diese Werte ggf. angepasst werden.
float ELEVATION_MIN_SOFT = 25.0f;
float ELEVATION_MAX_SOFT = 34.0f;

// Zentraler Standard-Sollwert fuer die Elevation.
// Kommentarstand: V3
//
// Dieser Wert ist bewusst die eine zentrale Stelle fuer den gewuenschten
// Standard-EZ-Winkel nach dem Einschalten.
//
// Verwendung in dieser Version:
// - Suchen/Web-UI: wird als empfohlener Standardwinkel angezeigt
// - Web-UI: wird im Such-Setup als Standard-EZ/Ziel angezeigt
// - Diagnose: wird im Serial-Settings-Dump ausgegeben
//
// Wenn sich beim Test zuhause ein besserer Startwinkel ergibt, wird dieser
// Wert hier im Sketch geaendert. Es wird weiterhin kein NVS/Preferences
// verwendet, damit der Code die verbindliche Quelle bleibt.
//
// Live-Test-Anpassung V3:
// Der Standard-Sollwert wird testweise auf 30.0 Grad gesetzt. Die Elevation
// darf fuer den Start bewusst grob sein, weil die spaetere Satellitensuche
// ueber das RF-Signal entscheidet.
float DEFAULT_TARGET_ELEVATION = 30.0f;

// Toleranz für normale Zielanfahrten.
float ELEVATION_TOLERANCE_DEG = 1.0f;

// V3: Der fruehere automatische/manuelle Boot-EZ-Start wurde entfernt.
// Die Elevation wird nun direkt in den normalen Menues korrigiert.

// PWM für schnelle Elevationsfahrt
int EL_PWM_FAST = 140;

// PWM für langsame / feinere Elevationsfahrt
int EL_PWM_SLOW = 90;

// Pulsdauer für schnelle Elevationsbewegung
unsigned long EL_PULSE_FAST_MS = 180;

// Pulsdauer für langsame / feinere Elevationsbewegung
unsigned long EL_PULSE_SLOW_MS = 80;

// Fehlerband, innerhalb dessen langsam / fein geregelt wird
float ELEVATION_SLOW_BAND_DEG = 3.0f;

// -----------------------------------------------------
// Anzeige-Offset Elevation
// -----------------------------------------------------

// Korrigiert die angezeigte Elevation ohne die gesamte Sensorik
// oder Referenzlogik umzubauen.
//
// Diagnosebasis vom MPU/TFT-Test:
// - FiltX lag bei ca. 66,9 Grad
// - die reale Elevation wurde auf ca. 30 Grad geschaetzt
// - Formel: EZ = 90 - FiltX + Offset
// Neubrandenburg/Astra-Test:
//   gemessener Astra-Punkt lag im Serial bei REL ca. 23,0 Grad.
//   rechnerischer Sollwert fuer Astra 19,2E bei Neubrandenburg ca. 28,6 Grad.
//   Daraus ergibt sich ein fester Anzeigeoffset von +11,6 Grad.
//
// Dieser Wert wird bewusst fest im Sketch gepflegt und nicht in NVS
// gespeichert. Wenn spaeter ein besserer Wert ermittelt wird, wird er hier
// im Code angepasst.
float DISPLAY_ANGLE_OFFSET_DEG = 11.6f;

// Definierter Start-/Referenzwinkel fuer fruehere Testversionen.
//
// Hinweis:
// Die automatische Session-Referenz ist deaktiviert.
// Die Elevation wird wieder direkt ueber die Anzeigeformel
// 90 - FiltX + DISPLAY_ANGLE_OFFSET_DEG berechnet.
// Der Wert bleibt nur als dokumentierter Platzhalter erhalten.
float SESSION_START_ELEVATION_DEG = 29.0f;

// -----------------------------------------------------
// Web / Taster Elevation
// -----------------------------------------------------

// Pulsdauer für manuelle Elevations-Kommandos über Web/Taster
unsigned long WEB_EL_PULSE_MS = 250;

// PWM für dieselben manuellen Web-/Tasterkommandos
int WEB_EL_PWM = 110;

// -----------------------------------------------------
// 3-Taster-Logik
// -----------------------------------------------------

// Entprellzeit der Taster
unsigned long BTN_DEBOUNCE_MS = 35;

// Grenzwert für langen MODE-Druck
unsigned long BTN_MODE_LONGPRESS_MS = 700;

// -----------------------------------------------------
// RF / AD8318
// -----------------------------------------------------

// Anzahl ADC-Messungen pro RF-Update
int RF_ADC_SAMPLES_PER_CYCLE = 32;

// Glättungsfaktor für das RF-Signal
float RF_FILTER_ALPHA = 0.90f;

// -----------------------------------------------------
// RF-Bewertung aus Aussentest / TV-Bild-Grenzen
// -----------------------------------------------------
// Kommentarstand: V3
//
// Diese ADC-Grenzen stammen aus den praktischen Aussentests mit eingeschaltetem
// Sat-Receiver und beobachtetem TV-Bild. Sie dienen NICHT als Sperre fuer die
// Benutzerentscheidung: Wenn der Nutzer im Kandidatenmodus PLUS drueckt, wird
// der Satellit bestaetigt. Die Werte sind nur eine Ampel fuer Anzeige und
// Diagnose.
//
// Wichtig fuer den aktuellen AD8317/AD8318-Aufbau:
// kleinerer ADC-/RF-Wert = staerkeres Signal.
float RF_TV_USABLE_MAX_ADC = 900.0f;   // oberhalb: Signal erkannt, aber eher schwach
float RF_TV_GOOD_MAX_ADC   = 800.0f;   // unterhalb: guter Kandidat
float RF_TV_STRONG_MAX_ADC = 750.0f;   // unterhalb: sehr guter/Peak-naher Bereich

// -----------------------------------------------------
// Reservierte Altwerte der entfernten Signaloptimierung
// -----------------------------------------------------
// Kommentarstand: V3
//
// Diese Werte werden in der aktuellen Bedienlinie nicht aktiv verwendet.
// Die automatische Optimierung nach PLUS wurde nach dem Live-Test entfernt,
// weil der Nutzer bestaetigte TV-Punkt dadurch verlassen werden konnte.
// PLUS bestaetigt jetzt nur den Kandidaten und behaelt die Position.
//
// Die Werte bleiben vorerst als reservierte Altwerte erhalten, falls spaeter
// eine neu geplante und getestete Optimierung wieder aufgebaut werden soll.
// RF-Logik im aktuellen Aufbau: kleinerer ADC-Wert = staerkeres Signal.
unsigned long SIGNAL_OPT_AZ_STEP_MS = 80;
unsigned long SIGNAL_OPT_AZ_SETTLE_MS = 700;
unsigned long SIGNAL_OPT_EL_STEP_MS = 60;
unsigned long SIGNAL_OPT_EL_SETTLE_MS = 1000;
float SIGNAL_OPT_RF_IMPROVE_ADC = 40.0f;
float SIGNAL_OPT_RF_WORSE_ADC = 70.0f;
float SIGNAL_OPT_RETURN_EL_TOLERANCE_DEG = 1.0f;
// V3: Primaere AZ-Grenze ist der Ost-/West-Hallsensor.
// Die folgende maximale AZ-Schrittzahl ist nur ein Sicherheitslimit.
// Fuer die konservative Optimierung sind die Testwege bewusst kurz.
int SIGNAL_OPT_AZ_MAX_STEPS_PER_DIR = 2;
int SIGNAL_OPT_EL_MAX_STEPS_PER_DIR = 1;

// =====================================================
// Speicherstrategie / No-NVS-Betrieb
// =====================================================
//
// Aktueller Projektstand:
// Diese Version verwendet bewusst KEIN Preferences/NVS.
//
// Grund:
// - Alle Test-, Sicherheits- und Projektwerte sollen eindeutig aus
//   dem Sketch bzw. aus settings.cpp kommen.
// - Alte Werte im ESP32-Flash duerfen keine geaenderten Codewerte
//   mehr ueberdecken.
// - Falls spaeter wieder ein echter Speicherbedarf entsteht
//   (z. B. RF-Zero oder Benutzerkalibrierung), wird das gezielt
//   neu entschieden und separat eingebaut.
//
// Die Funktionen initSettings(), saveAllSettings() und
// resetSettingsToDefaults() bleiben als Schnittstelle erhalten,
// damit andere Projektdateien nicht angepasst werden muessen.

// Setzt ALLE Runtime-Settings wieder auf ihre definierten Defaultwerte.
//
// WICHTIG:
// Diese Funktion schreibt noch nicht in den Flash.
// Sie setzt nur die Variablen im RAM zurück.
static void applyDefaultSettings() {
  // Allgemein
  SERIAL_BAUDRATE = 115200;
  HEARTBEAT_MS = 2000;

  // Betriebsmodi
  DEFAULT_CONTROL_MODE = CONTROL_MAIN_MENU;
  MANUAL_OVERRIDE_TIMEOUT_MS = 15000;

  // Azimut
  AZ_PULSE_MS = 250;
  AZ_TEST_INTERVAL_MS = 2500;
  AZ_SETTLE_MS = 300;

  // Fuer den ersten groben Azimut-Test bewusst auf nahezu
  // durchlaufenden Betrieb eingestellt.
  //
  // Idee:
  // - erster Suchlauf / Endanschlagfahrt moeglichst schnell und grob
  // - spaeter erst feinere Impulsstrategie nahe am Ziel
  AZ_FIRST_PULSE_ON_MS = 3000;
  AZ_STEP_PULSE_ON_MS = 1200;
  AZ_STEP_PULSE_OFF_MS = 20;

  // A3144 mit externem Pull-up:
  // HIGH = frei, LOW = Magnet erkannt
  AZ_HALL_ACTIVE_LOW = true;

  // Externe 10k Pull-ups vorhanden -> interne Pull-ups aus
  AZ_HALL_USE_INTERNAL_PULLUP = false;

  AZ_HOME_FIRST_SEARCH_DIRECTION = AZ_DIR_EAST;

  // Elevation
  EL_MOVE_MS = 2000;
  EL_PAUSE_MS = 2000;

  ELEVATION_MIN_SOFT = 25.0f;
  ELEVATION_MAX_SOFT = 34.0f;

  DEFAULT_TARGET_ELEVATION = 30.0f;
  ELEVATION_TOLERANCE_DEG = 1.0f;

  EL_PWM_FAST = 140;
  EL_PWM_SLOW = 90;

  EL_PULSE_FAST_MS = 180;
  EL_PULSE_SLOW_MS = 80;

  ELEVATION_SLOW_BAND_DEG = 3.0f;

  // Anzeige-Offset
  DISPLAY_ANGLE_OFFSET_DEG = 11.6f;
  SESSION_START_ELEVATION_DEG = 29.0f;  // nicht aktiv verwendet

  // Web / Taster
  WEB_EL_PULSE_MS = 250;
  WEB_EL_PWM = 110;

  BTN_DEBOUNCE_MS = 35;
  BTN_MODE_LONGPRESS_MS = 700;

  // RF
  RF_ADC_SAMPLES_PER_CYCLE = 32;
  RF_FILTER_ALPHA = 0.90f;

  // V3: RF-Ampelwerte aus Aussentest. Diese Werte bewerten die
  // Signalqualitaet, blockieren PLUS bewusst nicht; der Nutzer entscheidet am TV/Receiver.
  RF_TV_USABLE_MAX_ADC = 900.0f;
  RF_TV_GOOD_MAX_ADC   = 800.0f;
  RF_TV_STRONG_MAX_ADC = 750.0f;

  // Reservierte Altwerte der entfernten Signaloptimierung.
  // V3: Startwerte fuer den Aussentest; bewusst zentral anpassbar.
  SIGNAL_OPT_AZ_STEP_MS = 80;
  SIGNAL_OPT_AZ_SETTLE_MS = 700;
  SIGNAL_OPT_EL_STEP_MS = 60;
  SIGNAL_OPT_EL_SETTLE_MS = 1000;
  SIGNAL_OPT_RF_IMPROVE_ADC = 40.0f;
  SIGNAL_OPT_RF_WORSE_ADC = 70.0f;
  SIGNAL_OPT_RETURN_EL_TOLERANCE_DEG = 1.0f;
  SIGNAL_OPT_AZ_MAX_STEPS_PER_DIR = 2;
  SIGNAL_OPT_EL_MAX_STEPS_PER_DIR = 1;
}

// Speichert aktuell nichts dauerhaft.
//
//
// WICHTIG:
// Diese Funktion sollte nicht ständig aufgerufen werden,
// um unnötige Flash-Schreibzyklen zu vermeiden.
void saveAllSettings() {
  // No-NVS-Version:
  // Bewusst kein Flash-Schreibzugriff.
  // Diese Funktion bleibt nur als kompatible Schnittstelle erhalten.
}

// Initialisiert das Settings-System.
//
// Ablauf in der No-NVS-Version:
// 1. Defaults im RAM setzen
// 2. keine Werte aus Preferences/NVS laden
// 3. keinen Flash-Schreibzugriff ausfuehren
//
// Rueckgabe:
// false = es wurden Code-Defaults verwendet, nicht Flash-Werte.
bool initSettings() {
  applyDefaultSettings();
  return false;
}

// Setzt alle Werte wieder auf Defaults.
//
// Praktisch nützlich für:
// - Fehlersuche
// - definierte Ausgangslage
// - Rücksetzen nach Experimenten
//
// No-NVS-Version:
// Es wird nur RAM auf Code-Defaults gesetzt.
// Es wird nichts in den ESP32-Flash geschrieben.
void resetSettingsToDefaults() {
  applyDefaultSettings();
}

// Gibt alle aktuell aktiven Settings seriell aus.
// Sehr hilfreich zur Diagnose nach dem Boot oder nach Änderungen.
void printSettingsToSerial() {
  Serial.println("===== SETTINGS =====");

  // Allgemein
  Serial.print("SERIAL_BAUDRATE = ");
  Serial.println(SERIAL_BAUDRATE);
  Serial.print("HEARTBEAT_MS = ");
  Serial.println(HEARTBEAT_MS);

  // Modus
  Serial.print("DEFAULT_CONTROL_MODE = ");
  Serial.println(DEFAULT_CONTROL_MODE == CONTROL_AUTO ? "AUTO" : (DEFAULT_CONTROL_MODE == CONTROL_CENTER ? "MITTE" : (DEFAULT_CONTROL_MODE == CONTROL_MANUAL ? "MANUELL" : "HAUPTMENUE")));
  Serial.print("MANUAL_OVERRIDE_TIMEOUT_MS = ");
  Serial.println(MANUAL_OVERRIDE_TIMEOUT_MS);

  // Azimut
  Serial.print("AZ_PULSE_MS = ");
  Serial.println(AZ_PULSE_MS);
  Serial.print("AZ_FIRST_PULSE_ON_MS = ");
  Serial.println(AZ_FIRST_PULSE_ON_MS);
  Serial.print("AZ_STEP_PULSE_ON_MS = ");
  Serial.println(AZ_STEP_PULSE_ON_MS);
  Serial.print("AZ_STEP_PULSE_OFF_MS = ");
  Serial.println(AZ_STEP_PULSE_OFF_MS);
  Serial.print("AZ_HALL_ACTIVE_LOW = ");
  Serial.println(AZ_HALL_ACTIVE_LOW ? "true" : "false");
  Serial.print("AZ_HALL_USE_INTERNAL_PULLUP = ");
  Serial.println(AZ_HALL_USE_INTERNAL_PULLUP ? "true" : "false");
  Serial.print("AZ_HOME_FIRST_SEARCH_DIRECTION = ");
  Serial.println(AZ_HOME_FIRST_SEARCH_DIRECTION == AZ_DIR_EAST ? "EAST" : "WEST");

  // Elevation
  Serial.print("ELEVATION_MIN_SOFT = ");
  Serial.println(ELEVATION_MIN_SOFT, 2);
  Serial.print("ELEVATION_MAX_SOFT = ");
  Serial.println(ELEVATION_MAX_SOFT, 2);
  Serial.print("DEFAULT_TARGET_ELEVATION = ");
  Serial.println(DEFAULT_TARGET_ELEVATION, 2);
  Serial.print("ELEVATION_TOLERANCE_DEG = ");
  Serial.println(ELEVATION_TOLERANCE_DEG, 2);

  Serial.print("EL_PWM_FAST = ");
  Serial.println(EL_PWM_FAST);
  Serial.print("EL_PWM_SLOW = ");
  Serial.println(EL_PWM_SLOW);
  Serial.print("EL_PULSE_FAST_MS = ");
  Serial.println(EL_PULSE_FAST_MS);
  Serial.print("EL_PULSE_SLOW_MS = ");
  Serial.println(EL_PULSE_SLOW_MS);
  Serial.print("ELEVATION_SLOW_BAND_DEG = ");
  Serial.println(ELEVATION_SLOW_BAND_DEG, 2);

  // Anzeige-Offset
  Serial.print("DISPLAY_ANGLE_OFFSET_DEG = ");
  Serial.println(DISPLAY_ANGLE_OFFSET_DEG, 2);

  // Web / Taster
  Serial.print("WEB_EL_PULSE_MS = ");
  Serial.println(WEB_EL_PULSE_MS);
  Serial.print("WEB_EL_PWM = ");
  Serial.println(WEB_EL_PWM);

  Serial.print("BTN_DEBOUNCE_MS = ");
  Serial.println(BTN_DEBOUNCE_MS);
  Serial.print("BTN_MODE_LONGPRESS_MS = ");
  Serial.println(BTN_MODE_LONGPRESS_MS);

  // RF
  Serial.print("RF_ADC_SAMPLES_PER_CYCLE = ");
  Serial.println(RF_ADC_SAMPLES_PER_CYCLE);
  Serial.print("RF_FILTER_ALPHA = ");
  Serial.println(RF_FILTER_ALPHA, 3);
  Serial.print("RF_TV_USABLE_MAX_ADC = ");
  Serial.println(RF_TV_USABLE_MAX_ADC, 1);
  Serial.print("RF_TV_GOOD_MAX_ADC = ");
  Serial.println(RF_TV_GOOD_MAX_ADC, 1);
  Serial.print("RF_TV_STRONG_MAX_ADC = ");
  Serial.println(RF_TV_STRONG_MAX_ADC, 1);

  Serial.print("SIGNAL_OPT_AZ_STEP_MS = ");
  Serial.println(SIGNAL_OPT_AZ_STEP_MS);
  Serial.print("SIGNAL_OPT_AZ_SETTLE_MS = ");
  Serial.println(SIGNAL_OPT_AZ_SETTLE_MS);
  Serial.print("SIGNAL_OPT_EL_STEP_MS = ");
  Serial.println(SIGNAL_OPT_EL_STEP_MS);
  Serial.print("SIGNAL_OPT_EL_SETTLE_MS = ");
  Serial.println(SIGNAL_OPT_EL_SETTLE_MS);
  Serial.print("SIGNAL_OPT_RF_IMPROVE_ADC = ");
  Serial.println(SIGNAL_OPT_RF_IMPROVE_ADC, 1);
  Serial.print("SIGNAL_OPT_RF_WORSE_ADC = ");
  Serial.println(SIGNAL_OPT_RF_WORSE_ADC, 1);
  Serial.print("SIGNAL_OPT_RETURN_EL_TOLERANCE_DEG = ");
  Serial.println(SIGNAL_OPT_RETURN_EL_TOLERANCE_DEG, 2);
  Serial.print("SIGNAL_OPT_AZ_MAX_STEPS_PER_DIR = ");
  Serial.println(SIGNAL_OPT_AZ_MAX_STEPS_PER_DIR);
  Serial.print("SIGNAL_OPT_EL_MAX_STEPS_PER_DIR = ");
  Serial.println(SIGNAL_OPT_EL_MAX_STEPS_PER_DIR);

  Serial.println("====================");
}
