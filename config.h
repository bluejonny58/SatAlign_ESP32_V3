/*
  SatAlign V3 - zentrale Konfiguration
  ------------------------------------------------------------
  Diese Datei sammelt globale Projektparameter, die von mehreren Modulen
  benoetigt werden. Werte, die beim Test haeufig angepasst werden, sollen
  bevorzugt ueber settings.cpp/settings.h gepflegt werden. config.h stellt
  dazu die gemeinsamen Deklarationen bereit.

  Wichtige Regel fuer V3: Kommentare beschreiben den aktuellen Stand V3;
  Altbezeichnungen wie AUTO koennen intern bestehen bleiben, wenn sie fuer
  Kompatibilitaet der Zustandsmaschine noetig sind.
*/

#pragma once

// Benötigt die Typdefinitionen für ControlMode und AzimuthDirection.
// Diese Typen werden hier für globale Konfigurationsvariablen verwendet.
#include "types.h"

// =====================================================
// Webserver
// =====================================================
// V3: Port des lokalen Webservers.
// Dieser Wert ist keine geheime Information und bleibt bewusst im Projektcode.
// Standard fuer HTTP ist Port 80.
static const uint16_t WEB_SERVER_PORT = 80;


// =====================================================
// Allgemein
// =====================================================
// Serielle Baudrate für den Monitor.
// Wird in settings.cpp mit einem Defaultwert belegt und kann
// prinzipiell später zentral angepasst werden.
extern unsigned long SERIAL_BAUDRATE;

// Abstand der Heartbeat-Ausgaben im seriellen Monitor.
// Dient nur der Diagnose und hat keine direkte Steuerfunktion.
extern unsigned long HEARTBEAT_MS;

// =====================================================
// Betriebsmodi
// =====================================================
// Startmodus des Systems nach dem Boot.
// Aktuell wird dieser Wert aus settings.cpp versorgt.
// In der praktischen Logik wird danach aber oft sehr schnell
// explizit in MANUELL oder AUTO umgeschaltet.
extern ControlMode DEFAULT_CONTROL_MODE;

// Zeitstempel-/Override-Hilfswert aus älteren Zustandsideen.
// Ist aktuell nicht der wichtigste Live-Parameter, bleibt aber
// als globaler Konfigurationswert erhalten.
extern unsigned long MANUAL_OVERRIDE_TIMEOUT_MS;

// =====================================================
// Persistente Systemwerte
// =====================================================
// WICHTIG:
// Diese Variablen sind hier nur deklariert.
// Die eigentlichen Definitionen und Defaultwerte liegen in settings.cpp.
// Von dort werden sie beim Start aus dem Flash geladen oder auf Default gesetzt.

// -----------------------------------------------------
// Azimut
// -----------------------------------------------------

// Standard-Pulsdauer für manuelle Azimut-Korrekturen.
extern unsigned long AZ_PULSE_MS;

// Historischer/zusätzlicher Intervallwert für Azimut-Tests oder Logik.
// Kann später bei Bedarf weiter reduziert oder entfernt werden,
// wenn das System endgültig bereinigt wird.
extern unsigned long AZ_TEST_INTERVAL_MS;

// Allgemeine Einschwing-/Wartezeit für Azimut-nahe Abläufe.
extern unsigned long AZ_SETTLE_MS;

// Länge des ersten Pulses bei gestufter Azimutfahrt.
// Der erste Puls ist meist etwas länger, damit eine sichere
// sichtbare Bewegung einsetzt.
extern unsigned long AZ_FIRST_PULSE_ON_MS;

// Länge der normalen Folgeschritte im gestuften Azimutbetrieb.
extern unsigned long AZ_STEP_PULSE_ON_MS;

// Pausenzeit zwischen zwei gestuften Azimut-Pulsen.
extern unsigned long AZ_STEP_PULSE_OFF_MS;

// Legt fest, ob die Hall-Sensoren active-low arbeiten.
// Bei true gilt:
// LOW = Sensor aktiv
// HIGH = Sensor frei
extern bool AZ_HALL_ACTIVE_LOW;

// Legt fest, ob für die Hall-Sensor-Eingänge interne Pull-ups
// des ESP32 verwendet werden.
// Aktueller Projektstand: externe 10k-Pull-ups auf der Platine,
// daher ist dieser Wert in settings.cpp aktuell false.
extern bool AZ_HALL_USE_INTERNAL_PULLUP;

// Bevorzugte erste Suchrichtung für die Referenz-/Mittensuche.
// Dient der AUTO-Strategie und der Hall-basierten Azimut-Homing-Logik.
extern AzimuthDirection AZ_HOME_FIRST_SEARCH_DIRECTION;

// -----------------------------------------------------
// Elevation
// -----------------------------------------------------

// Allgemeiner historischer Bewegungswert für Elevation.
// Aktuell ist die eigentliche Puls-/Regellogik wichtiger,
// der Wert bleibt aber als globaler Parameter verfügbar.
extern unsigned long EL_MOVE_MS;

// Allgemeiner Pausenwert für Elevation.
// Kann für ältere Tests oder spätere Hilfsroutinen relevant sein.
extern unsigned long EL_PAUSE_MS;

// Untere softwareseitige Arbeitsgrenze für die Elevation.
// WICHTIG:
// Das ist nicht der interne Endschalter des Aktuators,
// sondern die normale erlaubte Arbeitsgrenze der Software.
extern float ELEVATION_MIN_SOFT;

// Obere softwareseitige Arbeitsgrenze für die Elevation.
extern float ELEVATION_MAX_SOFT;

// Zentraler Standard-Sollwert fuer die Elevation.
// Kommentarstand: V3
// Wird beim Boot vor dem Hauptmenue automatisch angefahren und im Web-UI
// Such-Setup als Standard-EZ angezeigt.
extern float DEFAULT_TARGET_ELEVATION;

// Toleranz, innerhalb der ein Ziel als erreicht gilt.
extern float ELEVATION_TOLERANCE_DEG;

// Boot-Toleranz fuer die automatische Standard-EZ-Anfahrt.
// Kommentarstand: V3
// Wird vor dem Hauptmenue verwendet, damit die Antenne nach jedem Einschalten
// von einem definierten Elevationswert startet.
extern float BOOT_ELEVATION_TOLERANCE_DEG;

// Sicherheits-Timeout fuer die Boot-Anfahrt auf DEFAULT_TARGET_ELEVATION.
// Kommentarstand: V3
extern unsigned long BOOT_ELEVATION_TIMEOUT_MS;

// Anzeige-/Bedien-Timeout fuer den sichtbaren EZ-Start-Countdown.
// Kommentarstand: V3
// Dieser Wert steuert die Boot-EZ-Anfahrt und wird auf dem TFT heruntergezaehlt,
// damit der User erkennt, wie lange der automatische EZ-Start maximal laeuft.
extern unsigned long BOOT_ELEVATION_DISPLAY_TIMEOUT_MS;

// Zeitfenster fuer den manuellen Winkel-Startbildschirm beim Einschalten.
// Kommentarstand: V3
// Der Nutzer kann in dieser Zeit den Winkel mit PLUS/MINUS korrigieren;
// danach geht die Anlage automatisch ins Hauptmenue.
extern unsigned long BOOT_MANUAL_ELEVATION_WINDOW_MS;

// PWM-Wert für schnelle Elevationsbewegung.
extern int EL_PWM_FAST;

// PWM-Wert für langsame / feinere Elevationsbewegung.
extern int EL_PWM_SLOW;

// Pulsdauer für schnelle Elevationsschritte.
extern unsigned long EL_PULSE_FAST_MS;

// Pulsdauer für langsame / feinere Elevationsschritte.
extern unsigned long EL_PULSE_SLOW_MS;

// Fehlerband, unterhalb dessen die Regelung auf die langsamere,
// feinere Schrittgröße umschaltet.
extern float ELEVATION_SLOW_BAND_DEG;

// -----------------------------------------------------
// Anzeige-Offset Elevation
// -----------------------------------------------------
// Korrekturwert für die angezeigte Elevation.
// Dieser Wert wird auf den berechneten Winkel addiert und dient dazu,
// kleine systematische Abweichungen der Anzeige auszugleichen,
// ohne die restliche Sensorlogik umzubauen.
extern float DISPLAY_ANGLE_OFFSET_DEG;
// Nur noch dokumentierter Platzhalter, nicht aktiv fuer die EZ-Anzeige genutzt.
extern float SESSION_START_ELEVATION_DEG;

// -----------------------------------------------------
// Web / Taster Elevation
// -----------------------------------------------------

// Pulsdauer für Web- oder Taster-Kommandos der Elevation im manuellen Betrieb.
extern unsigned long WEB_EL_PULSE_MS;

// PWM-Wert für diese manuellen Web-/Taster-Elevationsbewegungen.
extern int WEB_EL_PWM;

// -----------------------------------------------------
// 3-Taster-Logik
// -----------------------------------------------------

// Entprellzeit für die Taster.
// Verhindert, dass ein mechanischer Tasterdruck mehrfach
// als mehrere Eingaben erkannt wird.
extern unsigned long BTN_DEBOUNCE_MS;

// Zeitgrenze für langen MODE-Druck.
// Kurz <-> Achse wechseln
// Lang <-> AUTO/MANUELL wechseln
extern unsigned long BTN_MODE_LONGPRESS_MS;

// -----------------------------------------------------
// RF / AD8318
// -----------------------------------------------------

// Anzahl ADC-Messungen pro RF-Zyklus für die Mittelung.
// Höher = ruhigeres Signal, aber träger.
// Kleiner = schneller, aber unruhiger.
extern int RF_ADC_SAMPLES_PER_CYCLE;

// IIR-/Exponentialfilterfaktor für das RF-Signal.
// Hoher Wert = stärker geglättet, langsamer
// Niedriger Wert = schneller, aber nervöser
extern float RF_FILTER_ALPHA;

// -----------------------------------------------------
// RF-Bewertung aus Aussentest / TV-Bild-Grenzen
// -----------------------------------------------------
// Kommentarstand: V3
// Diese Werte bewerten die RF-Signalqualitaet fuer Anzeige und Diagnose.
// Sie blockieren PLUS ausdruecklich nicht: Wenn der Nutzer ein TV-Bild sieht
// und PLUS drueckt, wird der Kandidat bestaetigt.
extern float RF_TV_USABLE_MAX_ADC;
extern float RF_TV_GOOD_MAX_ADC;
extern float RF_TV_STRONG_MAX_ADC;
// -----------------------------------------------------
// Reservierte Altwerte der entfernten Signaloptimierung
// -----------------------------------------------------
// Kommentarstand: V3
// Diese Werte bleiben nur als reservierte Altwerte im Code, weil die
// automatische Optimierung nach dem Live-Test aus der Bedienung entfernt wurde.
// PLUS bestaetigt aktuell nur den Kandidaten und laesst die Anlage an der
// geprueften Position stehen. Die Werte koennen spaeter geloescht oder fuer
// eine neu geplante Optimierung bewusst wiederverwendet werden.
extern unsigned long SIGNAL_OPT_AZ_STEP_MS;
extern unsigned long SIGNAL_OPT_AZ_SETTLE_MS;
extern unsigned long SIGNAL_OPT_EL_STEP_MS;
extern unsigned long SIGNAL_OPT_EL_SETTLE_MS;
extern float SIGNAL_OPT_RF_IMPROVE_ADC;
extern float SIGNAL_OPT_RF_WORSE_ADC;
extern float SIGNAL_OPT_RETURN_EL_TOLERANCE_DEG;
// V3: Primaere AZ-Grenze ist der Ost-/West-Hallsensor.
// Die folgende maximale AZ-Schrittzahl ist nur ein Sicherheitslimit.
extern int SIGNAL_OPT_AZ_MAX_STEPS_PER_DIR;
extern int SIGNAL_OPT_EL_MAX_STEPS_PER_DIR;
