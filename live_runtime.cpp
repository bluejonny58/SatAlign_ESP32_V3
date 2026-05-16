/*
  SatAlign V3 - Runtime und Zustandsmaschine
  ------------------------------------------------------------
  Dieses Modul enthaelt die fachliche Ablaufsteuerung:
  - Hauptmenue, Ausrichten/Mitte, Suchen, Manuell
  - Kandidatenentscheidung PLUS/MINUS
  - Signal optimieren nach PLUS
  - Live-Status fuer Web-UI und TFT

  Wichtige Trennung: web_server.cpp zeigt und bedient nur; die eigentliche
  Logik und Motorentscheidungen bleiben hier bzw. in den Motor-Modulen.
*/

/*
  SatAlign ESP32 - live_runtime.cpp
  Version: V3
  ---------------------------------------------------------------------------
  Zentrale Laufzeitlogik des Projekts. Diese Datei verbindet:

  - 3-Tasten-Bedienung am Geraet
  - Hauptmenue / Ausrichten / AUTO / manuelle Steuerung
  - aktuelle AUTO-Suchstrategie von V3 - Kandidatenentscheidung PLUS = OK, MINUS = falsch, MODE lang = Abbruch
  - Runtime-Status fuer TFT, Serial Monitor und Web-UI

  Aktuelle AUTO-/Such-Strategie V3:
  1. getestete Mitten-/Ausrichtfunktion als Referenz ausfuehren
  2. von der Mitte gezielt Richtung Osten suchen
  3. danach Richtung Westen bis zum West-Limit suchen
  4. von der West-Position wieder zur Mitte zurueckfahren
  5. in der Mitte stoppen und den Nutzer zur manuellen Hoehenkorrektur auffordern
  6. der Nutzer kann danach denselben Suchdurchlauf erneut starten

  Hinweis V3:
  Die fruehere Feinsuche AN/AUS und das automatische Elevationsraster wurden
  bewusst entfernt. Die Elevation wird nur grob vorgegeben bzw. manuell
  nachgefuehrt; die eigentliche Bewertung erfolgt ueber RF.

  RF-Prinzip im aktuellen Aufbau:
  - niedrigere AD8317/AD8318-Spannung = staerkeres Signal
  - hoehere AD8317/AD8318-Spannung = schwaecheres Signal

  Wichtige Projektentscheidung:
  Es werden keine Werte in NVS/Preferences gespeichert. Wenn sich bessere
  Werte ergeben, sollen sie bewusst im Sketch geaendert werden.
*/

#include <Arduino.h>
#include <math.h>

#include "pins.h"
#include "config.h"
#include "sensor_mpu6050.h"
#include "elevation_control.h"
#include "azimuth_control.h"
#include "rf_detector.h"
#include "control_state.h"
#include "live_runtime.h"

namespace {
  // =====================================================
  // Interne Bedienachsen im manuellen Modus
  // =====================================================
  // Im normalen manuellen Betrieb wird zwischen zwei Achsen umgeschaltet:
  // - AZIMUT
  // - ELEVATION
  //
  // PLUS / MINUS beziehen sich dann auf die jeweils aktive Achse.
  enum ManualAxis {
    MANUAL_AXIS_AZ = 0,
    MANUAL_AXIS_EL
  };

  // =====================================================
  // Hauptmenue-Auswahl
  // =====================================================
  // Das Hauptmenue hat vier feste Punkte:
  // 1 = Ausrichten: Nach Sueden grob ausrichten, Elevation mit +/- korrigieren,
  //     MODE startet danach die getestete Mitten-/Referenzfahrt.
  // 2 = Automatik starten
  // 3 = manuelle Azimutsteuerung
  // 4 = manuelle Elevationssteuerung
  enum MainMenuSelection {
    MENU_ITEM_CENTER    = 1,
    MENU_ITEM_AUTO      = 2,
    MENU_ITEM_MANUAL_AZ = 3,
    MENU_ITEM_MANUAL_EL = 4
  };

  MainMenuSelection mainMenuSelection = MENU_ITEM_CENTER;

  // Im Menuepunkt 1 wird zuerst nur die Ausrichtung vorbereitet.
  // Die eigentliche Azimut-Referenzfahrt startet erst mit MODE.
  bool centerHomingStarted = false;

  enum CenterOwner {
    CENTER_OWNER_MENU = 0,
    CENTER_OWNER_AUTO
  };

  CenterOwner centerOwner = CENTER_OWNER_MENU;

  // =====================================================
  // AZ-Mitte-Zeitmessung
  // =====================================================
  // Zweck:
  // Die Antenne soll nicht nur irgendeinen Punkt am Center-Hall treffen,
  // sondern rechnerisch in die Mitte des Center-Hall-Einflussbereichs fahren.
  // Dazu wird der komplette aktive Bereich des Center-Halls durchfahren und
  // dessen Dauer gemessen. Danach fährt AZ um die halbe gemessene Zeit zurück.
  //
  // Verbindlicher Ablauf der getesteten Mittenfunktion:
  // - Grundrichtung ist immer logisch OSTEN.
  // - Ist CENTER beim Start bereits aktiv, fährt AZ zuerst nach OSTEN aus
  //   dem Center-Bereich heraus.
  // - Danach fährt AZ zurück nach WESTEN, misst Eintritt und Austritt des
  //   Center-Bereichs und fährt anschließend halbe Zeit zurück.
  // - Ist beim Start kein CENTER aktiv, fährt AZ zuerst nach OSTEN bis zum
  //   nächsten Hall-Sensor. Falls zuerst ein Endsensor erreicht wird, wird
  //   einmal in Gegenrichtung gesucht.
  // - Es gibt nur einen zentralen Sicherheits-Timeout pro Such-/Messfahrt.
  //
  // Wichtig zur zentralen Richtungsreferenz:
  // Die Bezeichnung EAST/WEST ist hier logisch gemeint.
  // Die reale Pin-Zuordnung wird nicht mehr lokal in centerAzDrive() korrigiert,
  // sondern zentral in azimuth_control.cpp. Grundlage ist die getestete
  // getesteten Mittenfunktion, bei der die mechanische Richtung korrekt war.
  enum CenterTimingState {
    CENTER_TIMING_IDLE = 0,
    CENTER_TIMING_LEAVE_ACTIVE,
    CENTER_TIMING_SEARCH_ENTER,
    CENTER_TIMING_CROSS_EXIT,
    CENTER_TIMING_RETURN_HALF,
    CENTER_TIMING_DONE,
    CENTER_TIMING_FAILED
  };

  CenterTimingState centerTimingState = CENTER_TIMING_IDLE;
  AzimuthDirection centerTimingDir = AZ_DIR_NONE;
  AzimuthDirection centerReturnDir = AZ_DIR_NONE;
  unsigned long centerStateStartedAtMs = 0;
  unsigned long centerEntryAtMs = 0;
  unsigned long centerZoneWidthMs = 0;
  unsigned long centerReturnMs = 0;
  uint8_t centerSearchReverseCount = 0;

  // Ein zentraler Sicherheits-Timeout fuer jede echte AZ-Hall-Suchfahrt.
  // 60 Sekunden als reine Software-Notbremse.
  // Die eigentliche Begrenzung erfolgt ueber Hall-/Endsensoren bzw.
  // mechanische Endabschalter.
  const unsigned long AZ_HALL_SEARCH_TIMEOUT_MS = 60000UL;

  const unsigned long CENTER_DIRECTION_PAUSE_MS = 350;
  const unsigned long CENTER_RETURN_MIN_MS = 120;

  // Sicherheit fuer die Elevationskorrektur im Menuepunkt "Mitte einstellen".
  // PLUS/MINUS loesen dort keine Dauerfahrt aus, sondern nur kurze
  // nicht-blockierende Einzelpulse. Dadurch bleibt MODE jederzeit als
  // Stop/Abbruch auswertbar und ein haengender Taster kann keine
  // unkontrollierte Dauerfahrt verursachen.
  const unsigned long CENTER_EL_ADJUST_PULSE_MS = 250;
  const unsigned long CENTER_EL_BUTTON_HOLD_MAX_MS = 800;
  unsigned long centerElButtonPressStartMs = 0;


  // Diagnoseanzeige fuer die AZ-Mittenfahrt.
  // Wird nur angezeigt/ausgegeben; aendert keine Fahrentscheidung.
  const char* centerLastFailText = "";
  // Wird fuer die Web-UI gesetzt, wenn die Ausrichten-/Mittenfahrt
  // erfolgreich aus dem Ausrichten-Menue beendet wurde.
  // Dadurch kann die Web-UI nach Abschluss eine klare Erfolgsmeldung anzeigen.
  bool centerSuccessNoticeActive = false;

  static const char* centerTimingStateText(CenterTimingState st) {
    switch (st) {
      case CENTER_TIMING_IDLE:         return "IDLE";
      case CENTER_TIMING_LEAVE_ACTIVE: return "LEAVE";
      case CENTER_TIMING_SEARCH_ENTER: return "SEARCH";
      case CENTER_TIMING_CROSS_EXIT:   return "MEASURE";
      case CENTER_TIMING_RETURN_HALF:  return "RETURN";
      case CENTER_TIMING_DONE:         return "DONE";
      case CENTER_TIMING_FAILED:       return "FAILED";
      default:                         return "?";
    }
  }

  static unsigned long centerTimingTimeoutMs(CenterTimingState st) {
    switch (st) {
      case CENTER_TIMING_LEAVE_ACTIVE:
      case CENTER_TIMING_SEARCH_ENTER:
      case CENTER_TIMING_CROSS_EXIT:
        return AZ_HALL_SEARCH_TIMEOUT_MS;
      case CENTER_TIMING_RETURN_HALF:
        return centerReturnMs;
      default:
        return 0;
    }
  }

  // =====================================================
  // AUTO-/Such-Zustandsmaschine V3
  // =====================================================
  // Neuer vereinfachter Ablauf nach den Aussentests:
  // - zuerst die getestete Mitten-/Ausrichtfunktion als Referenz ausfuehren
  // - danach von der Mitte gezielt Richtung Osten suchen
  // - danach Richtung Westen bis zum West-Limit suchen
  // - aus der West-Position zurueck in die Mitte fahren
  // - in der Mitte stoppen und den Nutzer zur manuellen Hoehenkorrektur auffordern
  //
  // V3: Die fruehere Feinsuche AN/AUS und das automatische Hoehenraster
  // wurden entfernt. Die Elevation wird bewusst grob bzw. manuell behandelt,
  // weil die RF-Signalsuche den relevanten Satellitenbereich findet.
  enum AutoState {
    AUTO_STATE_INACTIVE = 0,
    AUTO_STATE_CANDIDATE_HOLD,

    AUTO_STATE_NEW_CENTER_START,
    AUTO_STATE_NEW_CENTER_WAIT,
    AUTO_STATE_NEW_MOVE_ASTRA_EAST_START,
    AUTO_STATE_NEW_MOVE_ASTRA_EAST_WAIT,
    AUTO_STATE_NEW_SCAN_EAST_START,
    AUTO_STATE_NEW_SCAN_EAST_WAIT,
    AUTO_STATE_NEW_SCAN_WEST_START,
    AUTO_STATE_NEW_SCAN_WEST_WAIT,
    AUTO_STATE_NEW_STEP_EAST_START,
    AUTO_STATE_NEW_STEP_EAST_WAIT,
    AUTO_STATE_NEW_RETURN_CENTER_START,
    AUTO_STATE_NEW_RETURN_CENTER_WAIT,
    AUTO_STATE_NEW_CHANGE_ELEVATION_START,   // Altzustand bleibt aus Kompatibilitaet, wird in V3 nicht mehr aktiv genutzt.
    AUTO_STATE_NEW_CHANGE_ELEVATION_WAIT,    // Altzustand bleibt aus Kompatibilitaet, wird in V3 nicht mehr aktiv genutzt.
    AUTO_STATE_EZ_ADJUST_HINT,               // V3: Mitte erreicht, Hoehe manuell pruefen und Suchlauf ggf. wiederholen.

    // V3: Signaloptimierung nach PLUS-Bestaetigung eines Kandidaten.
    // Der Nutzer entscheidet weiterhin, ob der Satellit richtig ist. Erst dann
    // sucht die Anlage automatisch ein lokales RF-Minimum: zuerst Azimut,
    // danach Winkel/Elevation. Die Schrittweiten liegen zentral in settings.cpp.
    AUTO_STATE_SIGNAL_OPT_AZ_EAST_START,
    AUTO_STATE_SIGNAL_OPT_AZ_EAST_WAIT,
    AUTO_STATE_SIGNAL_OPT_AZ_WEST_START,
    AUTO_STATE_SIGNAL_OPT_AZ_WEST_WAIT,
    AUTO_STATE_SIGNAL_OPT_EL_UP_START,
    AUTO_STATE_SIGNAL_OPT_EL_UP_WAIT,
    AUTO_STATE_SIGNAL_OPT_EL_DOWN_START,
    AUTO_STATE_SIGNAL_OPT_EL_DOWN_WAIT,

    AUTO_STATE_COMPLETE,
    AUTO_STATE_FAILED
  };

  // =====================================================
  // Allgemeines Ergebnis für interne Teilaufgaben
  // =====================================================
  enum TaskResult {
    TASK_RUNNING = 0,
    TASK_DONE,
    TASK_FAILED
  };

  // =====================================================
  // Entprellter Tasterzustand
  // =====================================================
  struct DebouncedButton {
    int pin;
    bool stablePressed;
    bool lastReading;
    unsigned long lastDebounceMs;
    bool pressEvent;
    bool releaseEvent;
  };

  // =====================================================
  // Einfache Schrittaufgabe für Azimut
  // =====================================================
  // Wird verwendet für:
  // - Astra-Startposition
  // - definierte Resume-Fahrten nach einem gesperrten Bereich
  struct StepTask {
    bool active = false;
    bool failed = false;
    AzimuthDirection dir = AZ_DIR_NONE;
    int remainingSteps = 0;
    unsigned long pulseMs = 0;
    bool pulseIssued = false;
    unsigned long settleUntilMs = 0;
  };

  // =====================================================
  // Scan-Aufgabe für Azimut-Suche
  // =====================================================
  // Diese Struktur bildet weiterhin den groben Suchlauf ab:
  // - zuerst in positive Richtung
  // - dann in negative Richtung über die Gegenflanke hinaus
  // - danach zurück auf den lokal besten Punkt
  //
  // Der "beste" Punkt ist weiterhin der Punkt mit der kleinsten RF-Spannung,
  // weil im aktuellen Projekt gilt:
  // kleinere Spannung = besserer Kandidat.
  struct ScanTask {
    bool active = false;
    bool failed = false;
    int stage = 0;
    int sideSteps = 0;
    unsigned long pulseMs = 0;
    AzimuthDirection positiveDir = AZ_DIR_EAST;
    bool pulseIssued = false;
    AzimuthDirection lastPulseDir = AZ_DIR_NONE;
    unsigned long settleUntilMs = 0;
    int stepsMoved = 0;
    int stepsInStage = 0;
    int bestRelativeStep = 0;
    float startVoltage = 0.0f;
    float bestVoltage = 99.0f;
  };

  // =====================================================
  // Gesperrte Bereiche für falsche Satelliten
  // =====================================================
  // Jeder falsche Satellit wird nicht nur als Punkt, sondern als Bereich
  // gespeichert. Dadurch kann die Suche denselben Peak später deutlich
  // robuster überspringen.
  struct BlockedSatRange {
    bool active = false;

    // Mittelpunkt des Peak-Bereichs relativ zur Referenz
    int azCenterSteps = 0;

    // tatsächlich gesperrter Bereich
    int azMinSteps = 0;
    int azMaxSteps = 0;

    // gemessene Verbesserung gegenüber dem damaligen Basiswert
    float improvementV = 0.0f;

    // 1 = schwach falsch
    // 2 = stark falsch
    // 3 = sehr stark falsch (derzeit vorbereitet, noch nicht aktiv genutzt)
    uint8_t strengthClass = 0;

    // Elevation beim Fund. In Version 1 nur mitloggen, noch nicht zur
    // Sperrprüfung verwenden.
    float elevationDeg = 0.0f;

    // Reihenfolge des Eintrags. Wird benötigt, um bei vollem Array den
    // ältesten Eintrag zu überschreiben.
    uint32_t createdOrder = 0;
  };

  // ---------------------------------------------------
  // Laufzeitstatus
  // ---------------------------------------------------
  bool initOk = false;
  bool mpuOk = false;
  unsigned long lastPrint = 0;

  ManualAxis manualAxis = MANUAL_AXIS_AZ;
  ManualAxis lastManualAxis = MANUAL_AXIS_AZ;

  ElevationDirection manualElDirection = EL_DIR_STOP;
  AzimuthDirection manualAzIntent = AZ_DIR_NONE;

  // Web-Kommandos sind Start-/Stop-Befehle, keine dauerhaft gedrueckten Tasten.
  // Diese Merker verhindern, dass die normale Tastenlogik den per Web gestarteten
  // Motor sofort wieder stoppt, nur weil keine physische Taste gedrueckt ist.
  bool webManualAzHoldActive = false;
  bool webManualElHoldActive = false;

  DebouncedButton btnMode  = { PIN_BTN_MODE,  false, false, 0, false, false };
  DebouncedButton btnMinus = { PIN_BTN_MINUS, false, false, 0, false, false };
  DebouncedButton btnPlus  = { PIN_BTN_PLUS,  false, false, 0, false, false };

  bool buttonLock = false;
  unsigned long modePressStartMs = 0;
  bool modeLongHandled = false;

  // Mehrfachklick-Erkennung für MODE.
  // Diese Logik wird NUR in den Kandidaten-Zuständen verwendet.
  uint8_t modeClickCount = 0;
  unsigned long modeClickDeadlineMs = 0;

  // ---------------------------------------------------
  // AUTO-Strategie
  // ---------------------------------------------------
  AutoState autoState = AUTO_STATE_INACTIVE;
  const char* autoFailReason = "";
  bool autoHasPeak = false;

  // Such-Setup:
  // V3: Im Setup gibt es keine Feinsuche-Auswahl mehr. PLUS/MINUS
  // korrigiert die Hoehe, MODE startet den Suchlauf. Die alte
  // autoFineSearchEnabled-Variable bleibt nur als Kompatibilitaetswert stehen
  // und wird fuer die neue Suchstrategie nicht mehr ausgewertet.
  bool autoSetupActive = false;
  bool autoFineSearchEnabled = false;

  // Sichtbare AZ-Limit-Warnung.
  // Wird gesetzt, wenn MITTE oder AUTO aus einem aktiv erkannten
  // Ost-/West-Endsensorbereich gestartet werden soll.
  // Die Warnung blockiert nur AUTO/MITTE-Start, nicht die manuelle AZ-Steuerung.
  bool azLimitWarningActive = false;

  float autoBaseElevationDeg = 0.0f;
  int autoElevationIndex = 0;
  float autoTargetElevationDeg = 0.0f;

  // Basiswert am aktuellen Suchlevel. Gegen diesen Wert wird die spätere
  // Verbesserung bewertet.
  float autoScanBaselineVoltage = 0.0f;

  // Gemessenes aktuelles Schwankungsband im Grobbereich. Dieser Wert ist die
  // Grundlage für die dynamische Umschaltung zwischen Grob und Kandidat.
  float autoNoiseBandVoltage = 0.0f;

  // Dynamische Hysterese-Schwellen für Kandidat / Feinsuche.
  float autoCandidateEnterThresholdV = 0.0f;
  float autoCandidateExitThresholdV = 0.0f;

  // Bester gemerkter Wert des aktuellen Zyklus.
  float autoBestVoltage = 99.0f;

  // Aktueller Kandidat (lokaler Peak) nach Grobsuche.
  int autoCandidateAzSteps = 0;
  float autoCandidateElevationDeg = 0.0f;
  float autoCandidateImprovementV = 0.0f;
  uint8_t autoCandidateStrengthClass = 0;

  // Kandidatenbestätigung über mehrere Zyklen.
  uint8_t autoCandidateConfirmCounter = 0;
  unsigned long autoCandidateNextConfirmAtMs = 0;

  // Ziel-Azimut für das Verlassen eines gesperrten Bereichs.
  int autoResumeTargetAzSteps = 0;

  // Globale relative Azimut-Position. Diese wird ab der Referenz (Center) geführt.
  // 0 = Referenzpunkt
  // >0 = östlich der Referenz
  // <0 = westlich der Referenz
  int azPositionSteps = 0;

  // Optionaler Verweis auf die grobe Suchrichtung für Resume / Folgesuche.
  AzimuthDirection autoCurrentSearchDirection = AZ_DIR_EAST;

  // ---------------------------------------------------
  // Neue AUTO-Suche
  // ---------------------------------------------------
  AutoState autoResumeAfterCandidateState = AUTO_STATE_INACTIVE;
  AutoState autoReturnAfterCenterState = AUTO_STATE_INACTIVE;
  AzimuthDirection autoDriveDir = AZ_DIR_NONE;
  bool autoDriveStepped = false;
  unsigned long autoDriveStartedAtMs = 0;
  unsigned long autoLastPseudoStepAtMs = 0;

  // Eigene Fein-Step-Steuerung fuer den zweiten Ruecklauf.
  // West-Hall -> Ost-Hall. Nicht die groben allgemeinen AZ_STEP_* Werte
  // verwenden, weil diese fuer fast durchlaufende Fahrten gedacht sind.
  bool autoFineStepOutputOn = false;
  unsigned long autoFineStepPhaseStartedAtMs = 0;
  unsigned long autoLastAzStepCounter = 0;
  unsigned long autoLastRfDiagMs = 0;
  float autoRfZeroAdc = 0.0f;
  float autoRfBestAdc = 4095.0f;
  float autoRfCurrentAdc = 0.0f;
  float autoRfDropAdc = 0.0f;
  float autoCandidateAdc = 0.0f;
  int autoCycleIndex = 0;

  // =====================================================
  // Signaloptimierung nach Kandidaten-Bestaetigung
  // =====================================================
  // Kommentarstand: V3
  //
  // PLUS im Kandidatenmodus bedeutet nicht mehr nur "fertig", sondern:
  // "richtiger Satellit erkannt, jetzt Signal optimieren".
  //
  // Die Optimierung arbeitet bewusst einfach und testbar:
  // - RF wird als ADC-Wert bewertet. Kleinerer ADC = staerkeres Signal.
  // - zuerst werden kleine Azimut-Schritte getestet
  // - danach werden kleine Winkel-/Elevationsschritte getestet
  // - alle Schrittzeiten und Grenzwerte liegen zentral in settings.cpp
  //
  // Wichtig:
  // Das ist keine alte Feinsuche mit separatem Menue. Die Routine startet nur
  // nach bewusster PLUS-Bestaetigung durch den Nutzer.
  float signalOptBestAdc = 4095.0f;
  float signalOptLastAdc = 4095.0f;
  float signalOptStartAdc = 4095.0f;
  int signalOptBestAzSteps = 0;
  float signalOptBestElevationDeg = 0.0f;
  uint8_t signalOptStepsInPhase = 0;
  unsigned long signalOptStepStartedAtMs = 0;
  unsigned long signalOptLastDiagMs = 0;


  // Hilfsobjekte
  StepTask autoMoveTask;
  ScanTask autoScanTask;

  // Array für gesperrte falsche Satellitenbereiche.
  static const int MAX_BLOCKED_SAT_RANGES = 8;
  BlockedSatRange blockedRanges[MAX_BLOCKED_SAT_RANGES];
  uint32_t blockedRangeInsertCounter = 0;

  // ---------------------------------------------------
  // Strategieparameter
  // ---------------------------------------------------
  // Diese Werte sind Startwerte und bewusst direkt hier kommentiert.

  // Von der Referenz aus angenommene grobe Astra-Startlage.
  static const AzimuthDirection AUTO_PRESET_DIRECTION = AZ_DIR_EAST;
  static const int AUTO_PRESET_STEPS = 10;
  static const unsigned long AUTO_PRESET_PULSE_MS = 220;

  // Neue AUTO-Suche:
  // Die Position wird bei Dauerfahrt in Pseudo-Schritten gefuehrt, damit
  // Sperrbereiche und Astra-Startposition auch ohne echten Encoder nutzbar sind.
  static const int AUTO_ASTRA_OFFSET_STEPS = 10;
  static const unsigned long AUTO_CONTINUOUS_PSEUDO_STEP_MS = 300;
  static const float AUTO_RF_CANDIDATE_DROP_ADC = 100.0f;
  static const float AUTO_RF_CANDIDATE_EXIT_ADC = 45.0f;
  static const unsigned long AUTO_RF_DIAG_INTERVAL_MS = 500;
  static const unsigned long AUTO_RETURN_CENTER_TIMEOUT_MS = 60000UL;

  // Alt-Konstanten der frueheren Fein-Step-Suche.
  // V3: Die Feinsuche wird nicht mehr aktiv benutzt. Die Konstanten bleiben
  // stehen, weil einzelne Hilfsfunktionen noch die alte Step-Mechanik kennen;
  // der neue Suchablauf springt diese Zustaende jedoch nicht mehr an.
  static const unsigned long AUTO_FINE_STEP_ON_MS  = 200;
  static const unsigned long AUTO_FINE_STEP_OFF_MS = 300;

  // Grobsuche: relativ große Schritte.
  static const int AUTO_SCAN_COARSE_SIDE_STEPS = 8;
  static const unsigned long AUTO_SCAN_COARSE_PULSE_MS = 220;

  // Feine manuelle Azimutkorrektur im Kandidatenmodus.
  static const unsigned long AUTO_SCAN_FINE_PULSE_MS = 90;

  // Einschwingzeit nach einem Azimutimpuls.
  static const unsigned long AUTO_AZ_SETTLE_MS = 180;

  // =====================================================
  // Dynamische Kandidaten-/Feinlogik
  // =====================================================
  // Fein bzw. Kandidaten-Hold wird NICHT mehr über einen festen absoluten
  // Wert ausgelöst, sondern über:
  // - das aktuelle Schwankungsband ohne echten Peak
  // - plus einen Sicherheitsfaktor
  //
  // Enter-Schwelle:
  //   max(Minimum, noiseBand * Faktor)
  //
  // Exit-Schwelle:
  //   etwas niedriger als die Enter-Schwelle, damit die Logik nicht flattert.
  static const float AUTO_RF_DYNAMIC_MIN_ENTER_V = 0.05f;
  static const float AUTO_RF_DYNAMIC_MIN_EXIT_V  = 0.04f;
  static const float AUTO_RF_DYNAMIC_ENTER_FACTOR = 1.5f;
  static const float AUTO_RF_DYNAMIC_EXIT_FACTOR  = 1.2f;

  // Kandidat gilt erst dann als stabil, wenn er mehrfach hintereinander
  // oberhalb der Enter-Schwelle bestätigt wurde.
  static const uint8_t AUTO_CANDIDATE_CONFIRM_COUNT = 3;
  static const unsigned long AUTO_CANDIDATE_CONFIRM_INTERVAL_MS = 120;

  // =====================================================
  // Falsche Satelliten / Sperrbereiche
  // =====================================================
  // Ab dieser Verbesserung darf ein Peak überhaupt als speicherbarer
  // falscher Kandidat betrachtet werden.
  static const float BADSAT_MIN_TRIGGER_V = 0.08f;

  // 1) schwach falsch
  static const float BADSAT_WEAK_MAX_V = 0.15f;
  static const int   BADSAT_WEAK_RADIUS_STEPS = 3;

  // 2) stark falsch
  static const float BADSAT_STRONG_MAX_V = 0.30f;
  static const int   BADSAT_STRONG_RADIUS_STEPS = 6;

  // 3) sehr stark falsch
  // Noch nicht aktiv benutzt, aber bewusst bereits vorbereitet.
  // Beim Aktivieren später einfach diese Klasse in classifyBadSatRadius()
  // einschalten.
  // static const float BADSAT_VERY_STRONG_MIN_V = 0.30f;
  // static const int   BADSAT_VERY_STRONG_RADIUS_STEPS = 9;

  // Sicherheitsabstand, um nach dem Speichern eines falschen Satelliten sicher
  // außerhalb des gesperrten Bereichs wieder einzusetzen.
  static const int BADSAT_RESUME_MARGIN_STEPS = 2;

  // Bereiche, die sich überlappen oder bis auf 3 Schritte annähern,
  // werden zusammengeführt.
  static const int BADSAT_MERGE_GAP_STEPS = 3;

  // =====================================================
  // Kandidatenbedienung
  // =====================================================
  // Aktueller Bedienstandard V3:
  // - PLUS  = richtiger Satellit / AUTO erfolgreich
  // - MINUS = falscher Satellit / Bereich sperren und weiter suchen
  // - MODE lang = AUTO abbrechen
  // Alte MODE-Mehrfachklick-Zustaende wurden aus dem aktiven Ablauf entfernt.

  // AUTO-Elevation-Suchraster relativ zur Start-Elevation.
  // Fuer Deutschland/Astra 19.2E wird bewusst in kleinen 1-Grad-Schritten gesucht,
  // weil 2 Grad bei der Elevation bereits zu grob sein koennen.
  // Ziele ausserhalb der EZ-Softlimits werden spaeter automatisch uebersprungen.
  static const float AUTO_ELEVATION_OFFSETS_DEG[] = {
    0.0f, +1.0f, -1.0f, +2.0f, -2.0f, +3.0f, -3.0f,
    +4.0f, -4.0f, +5.0f, -5.0f, +6.0f, -6.0f
  };

  static const int AUTO_ELEVATION_OFFSET_COUNT =
      sizeof(AUTO_ELEVATION_OFFSETS_DEG) / sizeof(AUTO_ELEVATION_OFFSETS_DEG[0]);
}

// =====================================================
// Hilfsfunktionen
// =====================================================

static AzimuthDirection oppositeDir(AzimuthDirection dir) {
  if (dir == AZ_DIR_EAST) return AZ_DIR_WEST;
  if (dir == AZ_DIR_WEST) return AZ_DIR_EAST;
  return AZ_DIR_NONE;
}

static const char* dirText(AzimuthDirection dir) {
  switch (dir) {
    case AZ_DIR_EAST: return "EAST";
    case AZ_DIR_WEST: return "WEST";
    default:          return "NONE";
  }
}

static bool isCandidateInteractionState() {
  return autoState == AUTO_STATE_CANDIDATE_HOLD;
}

static bool canMoveAzimuth(AzimuthDirection dir) {
  if (dir == AZ_DIR_EAST) return azimuthCanMoveEast();
  if (dir == AZ_DIR_WEST) return azimuthCanMoveWest();
  return false;
}

static void issueAzimuthPulse(AzimuthDirection dir, unsigned long pulseMs) {
  if (dir == AZ_DIR_EAST) {
    azimuthPulseEast(pulseMs);
  } else if (dir == AZ_DIR_WEST) {
    azimuthPulseWest(pulseMs);
  }
}

// Relative Azimutposition fortschreiben.
// Ost = +1, West = -1.
static void applyAzStepFromDir(AzimuthDirection dir) {
  if (dir == AZ_DIR_EAST) {
    azPositionSteps++;
  } else if (dir == AZ_DIR_WEST) {
    azPositionSteps--;
  }
}

static void clearModeMultiClick() {
  modeClickCount = 0;
  modeClickDeadlineMs = 0;
}

static void updateButton(DebouncedButton& btn) {
  btn.pressEvent = false;
  btn.releaseEvent = false;

  // Externe Pull-ups, active-low:
  // LOW = gedrückt, HIGH = frei
  const bool rawPressed = (digitalRead(btn.pin) == LOW);

  if (rawPressed != btn.lastReading) {
    btn.lastReading = rawPressed;
    btn.lastDebounceMs = millis();
  }

  if ((millis() - btn.lastDebounceMs) >= BTN_DEBOUNCE_MS) {
    if (btn.stablePressed != rawPressed) {
      btn.stablePressed = rawPressed;
      if (btn.stablePressed) {
        btn.pressEvent = true;
      } else {
        btn.releaseEvent = true;
      }
    }
  }
}

static int pressedCount() {
  int c = 0;
  if (btnMode.stablePressed)  c++;
  if (btnMinus.stablePressed) c++;
  if (btnPlus.stablePressed)  c++;
  return c;
}

static void stopManualActuators() {
  azimuthStop();
  elevationStop();
  manualAzIntent = AZ_DIR_NONE;
  manualElDirection = EL_DIR_STOP;
  webManualAzHoldActive = false;
  webManualElHoldActive = false;
}

static void setManualAxis(ManualAxis axis) {
  manualAxis = axis;
  lastManualAxis = axis;
}

static float currentRelativeAngle() {
  // Elevationsanzeige:
  // Fuer Bedienung, Anzeige, Softlimits und AUTO wird wieder der
  // echte angezeigte Elevationswinkel verwendet:
  // EZ = 90 - FiltX + DISPLAY_ANGLE_OFFSET_DEG.
  // Die fruehere Session-Referenz ist deaktiviert, weil sie die
  // Anzeige im Menue "Mitte einstellen" verfaelschen konnte.
  return mpuGetDisplayedAngleDeg();
}

// Setzt den aktuellen MPU-Winkel als definierten echten Startwinkel.
//
// Beispiel:
// SESSION_START_ELEVATION_DEG = 29.0
// -> aktuelle MPU-Lage wird ab jetzt als 29.0 Grad interpretiert.
//
// Dadurch bleibt der Bezug sichtbar im Sketch definiert und wird nicht
// ueber NVS/Preferences gespeichert.
static void setCurrentMpuAsSessionStartElevation() {
  // Bewusst deaktiviert.
  // Die Elevation wird nicht mehr ueber eine Session-Referenz gesetzt,
  // sondern direkt als angezeigter Winkel berechnet:
  // 90 - FiltX + DISPLAY_ANGLE_OFFSET_DEG.
  // Funktion bleibt nur als Kompatibilitaetsplatzhalter erhalten.
  Serial.println("MPU-Session-Referenz: deaktiviert, nutze direkten EZ-Anzeigewinkel.");
}

static bool canMoveElevationUp() {
  if (!mpuOk) return true;
  return currentRelativeAngle() < ELEVATION_MAX_SOFT;
}

static bool canMoveElevationDown() {
  if (!mpuOk) return true;
  return currentRelativeAngle() > ELEVATION_MIN_SOFT;
}

// AUTO-Elevation-Limits:
// AUTO darf Elevationsziele ausserhalb der definierten Arbeitsgrenzen
// nicht anfahren. Die Grenzen beziehen sich auf denselben angezeigten
// Elevationswinkel wie currentRelativeAngle() und enthalten damit auch
// DISPLAY_ANGLE_OFFSET_DEG.
static bool autoElevationTargetWithinSoftLimits(float targetDeg) {
  return targetDeg >= ELEVATION_MIN_SOFT && targetDeg <= ELEVATION_MAX_SOFT;
}

// Misst ein lokales RF-Schwankungsband im aktuellen Stillstand.
// Diese kurze Messung dient dazu, typische Schwankungen ohne echten Peak
// abzuschätzen. Daraus werden die dynamischen Umschaltschwellen gebildet.
static float measureRfNoiseBandV(uint8_t samples, unsigned long delayMs) {
  float minV = 99.0f;
  float maxV = -99.0f;

  if (samples < 2) samples = 2;

  for (uint8_t i = 0; i < samples; i++) {
    rfUpdate();
    const float v = rfGetPinVoltage();
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
    delay(delayMs);
  }

  const float band = maxV - minV;
  return (band < 0.0f) ? 0.0f : band;
}


// Dynamische Schwellen für Kandidaten-/Feinlogik ableiten.
static void updateDynamicCandidateThresholds() {
  autoCandidateEnterThresholdV = fmaxf(AUTO_RF_DYNAMIC_MIN_ENTER_V,
                                       autoNoiseBandVoltage * AUTO_RF_DYNAMIC_ENTER_FACTOR);
  autoCandidateExitThresholdV = fmaxf(AUTO_RF_DYNAMIC_MIN_EXIT_V,
                                      autoNoiseBandVoltage * AUTO_RF_DYNAMIC_EXIT_FACTOR);
}

// =====================================================
// BlockedSatRange-Verwaltung
// =====================================================

static void clearBlockedRanges() {
  for (int i = 0; i < MAX_BLOCKED_SAT_RANGES; i++) {
    blockedRanges[i] = BlockedSatRange{};
  }
  blockedRangeInsertCounter = 0;
}

static int blockedRangeCount() {
  int count = 0;
  for (int i = 0; i < MAX_BLOCKED_SAT_RANGES; i++) {
    if (blockedRanges[i].active) count++;
  }
  return count;
}

static bool isBlockedAzPosition(int azPosSteps) {
  for (int i = 0; i < MAX_BLOCKED_SAT_RANGES; i++) {
    if (!blockedRanges[i].active) continue;
    if (azPosSteps >= blockedRanges[i].azMinSteps &&
        azPosSteps <= blockedRanges[i].azMaxSteps) {
      return true;
    }
  }
  return false;
}

// Liefert Radius und Klasse für einen falschen Kandidaten.
static int classifyBadSatRadius(float improvementV, uint8_t& strengthClass) {
  strengthClass = 1;
  int radius = BADSAT_WEAK_RADIUS_STEPS;

  if (improvementV >= BADSAT_WEAK_MAX_V && improvementV < BADSAT_STRONG_MAX_V) {
    strengthClass = 2;
    radius = BADSAT_STRONG_RADIUS_STEPS;
  }

  // 3 = sehr stark falsch
  // Der dritte Bereich ist bewusst schon vorbereitet. Zum Aktivieren später
  // einfach diese Bedingung einkommentieren.
  // if (improvementV >= BADSAT_VERY_STRONG_MIN_V) {
  //   strengthClass = 3;
  //   radius = BADSAT_VERY_STRONG_RADIUS_STEPS;
  // }

  return radius;
}

static int findOldestBlockedRangeIndex() {
  int oldestIndex = -1;
  uint32_t oldestOrder = 0xFFFFFFFFu;

  for (int i = 0; i < MAX_BLOCKED_SAT_RANGES; i++) {
    if (!blockedRanges[i].active) continue;
    if (blockedRanges[i].createdOrder < oldestOrder) {
      oldestOrder = blockedRanges[i].createdOrder;
      oldestIndex = i;
    }
  }

  return oldestIndex;
}

// Speichert einen falschen Satelliten als Sperrbereich.
// Nahe / überlappende Bereiche werden zusammengeführt.
static void addOrMergeBlockedSatRange(int azCenterSteps,
                                      float improvementV,
                                      float elevationDeg) {
  if (improvementV < BADSAT_MIN_TRIGGER_V) {
    return;
  }

  uint8_t strengthClass = 1;
  const int radius = classifyBadSatRadius(improvementV, strengthClass);
  int newMin = azCenterSteps - radius;
  int newMax = azCenterSteps + radius;

  // 1) Prüfen, ob ein bestehender Bereich nahe genug ist -> zusammenführen
  for (int i = 0; i < MAX_BLOCKED_SAT_RANGES; i++) {
    if (!blockedRanges[i].active) continue;

    const bool overlapsOrNear =
        !(newMax < (blockedRanges[i].azMinSteps - BADSAT_MERGE_GAP_STEPS) ||
          newMin > (blockedRanges[i].azMaxSteps + BADSAT_MERGE_GAP_STEPS));

    if (overlapsOrNear) {
      blockedRanges[i].azMinSteps = min(blockedRanges[i].azMinSteps, newMin);
      blockedRanges[i].azMaxSteps = max(blockedRanges[i].azMaxSteps, newMax);
      blockedRanges[i].azCenterSteps =
          (blockedRanges[i].azMinSteps + blockedRanges[i].azMaxSteps) / 2;
      blockedRanges[i].improvementV = max(blockedRanges[i].improvementV, improvementV);
      blockedRanges[i].strengthClass = max(blockedRanges[i].strengthClass, strengthClass);
      blockedRanges[i].elevationDeg = elevationDeg;
      Serial.println("BADSAT: Bereich zusammengefuehrt.");
      return;
    }
  }

  // 2) Freien Slot suchen
  for (int i = 0; i < MAX_BLOCKED_SAT_RANGES; i++) {
    if (!blockedRanges[i].active) {
      blockedRanges[i].active = true;
      blockedRanges[i].azCenterSteps = azCenterSteps;
      blockedRanges[i].azMinSteps = newMin;
      blockedRanges[i].azMaxSteps = newMax;
      blockedRanges[i].improvementV = improvementV;
      blockedRanges[i].strengthClass = strengthClass;
      blockedRanges[i].elevationDeg = elevationDeg;
      blockedRanges[i].createdOrder = ++blockedRangeInsertCounter;
      Serial.println("BADSAT: Neuer Bereich gespeichert.");
      return;
    }
  }

  // 3) Wenn voll -> ältesten Eintrag überschreiben
  const int oldest = findOldestBlockedRangeIndex();
  if (oldest >= 0) {
    blockedRanges[oldest].active = true;
    blockedRanges[oldest].azCenterSteps = azCenterSteps;
    blockedRanges[oldest].azMinSteps = newMin;
    blockedRanges[oldest].azMaxSteps = newMax;
    blockedRanges[oldest].improvementV = improvementV;
    blockedRanges[oldest].strengthClass = strengthClass;
    blockedRanges[oldest].elevationDeg = elevationDeg;
    blockedRanges[oldest].createdOrder = ++blockedRangeInsertCounter;
    Serial.println("BADSAT: Array voll, aeltesten Eintrag ersetzt.");
  }
}

// Ziel außerhalb eines gesperrten Bereichs bestimmen.
// Ziel:
// - nicht direkt wieder am selben Peak landen
// - Resume klar außerhalb des blockierten Bereichs beginnen
static int computeResumeTargetOutsideBlockedRange(int azCenterSteps) {
  for (int i = 0; i < MAX_BLOCKED_SAT_RANGES; i++) {
    if (!blockedRanges[i].active) continue;

    if (azCenterSteps >= blockedRanges[i].azMinSteps &&
        azCenterSteps <= blockedRanges[i].azMaxSteps) {
      if (azCenterSteps >= 0) {
        return blockedRanges[i].azMaxSteps + BADSAT_RESUME_MARGIN_STEPS;
      }
      return blockedRanges[i].azMinSteps - BADSAT_RESUME_MARGIN_STEPS;
    }
  }

  // Fallback: wenn kein Bereich exakt getroffen wurde, abhängig vom Vorzeichen
  return (azCenterSteps >= 0)
      ? (azCenterSteps + BADSAT_RESUME_MARGIN_STEPS)
      : (azCenterSteps - BADSAT_RESUME_MARGIN_STEPS);
}

// =====================================================
// AUTO Hilfsobjekte
// =====================================================

static void clearStepTask(StepTask& task) {
  task.active = false;
  task.failed = false;
  task.dir = AZ_DIR_NONE;
  task.remainingSteps = 0;
  task.pulseMs = 0;
  task.pulseIssued = false;
  task.settleUntilMs = 0;
}

static void startStepTask(StepTask& task,
                          AzimuthDirection dir,
                          int steps,
                          unsigned long pulseMs) {
  clearStepTask(task);

  if (steps <= 0) {
    return;
  }

  task.active = true;
  task.dir = dir;
  task.remainingSteps = steps;
  task.pulseMs = pulseMs;
}

static TaskResult updateStepTask(StepTask& task) {
  if (!task.active) {
    return task.failed ? TASK_FAILED : TASK_DONE;
  }

  if (task.pulseIssued) {
    if (azimuthIsPulseActive()) {
      return TASK_RUNNING;
    }

    if (task.settleUntilMs == 0) {
      task.settleUntilMs = millis() + AUTO_AZ_SETTLE_MS;
      return TASK_RUNNING;
    }

    if (millis() < task.settleUntilMs) {
      return TASK_RUNNING;
    }

    task.pulseIssued = false;
    task.settleUntilMs = 0;
    task.remainingSteps--;

    // Jede beendete StepTask-Bewegung verändert die globale Positionszahl.
    applyAzStepFromDir(task.dir);

    if (task.remainingSteps <= 0) {
      task.active = false;
      return TASK_DONE;
    }
  }

  if (!canMoveAzimuth(task.dir)) {
    task.active = false;
    task.failed = true;
    return TASK_FAILED;
  }

  issueAzimuthPulse(task.dir, task.pulseMs);
  task.pulseIssued = true;
  return TASK_RUNNING;
}

static void clearScanTask(ScanTask& task) {
  task.active = false;
  task.failed = false;
  task.stage = 0;
  task.sideSteps = 0;
  task.pulseMs = 0;
  task.positiveDir = AZ_DIR_EAST;
  task.pulseIssued = false;
  task.lastPulseDir = AZ_DIR_NONE;
  task.settleUntilMs = 0;
  task.stepsMoved = 0;
  task.stepsInStage = 0;
  task.bestRelativeStep = 0;
  task.startVoltage = 0.0f;
  task.bestVoltage = 99.0f;
}

static void startScanTask(ScanTask& task,
                          int sideSteps,
                          unsigned long pulseMs,
                          AzimuthDirection positiveDir) {
  clearScanTask(task);

  task.active = true;
  task.stage = 1;
  task.sideSteps = sideSteps;
  task.pulseMs = pulseMs;
  task.positiveDir = positiveDir;

  task.startVoltage = rfGetPinVoltage();
  task.bestVoltage = task.startVoltage;
  task.bestRelativeStep = 0;
}

static void updateBestVoltage(ScanTask& task) {
  const float v = rfGetPinVoltage();
  if (v < task.bestVoltage) {
    task.bestVoltage = v;
    task.bestRelativeStep = task.stepsMoved;
  }
}

static TaskResult updateScanTask(ScanTask& task) {
  if (!task.active) {
    return task.failed ? TASK_FAILED : TASK_DONE;
  }

  if (task.pulseIssued) {
    if (azimuthIsPulseActive()) {
      return TASK_RUNNING;
    }

    if (task.settleUntilMs == 0) {
      task.settleUntilMs = millis() + AUTO_AZ_SETTLE_MS;
      return TASK_RUNNING;
    }

    if (millis() < task.settleUntilMs) {
      return TASK_RUNNING;
    }

    task.pulseIssued = false;
    task.settleUntilMs = 0;

    // Globale Positionsfortschreibung und relative Scan-Positionszahl.
    applyAzStepFromDir(task.lastPulseDir);
    if (task.lastPulseDir == task.positiveDir) {
      task.stepsMoved++;
    } else {
      task.stepsMoved--;
    }

    if (task.stage != 3) {
      updateBestVoltage(task);
    }
  }

  if (task.stage == 1) {
    if (task.stepsInStage >= task.sideSteps) {
      task.stage = 2;
      task.stepsInStage = 0;
      return TASK_RUNNING;
    }

    if (!canMoveAzimuth(task.positiveDir)) {
      task.stage = 2;
      task.stepsInStage = 0;
      return TASK_RUNNING;
    }

    issueAzimuthPulse(task.positiveDir, task.pulseMs);
    task.lastPulseDir = task.positiveDir;
    task.pulseIssued = true;
    task.stepsInStage++;
    return TASK_RUNNING;
  }

  if (task.stage == 2) {
    const int targetCount = task.sideSteps * 2;

    if (task.stepsInStage >= targetCount) {
      task.stage = 3;
      return TASK_RUNNING;
    }

    const AzimuthDirection negDir = oppositeDir(task.positiveDir);

    if (!canMoveAzimuth(negDir)) {
      task.stage = 3;
      return TASK_RUNNING;
    }

    issueAzimuthPulse(negDir, task.pulseMs);
    task.lastPulseDir = negDir;
    task.pulseIssued = true;
    task.stepsInStage++;
    return TASK_RUNNING;
  }

  if (task.stage == 3) {
    const int diff = task.bestRelativeStep - task.stepsMoved;

    if (diff == 0) {
      task.active = false;
      return TASK_DONE;
    }

    const AzimuthDirection returnDir =
        (diff > 0) ? task.positiveDir : oppositeDir(task.positiveDir);

    if (!canMoveAzimuth(returnDir)) {
      task.active = false;
      task.failed = true;
      return TASK_FAILED;
    }

    issueAzimuthPulse(returnDir, task.pulseMs);
    task.lastPulseDir = returnDir;
    task.pulseIssued = true;
    return TASK_RUNNING;
  }

  task.active = false;
  task.failed = true;
  return TASK_FAILED;
}

// =====================================================
// Forward-Deklarationen fuer AUTO/Mitte-Kopplung und Hilfsfunktionen
// =====================================================
static void startCenterHomingFromAuto();
static void startCenterHomingFromAlignMode();
static void abortCenterModeToMenu();
static void centerAzStopOnly();
static bool autoBeginAzDrive(AzimuthDirection dir, bool stepped, const char* reason);
static void autoStopAzDrive();
static void autoStartCandidateHold(AutoState resumeState, const char* source);
static bool autoServiceRfAndCandidate(AutoState resumeState);
static void autoServiceAzPosition();
static bool autoReachedLimitForDir(AzimuthDirection dir);

// =====================================================
// AUTO Ablauf
// =====================================================

static void resetAutoState() {
  autoState = AUTO_STATE_INACTIVE;
  autoFailReason = "";
  autoHasPeak = false;
  autoBaseElevationDeg = 0.0f;
  autoElevationIndex = 0;
  autoTargetElevationDeg = 0.0f;
  autoScanBaselineVoltage = 0.0f;
  autoNoiseBandVoltage = 0.0f;
  autoCandidateEnterThresholdV = 0.0f;
  autoCandidateExitThresholdV = 0.0f;
  autoBestVoltage = 99.0f;
  autoCandidateAzSteps = 0;
  autoCandidateElevationDeg = 0.0f;
  autoCandidateImprovementV = 0.0f;
  autoCandidateStrengthClass = 0;
  autoCandidateConfirmCounter = 0;
  autoCandidateNextConfirmAtMs = 0;
  autoResumeTargetAzSteps = 0;
  autoCurrentSearchDirection = AUTO_PRESET_DIRECTION;
  autoResumeAfterCandidateState = AUTO_STATE_INACTIVE;
  autoReturnAfterCenterState = AUTO_STATE_INACTIVE;
  autoDriveDir = AZ_DIR_NONE;
  autoDriveStepped = false;
  autoDriveStartedAtMs = 0;
  autoLastPseudoStepAtMs = 0;
  autoFineStepOutputOn = false;
  autoFineStepPhaseStartedAtMs = 0;
  autoLastAzStepCounter = 0;
  autoLastRfDiagMs = 0;
  autoRfZeroAdc = 0.0f;
  autoRfBestAdc = 4095.0f;
  autoRfCurrentAdc = 0.0f;
  autoRfDropAdc = 0.0f;
  autoCandidateAdc = 0.0f;
  autoCycleIndex = 0;

  signalOptBestAdc = 4095.0f;
  signalOptLastAdc = 4095.0f;
  signalOptStartAdc = 4095.0f;
  signalOptBestAzSteps = 0;
  signalOptBestElevationDeg = 0.0f;
  signalOptStepsInPhase = 0;
  signalOptStepStartedAtMs = 0;
  signalOptLastDiagMs = 0;

  clearModeMultiClick();

  clearStepTask(autoMoveTask);
  clearScanTask(autoScanTask);
}

// V3: Displaymeldungen kurz halten, Serial bleibt technisch.
// Das TFT hat nur wenig Platz; autoFailReason ist deshalb bewusst knapp.
static const char* shortCenterAutoFailText(const char* msg) {
  if (!msg) return "AUSR FEHLER";
  String m = msg;
  m.toUpperCase();

  if (m.indexOf("TIMEOUT") >= 0 && m.indexOf("CENTER") >= 0) return "CENTER FEHLT";
  if (m.indexOf("ENDSensor") >= 0 || m.indexOf("ENDSENSOR") >= 0) return "AZ LIMIT AKTIV";
  if (m.indexOf("BLOCK") >= 0) return "AZ BLOCKIERT";
  if (m.indexOf("BREIT") >= 0) return "CENTER BREIT";
  return "AUSR FEHLER";
}

static void printHallDebugShort() {
  Serial.print(" | HREF C/E/W=");
  Serial.print(azimuthIsCenterDetected() ? 1 : 0);
  Serial.print("/");
  Serial.print(azimuthIsEastLimitDetected() ? 1 : 0);
  Serial.print("/");
  Serial.print(azimuthIsWestLimitDetected() ? 1 : 0);
}

static void failAuto(const char* reason) {
  stopManualActuators();
  autoFailReason = reason;
  autoState = AUTO_STATE_FAILED;
  autoHasPeak = false;
  clearModeMultiClick();
  Serial.print("AUTO FEHLER: ");
  Serial.print(reason);
  printHallDebugShort();
  Serial.print(" | EZ=");
  Serial.print(currentRelativeAngle(), 1);
  Serial.print(" | RF=");
  Serial.println(rfGetPinVoltage(), 3);
}

static void failAutoDetailed(const char* displayReason, const char* serialDetail) {
  stopManualActuators();
  autoFailReason = displayReason ? displayReason : "AUTO FEHLER";
  autoState = AUTO_STATE_FAILED;
  autoHasPeak = false;
  clearModeMultiClick();
  Serial.print("AUTO FEHLER: ");
  Serial.print(autoFailReason);
  if (serialDetail) {
    Serial.print(" | ");
    Serial.print(serialDetail);
  }
  printHallDebugShort();
  Serial.print(" | EZ=");
  Serial.print(currentRelativeAngle(), 1);
  Serial.print(" | RF=");
  Serial.println(rfGetPinVoltage(), 3);
}

static void startAutoSequence() {
  stopManualActuators();
  autoSetupActive = false;
  enterAutoMode();
  resetAutoState();

  autoBaseElevationDeg = currentRelativeAngle();
  autoElevationIndex = 0;
  autoTargetElevationDeg = autoBaseElevationDeg;

  autoState = AUTO_STATE_NEW_CENTER_START;

  Serial.print("SUCHE START V3 | Ablauf=Mitte->Ost->West->Mitte->Hoehe pruefen");
  Serial.print(" | Basis-Hoehe = ");
  Serial.println(autoBaseElevationDeg, 2);
}


static bool azReferenceEndLimitActive() {
  return azimuthIsEastLimitDetected() || azimuthIsWestLimitDetected();
}

static void activateAzLimitWarning(const char* source) {
  stopManualActuators();
  autoSetupActive = false;
  azLimitWarningActive = true;
  enterMainMenuMode();

  Serial.print("AZ WARNUNG: Start ");
  Serial.print(source ? source : "?");
  Serial.print(" blockiert | Ost=");
  Serial.print(azimuthIsEastLimitDetected() ? 1 : 0);
  Serial.print(" West=");
  Serial.println(azimuthIsWestLimitDetected() ? 1 : 0);
}

static void clearAzLimitWarning() {
  if (azLimitWarningActive) {
    azLimitWarningActive = false;
    Serial.println("AZ WARNUNG: quittiert, zurueck ins Hauptmenue.");
  }
}

static void startAutoSetupSelection() {
  if (azReferenceEndLimitActive()) {
    activateAzLimitWarning("AUTO");
    return;
  }

  stopManualActuators();
  azLimitWarningActive = false;
  autoSetupActive = true;
  enterMainMenuMode();
  mainMenuSelection = MENU_ITEM_AUTO;
  Serial.println("SUCHE SETUP V3: PLUS/MINUS korrigiert Hoehe, MODE startet, MODE lang zurueck.");
}

static void cancelAutoSetupSelection() {
  autoSetupActive = false;
  enterMainMenuMode();
  Serial.println("AUTO SETUP: Abbruch, zurueck ins Hauptmenue.");
}

static void toggleAutoFineSearchOption() {
  // V3: Feinsuche AN/AUS wurde ersatzlos aus der Bedienlogik entfernt.
  // Diese Funktion bleibt nur bestehen, damit eventuell alte Web-Aufrufe oder
  // Diagnosepfade nicht ins Leere laufen. Sie aendert bewusst nichts mehr.
  autoFineSearchEnabled = false;
  Serial.println("SUCHE SETUP V3: Feinsuche entfernt - Hoehe bitte mit +/- korrigieren.");
}



static void abortAutoSequence() {
  autoSetupActive = false;

  if (azimuthIsHomingActive()) {
    azimuthAbortHoming();
  } else {
    azimuthStop();
  }

  elevationStop();
  resetAutoState();
}

// Kandidatenverbesserung neu berechnen.
static float currentCandidateImprovement() {
  return autoScanBaselineVoltage - rfGetPinVoltage();
}

// Kandidat ist vorhanden, wenn die Verbesserung oberhalb der dynamischen
// Enter-Schwelle liegt.
static bool candidateLooksStrongEnough() {
  return currentCandidateImprovement() >= autoCandidateEnterThresholdV;
}

// Forward-Deklaration: Wird von Web-/Runtime-Kommandos vor der eigentlichen
// Funktionsdefinition genutzt. Noetig, damit die Arduino/C++-Kompilierung
// die spaeter definierte interne Hilfsfunktion bereits kennt.
static void confirmRightSatellite();

static void handleFalseSatelliteAtCurrentCandidate() {
  // MINUS speichert den aktuellen RF-Kandidaten als falschen Satelliten.
  // Der aufrufende Kandidatenhandler setzt danach direkt den passenden
  // aktuellen Suchzustand aus autoResumeAfterCandidateState.
  addOrMergeBlockedSatRange(autoCandidateAzSteps,
                            autoCandidateImprovementV,
                            autoCandidateElevationDeg);

  autoResumeTargetAzSteps = computeResumeTargetOutsideBlockedRange(autoCandidateAzSteps);

  Serial.print("BADSAT gespeichert | AZ=");
  Serial.print(autoCandidateAzSteps);
  Serial.print(" | ResumeTarget(nur Diagnose)=");
  Serial.println(autoResumeTargetAzSteps);
}

static void autoResetRfTracker() {
  rfUpdate();
  const float adc = rfGetFilteredAdc();

  autoRfCurrentAdc = adc;
  autoRfZeroAdc = adc;
  autoRfBestAdc = adc;
  autoRfDropAdc = 0.0f;
  autoBestVoltage = rfGetPinVoltage();
  autoScanBaselineVoltage = autoBestVoltage;
  autoLastRfDiagMs = 0;

  Serial.print("AUTO RF: dynamische Referenz gestartet | ADC=");
  Serial.println(adc, 1);
}

static void autoUpdateRfTracker() {
  rfUpdate();

  const float adc = rfGetFilteredAdc();
  autoRfCurrentAdc = adc;

  // AD8317/AD8318 am aktuellen Aufbau:
  // hoeherer ADC = schlechteres/kein Signal -> dynamisches Zero.
  // niedrigerer ADC = staerkeres Signal -> bester Kandidat.
  if (autoRfZeroAdc <= 0.0f || adc > autoRfZeroAdc) {
    autoRfZeroAdc = adc;
  }

  if (adc < autoRfBestAdc) {
    autoRfBestAdc = adc;
    autoBestVoltage = rfGetPinVoltage();
  }

  autoRfDropAdc = autoRfZeroAdc - adc;
  if (autoRfDropAdc < 0.0f) autoRfDropAdc = 0.0f;

  if (millis() - autoLastRfDiagMs >= AUTO_RF_DIAG_INTERVAL_MS) {
    autoLastRfDiagMs = millis();
    Serial.print("AUTO RF | ADC=");
    Serial.print(autoRfCurrentAdc, 1);
    Serial.print(" | ZERO_ADC=");
    Serial.print(autoRfZeroAdc, 1);
    Serial.print(" | BEST_ADC=");
    Serial.print(autoRfBestAdc, 1);
    Serial.print(" | DROP=");
    Serial.print(autoRfDropAdc, 1);
    Serial.print(" | AZPOS=");
    Serial.println(azPositionSteps);
  }
}

static bool autoReachedLimitForDir(AzimuthDirection dir) {
  if (dir == AZ_DIR_EAST) return azimuthIsEastLimitDetected();
  if (dir == AZ_DIR_WEST) return azimuthIsWestLimitDetected();
  return false;
}

static bool autoBeginAzDrive(AzimuthDirection dir, bool stepped, const char* reason) {
  autoStopAzDrive();

  autoDriveDir = dir;
  autoDriveStepped = stepped;
  autoDriveStartedAtMs = millis();
  autoLastPseudoStepAtMs = millis();
  autoLastAzStepCounter = azimuthGetStepCounter();
  autoFineStepOutputOn = false;
  autoFineStepPhaseStartedAtMs = millis();

  Serial.print("AUTO AZ START | ");
  Serial.print(reason ? reason : "-");
  Serial.print(" | DIR=");
  Serial.print(dirText(dir));
  Serial.print(" | MODE=");
  Serial.println(stepped ? "FEIN-STEP" : "DURCHLAUF");

  if (stepped) {
    // Die AUTO-Feinsuche nutzt eigene kurze Pulse.
    // Nicht azimuthStartEast/WestStepped() verwenden, denn dessen
    // AZ_STEP_* Werte sind fuer grobe, fast durchlaufende Fahrten eingestellt.
    if (!autoReachedLimitForDir(dir) && azimuthDriveRawLogical(dir)) {
      autoFineStepOutputOn = true;
      autoFineStepPhaseStartedAtMs = millis();
      Serial.print("AUTO FEINSTEP | ON=");
      Serial.print(AUTO_FINE_STEP_ON_MS);
      Serial.print("ms OFF=");
      Serial.print(AUTO_FINE_STEP_OFF_MS);
      Serial.println("ms");
    } else {
      Serial.print("AUTO FEINSTEP: Start blockiert durch Hall/Limit.");
      printHallDebugShort();
      Serial.println();
      autoDriveDir = AZ_DIR_NONE;
      return false;
    }
  } else {
    // Auch Dauerfahrten muessen bestaetigen, dass der Motor
    // wirklich gestartet wurde. Vorher konnte AUTO in einem WAIT-State
    // haengen bleiben, wenn Hall/Limit die Fahrt blockierte.
    if (autoReachedLimitForDir(dir)) {
      Serial.print("AUTO DURCHLAUF: Start blockiert, Limit aktiv in Richtung ");
      Serial.print(dirText(dir));
      printHallDebugShort();
      Serial.println();
      autoDriveDir = AZ_DIR_NONE;
      return false;
    }

    if (!azimuthDriveRawLogical(dir)) {
      Serial.print("AUTO DURCHLAUF: Motorstart fehlgeschlagen in Richtung ");
      Serial.print(dirText(dir));
      printHallDebugShort();
      Serial.println();
      autoDriveDir = AZ_DIR_NONE;
      return false;
    }
  }

  return true;
}

static void autoStopAzDrive() {
  azimuthStop();
  autoDriveDir = AZ_DIR_NONE;
  autoDriveStepped = false;
  autoDriveStartedAtMs = 0;
  autoLastPseudoStepAtMs = 0;
  autoFineStepOutputOn = false;
  autoFineStepPhaseStartedAtMs = 0;
  autoLastAzStepCounter = azimuthGetStepCounter();
}


// =====================================================
// Signaloptimierung nach PLUS-Bestaetigung
// =====================================================
// Kommentarstand: V3
//
// Grundidee:
// Die eigentliche Suche findet nur einen RF-Kandidaten. Der Nutzer entscheidet
// mit PLUS, dass dieser Kandidat der richtige Satellit ist. Erst danach darf
// die Anlage aktiv nach dem lokalen Signalmaximum suchen.
//
// Wichtig zum AD8317/AD8318-Aufbau:
// Ein kleinerer RF-ADC-Wert bedeutet staerkeres Signal. Die Optimierung sucht
// daher ein lokales RF-Minimum, kein numerisches Maximum.
//
// Die Schrittweiten und Schwellen liegen bewusst in settings.cpp:
// - SIGNAL_OPT_AZ_STEP_MS
// - SIGNAL_OPT_EL_STEP_MS
// - SIGNAL_OPT_RF_IMPROVE_ADC
// - SIGNAL_OPT_RF_WORSE_ADC
// So koennen die Werte nach Aussentests angepasst werden, ohne die Logik zu
// suchen oder NVS/Preferences zu verwenden.
static void signalOptResetFromCurrent() {
  autoStopAzDrive();
  elevationStop();
  rfUpdate();

  const float adc = rfGetFilteredAdc();
  signalOptStartAdc = adc;
  signalOptLastAdc = adc;
  signalOptBestAdc = adc;
  signalOptBestAzSteps = azPositionSteps;
  signalOptBestElevationDeg = currentRelativeAngle();
  signalOptStepsInPhase = 0;
  signalOptStepStartedAtMs = 0;
  signalOptLastDiagMs = 0;

  Serial.print("SIGNAL OPT V3: Start | ADC=");
  Serial.print(adc, 1);
  Serial.print(" | AZ=");
  Serial.print(signalOptBestAzSteps);
  Serial.print(" | Winkel=");
  Serial.println(signalOptBestElevationDeg, 2);
}

static void signalOptUpdateBest(float adc) {
  if (adc <= signalOptBestAdc - SIGNAL_OPT_RF_IMPROVE_ADC || adc < signalOptBestAdc) {
    signalOptBestAdc = adc;
    signalOptBestAzSteps = azPositionSteps;
    signalOptBestElevationDeg = currentRelativeAngle();
  }
  signalOptLastAdc = adc;

  if (millis() - signalOptLastDiagMs >= AUTO_RF_DIAG_INTERVAL_MS) {
    signalOptLastDiagMs = millis();
    Serial.print("SIGNAL OPT V3 | ADC=");
    Serial.print(adc, 1);
    Serial.print(" | BEST=");
    Serial.print(signalOptBestAdc, 1);
    Serial.print(" | AZ=");
    Serial.print(azPositionSteps);
    Serial.print(" | Winkel=");
    Serial.println(currentRelativeAngle(), 2);
  }
}

static bool signalOptIsClearlyWorse(float adc) {
  return adc >= signalOptBestAdc + SIGNAL_OPT_RF_WORSE_ADC;
}

// V3: Prueft die echte logische AZ-Grenze fuer die aktuelle
// Optimierungsrichtung. Fuer die Signaloptimierung sind die Ost-/West-
// Hallsensoren die primaere Begrenzung. Die Schrittanzahl bleibt nur als
// Sicherheitsnetz, falls ein Sensor nicht ausloest oder ein Testlauf aus
// anderen Gruenden zu lange in einer Richtung weiterlaufen wuerde.
static bool signalOptAzLimitReached(AzimuthDirection dir) {
  if (dir == AZ_DIR_EAST) return azimuthIsEastLimitDetected();
  if (dir == AZ_DIR_WEST) return azimuthIsWestLimitDetected();
  return true;
}

static const char* signalOptAzLimitText(AzimuthDirection dir) {
  if (dir == AZ_DIR_EAST) return "OST-SENSOR";
  if (dir == AZ_DIR_WEST) return "WEST-SENSOR";
  return "UNBEKANNT";
}

static bool signalOptIssueAzStep(AzimuthDirection dir) {
  // V3: Primaere Grenze der AZ-Optimierung ist der jeweilige
  // Ost-/West-Hallsensor. Dadurch richtet sich die Optimierung nach der
  // realen Mechanik und nicht nur nach einer theoretischen Zeit oder
  // Schrittanzahl.
  if (signalOptAzLimitReached(dir)) {
    Serial.print("SIGNAL OPT V3: AZ Grenze erreicht | ");
    Serial.print(signalOptAzLimitText(dir));
    Serial.print(" | DIR=");
    Serial.println(dirText(dir));
    return false;
  }

  // V3: Die maximale Schrittzahl bleibt bewusst nur als zweite
  // Sicherheitsgrenze. Normalerweise beendet der Hall-Sensor die Richtung.
  if (signalOptStepsInPhase >= (uint8_t)max(0, SIGNAL_OPT_AZ_MAX_STEPS_PER_DIR)) {
    Serial.print("SIGNAL OPT V3: AZ Sicherheitslimit Schritte erreicht | DIR=");
    Serial.println(dirText(dir));
    return false;
  }

  if (!canMoveAzimuth(dir)) {
    Serial.print("SIGNAL OPT V3: AZ Schritt blockiert | DIR=");
    Serial.println(dirText(dir));
    return false;
  }

  issueAzimuthPulse(dir, SIGNAL_OPT_AZ_STEP_MS);
  applyAzStepFromDir(dir);
  signalOptStepsInPhase++;
  signalOptStepStartedAtMs = millis();
  return true;
}

static bool signalOptIssueElevationStep(ElevationDirection dir) {
  if (signalOptStepsInPhase >= (uint8_t)max(0, SIGNAL_OPT_EL_MAX_STEPS_PER_DIR)) {
    return false;
  }

  if (dir == EL_DIR_UP) {
    if (!canMoveElevationUp()) return false;
    elevationPulseUp(SIGNAL_OPT_EL_STEP_MS, WEB_EL_PWM);
    manualElDirection = EL_DIR_UP;
  } else if (dir == EL_DIR_DOWN) {
    if (!canMoveElevationDown()) return false;
    elevationPulseDown(SIGNAL_OPT_EL_STEP_MS, WEB_EL_PWM);
    manualElDirection = EL_DIR_DOWN;
  } else {
    return false;
  }

  signalOptStepsInPhase++;
  signalOptStepStartedAtMs = millis();
  return true;
}

static void signalOptStartAzEastPhase() {
  signalOptStepsInPhase = 0;
  signalOptStepStartedAtMs = 0;
  autoState = AUTO_STATE_SIGNAL_OPT_AZ_EAST_START;
  Serial.println("SIGNAL OPT V3: Phase AZ Richtung Osten.");
}

static void signalOptStartAzWestPhase() {
  signalOptStepsInPhase = 0;
  signalOptStepStartedAtMs = 0;
  autoState = AUTO_STATE_SIGNAL_OPT_AZ_WEST_START;
  Serial.println("SIGNAL OPT V3: Phase AZ Richtung Westen.");
}

static void signalOptStartElUpPhase() {
  signalOptStepsInPhase = 0;
  signalOptStepStartedAtMs = 0;
  autoState = AUTO_STATE_SIGNAL_OPT_EL_UP_START;
  Serial.println("SIGNAL OPT V3: Phase Winkel hoch.");
}

static void signalOptStartElDownPhase() {
  signalOptStepsInPhase = 0;
  signalOptStepStartedAtMs = 0;
  autoState = AUTO_STATE_SIGNAL_OPT_EL_DOWN_START;
  Serial.println("SIGNAL OPT V3: Phase Winkel runter.");
}

static void signalOptFinish() {
  autoStopAzDrive();
  elevationStop();
  manualElDirection = EL_DIR_STOP;
  autoHasPeak = true;
  autoState = AUTO_STATE_COMPLETE;

  Serial.print("SIGNAL OPT V3: Fertig | BEST_ADC=");
  Serial.print(signalOptBestAdc, 1);
  Serial.print(" | BEST_AZ=");
  Serial.print(signalOptBestAzSteps);
  Serial.print(" | BEST_WINKEL=");
  Serial.println(signalOptBestElevationDeg, 2);
}

static void signalOptStartFromCandidate() {
  signalOptResetFromCurrent();
  signalOptStartAzEastPhase();
}

static void autoServiceAzPosition() {
  if (autoDriveDir == AZ_DIR_NONE) return;

  if (autoDriveStepped) {
    const unsigned long now = millis();

    if (autoFineStepOutputOn) {
      if (now - autoFineStepPhaseStartedAtMs >= AUTO_FINE_STEP_ON_MS) {
        azimuthStop();
        autoFineStepOutputOn = false;
        autoFineStepPhaseStartedAtMs = now;
        applyAzStepFromDir(autoDriveDir);
      }
      return;
    }

    if (now - autoFineStepPhaseStartedAtMs >= AUTO_FINE_STEP_OFF_MS) {
      if (autoReachedLimitForDir(autoDriveDir)) {
        return;
      }

      if (azimuthDriveRawLogical(autoDriveDir)) {
        autoFineStepOutputOn = true;
        autoFineStepPhaseStartedAtMs = now;
      }
    }
    return;
  }

  const unsigned long now = millis();
  while (now - autoLastPseudoStepAtMs >= AUTO_CONTINUOUS_PSEUDO_STEP_MS) {
    autoLastPseudoStepAtMs += AUTO_CONTINUOUS_PSEUDO_STEP_MS;
    applyAzStepFromDir(autoDriveDir);
  }
}

static void autoStartCandidateHold(AutoState resumeState, const char* source) {
  autoStopAzDrive();

  autoCandidateAdc = autoRfCurrentAdc;
  autoCandidateImprovementV = autoRfDropAdc;  // bewusst ADC-Drop, nicht Volt.
  autoCandidateAzSteps = azPositionSteps;
  autoCandidateElevationDeg = currentRelativeAngle();
  autoResumeAfterCandidateState = resumeState;
  // V3: Ein Kandidat ist noch keine bestaetigte erfolgreiche Suche.
  // autoHasPeak bleibt deshalb false, bis der Nutzer den Satelliten mit
  // PLUS/OK wirklich bestaetigt. Dadurch kann ein mit MINUS verworfener
  // Kandidat die Web-UI nicht faelschlich als "SAT OK" markieren.
  autoHasPeak = false;
  autoState = AUTO_STATE_CANDIDATE_HOLD;
  clearModeMultiClick();

  Serial.print("AUTO KANDIDAT GEFUNDEN | Quelle=");
  Serial.print(source ? source : "-");
  Serial.print(" | ADC=");
  Serial.print(autoCandidateAdc, 1);
  Serial.print(" | DROP_ADC=");
  Serial.print(autoCandidateImprovementV, 1);
  Serial.print(" | AZPOS=");
  Serial.print(autoCandidateAzSteps);
  Serial.print(" | EZ=");
  Serial.println(autoCandidateElevationDeg, 2);
  Serial.println("AUTO KANDIDAT: PLUS=richtig | MINUS=falsch/weiter | MODE lang=Abbruch");
}

static bool autoServiceRfAndCandidate(AutoState resumeState) {
  autoUpdateRfTracker();

  // Gesperrte falsche Satellitenbereiche werden beim erneuten Durchfahren
  // ignoriert. Dadurch kann MINUS einen Kandidatenbereich wirklich ueberspringen.
  if (isBlockedAzPosition(azPositionSteps)) {
    return false;
  }

  if (autoRfDropAdc >= AUTO_RF_CANDIDATE_DROP_ADC) {
    autoStartCandidateHold(resumeState, "RF_DROP");
    return true;
  }

  return false;
}

static void autoStartCenterForNextState(AutoState nextState) {
  autoReturnAfterCenterState = nextState;
  autoState = AUTO_STATE_NEW_CENTER_START;
}

static void updateAutoStrategy() {
  if (!isAutoMode()) {
    return;
  }

  switch (autoState) {
    case AUTO_STATE_INACTIVE:
      return;

    // -------------------------------------------------
    // Schritt 1: getestete Mittenfunktion als Referenz
    // -------------------------------------------------
    case AUTO_STATE_NEW_CENTER_START:
      Serial.println("SUCHE V3: Starte getestete Mittenfunktion als Referenz.");
      autoStopAzDrive();
      // V3: Nach erfolgreicher Mitte wird direkt Richtung Osten gesucht.
      // Die alte Zwischenfahrt zur 'Astra-Startposition Ost' wird uebersprungen,
      // weil der neue Ablauf bewusst einfach ist: Mitte -> Ost -> West -> Mitte.
      autoReturnAfterCenterState = AUTO_STATE_NEW_SCAN_EAST_START;
      startCenterHomingFromAuto();
      autoState = AUTO_STATE_NEW_CENTER_WAIT;
      return;

    case AUTO_STATE_NEW_CENTER_WAIT:
      // updateCenterMode() fuehrt die eigentliche Mitten-Zeitmessung aus.
      // Der erfolgreiche Abschluss setzt autoState auf autoReturnAfterCenterState.
      return;

    // -------------------------------------------------
    // Schritt 2: von Mitte nach Osten zur Astra-Startposition
    // RF wird bereits auf dieser Strecke beobachtet.
    // -------------------------------------------------
    case AUTO_STATE_NEW_MOVE_ASTRA_EAST_START:
      azPositionSteps = 0;
      autoResetRfTracker();
      if (!autoBeginAzDrive(AZ_DIR_EAST, false, "Mitte -> Astra Startposition Ost")) {
        failAutoDetailed("AZ OST BLOCK", "Astra-Start Ost blockiert");
        return;
      }
      autoState = AUTO_STATE_NEW_MOVE_ASTRA_EAST_WAIT;
      return;

    case AUTO_STATE_NEW_MOVE_ASTRA_EAST_WAIT:
      autoServiceAzPosition();
      if (autoServiceRfAndCandidate(AUTO_STATE_NEW_MOVE_ASTRA_EAST_START)) return;

      if (autoReachedLimitForDir(AZ_DIR_EAST)) {
        autoStopAzDrive();
        Serial.println("AUTO V3: Ost-Hall vor/bei Astra-Strecke erreicht, wechsle zu West-Suche.");
        autoState = AUTO_STATE_NEW_SCAN_WEST_START;
        return;
      }

      if (azPositionSteps >= AUTO_ASTRA_OFFSET_STEPS) {
        autoStopAzDrive();
        Serial.println("AUTO V3: Astra-Startposition erreicht, suche weiter nach Osten.");
        autoState = AUTO_STATE_NEW_SCAN_EAST_START;
        return;
      }
      return;

    // -------------------------------------------------
    // Schritt 3: weiter Richtung Osten bis Ost-Hall
    // -------------------------------------------------
    case AUTO_STATE_NEW_SCAN_EAST_START:
      if (!autoBeginAzDrive(AZ_DIR_EAST, false, "Ost-Suche bis Ost-Hall")) {
        failAutoDetailed("GROB OST BLOCK", "Grobsuche Ost blockiert");
        return;
      }
      autoState = AUTO_STATE_NEW_SCAN_EAST_WAIT;
      return;

    case AUTO_STATE_NEW_SCAN_EAST_WAIT:
      autoServiceAzPosition();
      if (autoServiceRfAndCandidate(AUTO_STATE_NEW_SCAN_EAST_START)) return;

      if (autoReachedLimitForDir(AZ_DIR_EAST)) {
        autoStopAzDrive();
        Serial.println("AUTO V3: Ost-Hall erreicht, Richtungswechsel nach Westen.");
        autoState = AUTO_STATE_NEW_SCAN_WEST_START;
        return;
      }
      return;

    // -------------------------------------------------
    // Schritt 4: von Ost-Hall nach West-Hall durchfahren
    // -------------------------------------------------
    case AUTO_STATE_NEW_SCAN_WEST_START:
      if (!autoBeginAzDrive(AZ_DIR_WEST, false, "West-Suche bis West-Hall")) {
        failAutoDetailed("GROB WEST BLOCK", "Grobsuche West blockiert");
        return;
      }
      autoState = AUTO_STATE_NEW_SCAN_WEST_WAIT;
      return;

    case AUTO_STATE_NEW_SCAN_WEST_WAIT:
      autoServiceAzPosition();
      if (autoServiceRfAndCandidate(AUTO_STATE_NEW_SCAN_WEST_START)) return;

      if (autoReachedLimitForDir(AZ_DIR_WEST)) {
        autoStopAzDrive();
        Serial.println("SUCHE V3: West-Limit erreicht, Rueckfahrt zur Mitte.");
        autoState = AUTO_STATE_NEW_RETURN_CENTER_START;
        return;
      }
      return;

    // -------------------------------------------------
    // Schritt 5 ALT: Feinsuche wurde entfernt
    // -------------------------------------------------
    case AUTO_STATE_NEW_STEP_EAST_START:
    case AUTO_STATE_NEW_STEP_EAST_WAIT:
      // V3: Diese Zustaende duerfen im neuen Suchablauf nicht mehr aktiv
      // angesprungen werden. Falls doch, wird sicher gestoppt und zur
      // Rueckfahrt in die Mitte gewechselt, statt eine alte Feinsuche zu fahren.
      autoStopAzDrive();
      Serial.println("SUCHE V3: alter Feinsuche-Zustand erkannt, Rueckfahrt zur Mitte.");
      autoState = AUTO_STATE_NEW_RETURN_CENTER_START;
      return;

    // -------------------------------------------------
    // Schritt 6: aus der West-Position zurueck zur Mitte
    // -------------------------------------------------
    case AUTO_STATE_NEW_RETURN_CENTER_START:
      // V3: Nach der West-Suche steht die Anlage am West-Limit.
      // Zur Mitte muss deshalb in Gegenrichtung, also logisch nach OSTEN,
      // gefahren werden. RF wird dabei weiter beobachtet, aber die Fahrt soll
      // bewusst in der Mitte enden, damit der Nutzer dort die Hoehe pruefen kann.
      if (!autoBeginAzDrive(AZ_DIR_EAST, false, "Rueckfahrt West -> Mitte")) {
        failAutoDetailed("RUECK MITTE BLOCK", "Rueckfahrt zur Mitte Richtung Ost blockiert");
        return;
      }
      autoState = AUTO_STATE_NEW_RETURN_CENTER_WAIT;
      return;

    case AUTO_STATE_NEW_RETURN_CENTER_WAIT:
      autoServiceAzPosition();
      autoUpdateRfTracker();  // V3: RF auf dem Rueckweg weiter beobachten, aber nicht vor Mitte stoppen.

      if (azimuthIsCenterDetected()) {
        autoStopAzDrive();
        azPositionSteps = 0;
        Serial.println("SUCHE V3: Mitte wieder erreicht, Hoehe manuell pruefen.");
        autoState = AUTO_STATE_EZ_ADJUST_HINT;
        return;
      }

      if (millis() - autoDriveStartedAtMs >= AUTO_RETURN_CENTER_TIMEOUT_MS) {
        autoStopAzDrive();
        failAutoDetailed("CENTER FEHLT", "Rueckfahrt Mitte Timeout 60s");
        return;
      }
      return;

    // -------------------------------------------------
    // Schritt 7 ALT: automatisches Hoehenraster entfernt
    // -------------------------------------------------
    case AUTO_STATE_NEW_CHANGE_ELEVATION_START:
    case AUTO_STATE_NEW_CHANGE_ELEVATION_WAIT:
      // V3: Das alte automatische Elevationsraster wurde entfernt, weil die
      // Aussentests gezeigt haben, dass die Hoehe nur grob passen muss und
      // der Nutzer die Hoehe nach dem Suchlauf besser manuell bewerten kann.
      elevationStop();
      autoState = AUTO_STATE_EZ_ADJUST_HINT;
      Serial.println("SUCHE V3: alter Hoehenraster-Zustand erkannt, Wechsel zu Hoehe pruefen.");
      return;

    case AUTO_STATE_EZ_ADJUST_HINT:
      // Endzustand des neuen Suchdurchlaufs: Anlage steht in der Mitte.
      // Der Nutzer kann jetzt die Hoehe mit +/- korrigieren und danach den
      // kompletten Suchdurchlauf erneut starten.
      stopManualActuators();
      return;

    // -------------------------------------------------
    // Kandidat / Erfolg / Fehler
    // -------------------------------------------------
    case AUTO_STATE_CANDIDATE_HOLD:
      stopManualActuators();
      return;

    // -------------------------------------------------
    // Signaloptimierung nach PLUS-Bestaetigung
    // -------------------------------------------------
    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_START:
      if (!signalOptIssueAzStep(AZ_DIR_EAST)) {
        signalOptStartAzWestPhase();
        return;
      }
      autoState = AUTO_STATE_SIGNAL_OPT_AZ_EAST_WAIT;
      return;

    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_WAIT:
      if (azimuthIsPulseActive()) return;
      if (millis() - signalOptStepStartedAtMs < SIGNAL_OPT_AZ_SETTLE_MS) return;
      rfUpdate();
      signalOptUpdateBest(rfGetFilteredAdc());
      // V3: Nach jedem AZ-Schritt erneut den Ost-Hallsensor pruefen.
      // Wenn die echte mechanische Ost-Grenze erreicht ist, wird diese
      // Richtung beendet und nicht weiter ueber Zeit/Schritte erzwungen.
      if (signalOptAzLimitReached(AZ_DIR_EAST)) {
        signalOptStartAzWestPhase();
        return;
      }
      if (signalOptIsClearlyWorse(rfGetFilteredAdc())) {
        signalOptStartAzWestPhase();
        return;
      }
      autoState = AUTO_STATE_SIGNAL_OPT_AZ_EAST_START;
      return;

    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_START:
      if (!signalOptIssueAzStep(AZ_DIR_WEST)) {
        signalOptStartElUpPhase();
        return;
      }
      autoState = AUTO_STATE_SIGNAL_OPT_AZ_WEST_WAIT;
      return;

    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_WAIT:
      if (azimuthIsPulseActive()) return;
      if (millis() - signalOptStepStartedAtMs < SIGNAL_OPT_AZ_SETTLE_MS) return;
      rfUpdate();
      signalOptUpdateBest(rfGetFilteredAdc());
      // V3: Nach jedem AZ-Schritt erneut den West-Hallsensor pruefen.
      // Wenn die echte mechanische West-Grenze erreicht ist, wechselt die
      // Signaloptimierung sauber zur Winkel-Optimierung.
      if (signalOptAzLimitReached(AZ_DIR_WEST)) {
        signalOptStartElUpPhase();
        return;
      }
      if (signalOptIsClearlyWorse(rfGetFilteredAdc())) {
        signalOptStartElUpPhase();
        return;
      }
      autoState = AUTO_STATE_SIGNAL_OPT_AZ_WEST_START;
      return;

    case AUTO_STATE_SIGNAL_OPT_EL_UP_START:
      if (!signalOptIssueElevationStep(EL_DIR_UP)) {
        signalOptStartElDownPhase();
        return;
      }
      autoState = AUTO_STATE_SIGNAL_OPT_EL_UP_WAIT;
      return;

    case AUTO_STATE_SIGNAL_OPT_EL_UP_WAIT:
      if (elevationIsPulseActive()) return;
      if (millis() - signalOptStepStartedAtMs < SIGNAL_OPT_EL_SETTLE_MS) return;
      manualElDirection = EL_DIR_STOP;
      rfUpdate();
      signalOptUpdateBest(rfGetFilteredAdc());
      if (signalOptIsClearlyWorse(rfGetFilteredAdc())) {
        signalOptStartElDownPhase();
        return;
      }
      autoState = AUTO_STATE_SIGNAL_OPT_EL_UP_START;
      return;

    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_START:
      if (!signalOptIssueElevationStep(EL_DIR_DOWN)) {
        signalOptFinish();
        return;
      }
      autoState = AUTO_STATE_SIGNAL_OPT_EL_DOWN_WAIT;
      return;

    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_WAIT:
      if (elevationIsPulseActive()) return;
      if (millis() - signalOptStepStartedAtMs < SIGNAL_OPT_EL_SETTLE_MS) return;
      manualElDirection = EL_DIR_STOP;
      rfUpdate();
      signalOptUpdateBest(rfGetFilteredAdc());
      if (signalOptIsClearlyWorse(rfGetFilteredAdc())) {
        signalOptFinish();
        return;
      }
      autoState = AUTO_STATE_SIGNAL_OPT_EL_DOWN_START;
      return;

    case AUTO_STATE_COMPLETE:
      return;

    case AUTO_STATE_FAILED:
      return;

    default:
      // Alte AUTO-Zustaende werden im aktuellen Ablauf bewusst nicht mehr benutzt.
      failAutoDetailed("AUTO CODE FEHLER", "Alter AUTO-Zustand in V3 nicht erlaubt");
      return;
  }
}

// =====================================================
// Texte fuer Web / Display
// =====================================================

static const char* autoStateText() {
  switch (autoState) {
    case AUTO_STATE_INACTIVE:                return "INAKTIV";
    case AUTO_STATE_CANDIDATE_HOLD:          return "KANDIDAT";
    case AUTO_STATE_NEW_CENTER_START:        return "MITTE START";
    case AUTO_STATE_NEW_CENTER_WAIT:         return "MITTENFAHRT";
    case AUTO_STATE_NEW_MOVE_ASTRA_EAST_START:return "ASTRA OST START";
    case AUTO_STATE_NEW_MOVE_ASTRA_EAST_WAIT:return "ASTRA OST";
    case AUTO_STATE_NEW_SCAN_EAST_START:     return "GROB OST START";
    case AUTO_STATE_NEW_SCAN_EAST_WAIT:      return "GROBSUCHE OST";
    case AUTO_STATE_NEW_SCAN_WEST_START:     return "GROB WEST START";
    case AUTO_STATE_NEW_SCAN_WEST_WAIT:      return "GROBSUCHE WEST";
    case AUTO_STATE_NEW_STEP_EAST_START:     return "FEIN ENTFERNT";
    case AUTO_STATE_NEW_STEP_EAST_WAIT:      return "FEIN ENTFERNT";
    case AUTO_STATE_NEW_RETURN_CENTER_START: return "RUECK MITTE START";
    case AUTO_STATE_NEW_RETURN_CENTER_WAIT:  return "RUECKFAHRT MITTE";
    case AUTO_STATE_NEW_CHANGE_ELEVATION_START:return "HOEHE ALT";
    case AUTO_STATE_NEW_CHANGE_ELEVATION_WAIT:return "HOEHE ALT";
    case AUTO_STATE_EZ_ADJUST_HINT:          return "HOEHE PRUEFEN";
    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_START:return "OPT AZ OST";
    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_WAIT: return "OPT AZ OST";
    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_START:return "OPT AZ WEST";
    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_WAIT: return "OPT AZ WEST";
    case AUTO_STATE_SIGNAL_OPT_EL_UP_START:  return "OPT WINKEL +";
    case AUTO_STATE_SIGNAL_OPT_EL_UP_WAIT:   return "OPT WINKEL +";
    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_START:return "OPT WINKEL -";
    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_WAIT: return "OPT WINKEL -";
    case AUTO_STATE_COMPLETE:                return "SAT OPTIMIERT";
    case AUTO_STATE_FAILED:                  return "FAILED";
    default:                                 return "UNKNOWN";
  }
}

static const char* autoInfoText() {
  switch (autoState) {
    case AUTO_STATE_INACTIVE:                return "INFO: AUTO inaktiv";
    case AUTO_STATE_CANDIDATE_HOLD:          return "SAT: +OK -FALSCH";
    case AUTO_STATE_NEW_CENTER_START:
    case AUTO_STATE_NEW_CENTER_WAIT:         return "AUTO: MITTENFAHRT";
    case AUTO_STATE_NEW_MOVE_ASTRA_EAST_START:
    case AUTO_STATE_NEW_MOVE_ASTRA_EAST_WAIT:return "AUTO: ASTRA START OST";
    case AUTO_STATE_NEW_SCAN_EAST_START:
    case AUTO_STATE_NEW_SCAN_EAST_WAIT:      return "AUTO: GROBSUCHE OST";
    case AUTO_STATE_NEW_SCAN_WEST_START:
    case AUTO_STATE_NEW_SCAN_WEST_WAIT:      return "AUTO: GROBSUCHE WEST";
    case AUTO_STATE_NEW_STEP_EAST_START:
    case AUTO_STATE_NEW_STEP_EAST_WAIT:      return "SUCHE: FEINSUCHE ENTFERNT";
    case AUTO_STATE_NEW_RETURN_CENTER_START:
    case AUTO_STATE_NEW_RETURN_CENTER_WAIT:  return "SUCHE: RUECKFAHRT MITTE";
    case AUTO_STATE_NEW_CHANGE_ELEVATION_START:
    case AUTO_STATE_NEW_CHANGE_ELEVATION_WAIT:return "SUCHE: HOEHE ALT";
    case AUTO_STATE_EZ_ADJUST_HINT:          return "MITTE: HOEHE PRUEFEN";
    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_START:
    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_WAIT: return "SIGNAL: AZ OST OPTIMIEREN";
    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_START:
    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_WAIT: return "SIGNAL: AZ WEST OPTIMIEREN";
    case AUTO_STATE_SIGNAL_OPT_EL_UP_START:
    case AUTO_STATE_SIGNAL_OPT_EL_UP_WAIT:   return "SIGNAL: WINKEL PLUS TEST";
    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_START:
    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_WAIT: return "SIGNAL: WINKEL MINUS TEST";
    case AUTO_STATE_COMPLETE:                return "SIGNAL: OPTIMUM GEFUNDEN";
    case AUTO_STATE_FAILED:                  return autoFailReason;
    default:                                 return "INFO: ---";
  }
}

// =====================================================
// Public Status
// =====================================================

bool liveMpuReady() {
  return initOk && mpuOk;
}

float liveGetRelativeAngleDeg() {
  return currentRelativeAngle();
}

float liveGetFilteredAngleDeg() {
  return mpuGetFilteredAngleDeg();
}

const char* liveGetModeText() {
  if (isMainMenuMode() && autoSetupActive) return "AUTO SETUP";
  if (isMainMenuMode()) return "HAUPTMENUE";
  if (isCenterMode()) return "MITTE";
  return isAutoMode() ? "AUTO" : "MANUELL";
}

const char* liveGetAxisText() {
  if (isMainMenuMode()) return "-";
  if (isCenterMode()) return "MITTE";
  return (manualAxis == MANUAL_AXIS_AZ) ? "AZIMUT" : "ELEVATION";
}

const char* liveGetAutoStateText() {
  return autoStateText();
}

const char* liveGetAzimuthStateText() {
  if (isMainMenuMode() && autoSetupActive) {
    return "Setup Hoehe";
  }

  if (isAutoMode()) {
    switch (autoState) {
      case AUTO_STATE_CANDIDATE_HOLD:          return "kandidat";
      case AUTO_STATE_NEW_CENTER_START:
      case AUTO_STATE_NEW_CENTER_WAIT:         return "Mittenfahrt";
      case AUTO_STATE_NEW_MOVE_ASTRA_EAST_START:
      case AUTO_STATE_NEW_MOVE_ASTRA_EAST_WAIT:return "Astra Ost";
      case AUTO_STATE_NEW_SCAN_EAST_START:
      case AUTO_STATE_NEW_SCAN_EAST_WAIT:      return "Grob Ost";
      case AUTO_STATE_NEW_SCAN_WEST_START:
      case AUTO_STATE_NEW_SCAN_WEST_WAIT:      return "Grob West";
      case AUTO_STATE_NEW_STEP_EAST_START:
      case AUTO_STATE_NEW_STEP_EAST_WAIT:      return "Fein entfernt";
      case AUTO_STATE_NEW_RETURN_CENTER_START:
      case AUTO_STATE_NEW_RETURN_CENTER_WAIT:  return "Rueck Mitte";
      case AUTO_STATE_NEW_CHANGE_ELEVATION_START:
      case AUTO_STATE_NEW_CHANGE_ELEVATION_WAIT:return "Hoehe alt";
      case AUTO_STATE_EZ_ADJUST_HINT:          return "Hoehe pruefen";
      case AUTO_STATE_SIGNAL_OPT_AZ_EAST_START:
      case AUTO_STATE_SIGNAL_OPT_AZ_EAST_WAIT: return "Signal opt. Ost";
      case AUTO_STATE_SIGNAL_OPT_AZ_WEST_START:
      case AUTO_STATE_SIGNAL_OPT_AZ_WEST_WAIT: return "Signal opt. West";
      case AUTO_STATE_SIGNAL_OPT_EL_UP_START:
      case AUTO_STATE_SIGNAL_OPT_EL_UP_WAIT:   return "Signal opt. Winkel +";
      case AUTO_STATE_SIGNAL_OPT_EL_DOWN_START:
      case AUTO_STATE_SIGNAL_OPT_EL_DOWN_WAIT: return "Signal opt. Winkel -";
      case AUTO_STATE_COMPLETE:                return "optimiert";
      case AUTO_STATE_FAILED:                  return "fehler";
      default:                                 return "wartet";
    }
  }

  if (isCenterMode()) {
    if (centerHomingStarted) {
      switch (centerTimingState) {
        case CENTER_TIMING_LEAVE_ACTIVE:  return "Mitte verlassen";
        case CENTER_TIMING_SEARCH_ENTER:  return "Mitte suchen";
        case CENTER_TIMING_CROSS_EXIT:    return "Mitte messen";
        case CENTER_TIMING_RETURN_HALF:   return "Mitte zentrieren";
        case CENTER_TIMING_FAILED:        return "Mitte Fehler";
        case CENTER_TIMING_DONE:          return "Mitte fertig";
        default:                          return "Mitte aktiv";
      }
    }
    if (azimuthIsCenterDetected()) return "Mitte";
    return "bereit";
  }

  return azimuthIsPulseActive() ? "bewegt sich" : "wartet";
}

const char* liveGetElevationStateText() {
  if (!mpuOk) return "MPU fehlt";

  const float rel = currentRelativeAngle();

  if (rel <= ELEVATION_MIN_SOFT) return "MIN LIMIT";
  if (rel >= ELEVATION_MAX_SOFT) return "MAX LIMIT";

  if (isAutoMode()) {
    switch (autoState) {
      case AUTO_STATE_NEW_CHANGE_ELEVATION_START:
      case AUTO_STATE_NEW_CHANGE_ELEVATION_WAIT:return "hoehe alt";
      case AUTO_STATE_EZ_ADJUST_HINT:          return "hoehe pruefen";
      case AUTO_STATE_SIGNAL_OPT_AZ_EAST_START:
      case AUTO_STATE_SIGNAL_OPT_AZ_EAST_WAIT:
      case AUTO_STATE_SIGNAL_OPT_AZ_WEST_START:
      case AUTO_STATE_SIGNAL_OPT_AZ_WEST_WAIT: return "az optimiert";
      case AUTO_STATE_SIGNAL_OPT_EL_UP_START:
      case AUTO_STATE_SIGNAL_OPT_EL_UP_WAIT:   return "winkel hoch";
      case AUTO_STATE_SIGNAL_OPT_EL_DOWN_START:
      case AUTO_STATE_SIGNAL_OPT_EL_DOWN_WAIT: return "winkel runter";
      case AUTO_STATE_COMPLETE:                return "signal optimiert";
      case AUTO_STATE_FAILED:                  return "auto fehler";
      default:                                 return "auto aktiv";
    }
  }

  switch (manualElDirection) {
    case EL_DIR_UP:   return "faehrt hoch";
    case EL_DIR_DOWN: return "faehrt runter";
    default:          return "im Bereich";
  }
}

const char* liveGetInfoText() {
  static char infoBuf[96];

  if (!mpuOk) return "INFO: MPU fehlt";

  if (azLimitWarningActive) {
    snprintf(infoBuf, sizeof(infoBuf),
             "AZWARN|SRC=START|E=%d|W=%d",
             azimuthIsEastLimitDetected() ? 1 : 0,
             azimuthIsWestLimitDetected() ? 1 : 0);
    return infoBuf;
  }

  if (!isMainMenuMode() && !isCenterMode() && !isAutoMode() && manualAxis == MANUAL_AXIS_AZ && azReferenceEndLimitActive()) {
    snprintf(infoBuf, sizeof(infoBuf),
             "AZWARN|SRC=MANUAL|E=%d|W=%d",
             azimuthIsEastLimitDetected() ? 1 : 0,
             azimuthIsWestLimitDetected() ? 1 : 0);
    return infoBuf;
  }

  if (isCenterMode()) {
    if (centerHomingStarted) {
      // V3: Die fruehere CENTERDBG-Zeile mit Phase/Richtung/Zeit/Hallwerten
      // wurde bewusst aus der normalen Infoausgabe entfernt. Sie war bei
      // Ausrichten/Mitte fuer den Anwender nicht hilfreich und hat Web/TFT bzw.
      // Serial-Ausgaben unnoetig technisch wirken lassen. Die eigentliche
      // Mittenfahrt-Logik bleibt unveraendert; nur der sichtbare Infotext ist
      // jetzt bewusst einfach und bedienerfreundlich.
      if (centerTimingState == CENTER_TIMING_FAILED) {
        snprintf(infoBuf, sizeof(infoBuf),
                 "INFO: Mitte Fehler%s%s",
                 centerLastFailText ? " - " : "",
                 centerLastFailText ? centerLastFailText : "");
        return infoBuf;
      }
      return "INFO: Mitte wird gesucht";
    }
    if (azimuthIsCenterDetected()) return "INFO: Mitte erkannt";
    return "INFO: Nach Sueden ausrichten";
  }

  if (isAutoMode()) {
    // Kandidatenmodus bekommt eine bewusst etwas reichhaltigere Anzeige.
    if (autoState == AUTO_STATE_CANDIDATE_HOLD) {
      snprintf(infoBuf, sizeof(infoBuf),
               "SAT: +OK -FALSCH");
      return infoBuf;
    }


    if (autoState >= AUTO_STATE_NEW_CENTER_START && autoState <= AUTO_STATE_EZ_ADJUST_HINT) {
      // V3: In der Web-UI im Menue "Suchen" werden D/AZ bewusst
      // nicht mehr im normalen Infotext angezeigt. Diese Werte sind interne
      // Diagnosewerte fuer Richtung/Positionszaehler und haben den Anwender
      // bei der Bedienung eher verwirrt. Der sichtbare Suchstatus bleibt
      // deshalb auf den Klartext aus autoInfoText() reduziert. Technische
      // Detailwerte koennen weiterhin in Diagnoseausgaben bleiben.
      return autoInfoText();
    }

    return autoInfoText();
  }

  if (isMainMenuMode()) {
    if (azLimitWarningActive) {
      snprintf(infoBuf, sizeof(infoBuf),
               "AZWARN|SRC=START|E=%d|W=%d",
               azimuthIsEastLimitDetected() ? 1 : 0,
               azimuthIsWestLimitDetected() ? 1 : 0);
      return infoBuf;
    }

    if (autoSetupActive) {
      snprintf(infoBuf, sizeof(infoBuf),
               "AUTO_SETUP|EZ=MANUELL|TARGET=%.1f",
               DEFAULT_TARGET_ELEVATION);
      return infoBuf;
    }

    snprintf(infoBuf, sizeof(infoBuf),
             "MENUE_SEL=%d",
             (int)mainMenuSelection);
    return infoBuf;
  }

  snprintf(infoBuf, sizeof(infoBuf),
           "INFO: Manuell aktiv | AZ=%d",
           azPositionSteps);
  return infoBuf;
}

bool liveAutoHasPeak() {
  return autoHasPeak;
}

bool liveAutoFailed() {
  return autoState == AUTO_STATE_FAILED;
}

// Web-/Diagnose-Status: aktiver Kandidatenhalt.
// true bedeutet: AUTO hat einen moeglichen Satelliten gefunden und wartet
// auf Benutzerentscheidung: PLUS/OK oder MINUS/FALSCH.
bool liveIsCandidateHold() {
  return autoState == AUTO_STATE_CANDIDATE_HOLD;
}


// Web-/Diagnose-Status: richtiger Satellit wurde bestaetigt.
// confirmRightSatellite() setzt autoHasPeak=true und AUTO_STATE_COMPLETE.
bool liveIsSatelliteConfirmed() {
  return autoHasPeak || autoState == AUTO_STATE_COMPLETE;
}

// Anzahl intern gesperrter Bereiche falscher Satelliten.
// Wird im Web-UI nur zur Diagnose angezeigt.
int liveGetBlockedRangeCount() {
  return blockedRangeCount();
}

bool liveAutoSetupActive() {
  return autoSetupActive;
}

bool liveAutoFineSearchEnabled() {
  return autoFineSearchEnabled;
}

float liveGetRfVoltage() {
  return rfGetPinVoltage();
}

uint16_t liveGetRfRawAdc() {
  return rfGetRawAdc();
}

float liveGetRfFilteredAdc() {
  return rfGetFilteredAdc();
}

const char* liveGetRfQualityText() {
  // V3: RF-Bewertung anhand der praktischen Aussentest-Grenzen.
  // Diese Ampel dient nur der Anzeige/Diagnose. Sie blockiert bewusst keine
  // Benutzerentscheidung: Wenn der Nutzer im Kandidatenmodus PLUS drueckt,
  // wird die Signaloptimierung auch bei schwachem RF-Wert gestartet.
  //
  // Wichtig fuer AD8317/AD8318 im aktuellen Aufbau:
  // kleinerer ADC-Wert = staerkeres Signal.
  const float adc = rfGetFilteredAdc();
  if (adc <= RF_TV_STRONG_MAX_ADC) return "sehr gut";
  if (adc <= RF_TV_GOOD_MAX_ADC)   return "gut";
  if (adc <= RF_TV_USABLE_MAX_ADC) return "brauchbar";
  return "schwach";
}


// V3: Strukturierte Statusfunktionen fuer die Web-UI-Seite
// "Signal optimieren". Die Web-UI soll nicht interne AUTO-State-Namen
// parsen muessen. Deshalb liefert die Runtime hier bewusst einfache Werte:
// - aktiv: Optimierung laeuft gerade wirklich
// - Phase/Info: Klartext fuer Anzeige
// - Start/Best/Current ADC: Diagnose fuer den Testlauf
// Kleinerer ADC-Wert bedeutet staerkeres Signal.
bool liveSignalOptimizationActive() {
  switch (autoState) {
    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_START:
    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_WAIT:
    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_START:
    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_WAIT:
    case AUTO_STATE_SIGNAL_OPT_EL_UP_START:
    case AUTO_STATE_SIGNAL_OPT_EL_UP_WAIT:
    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_START:
    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_WAIT:
      return true;
    default:
      return false;
  }
}

const char* liveGetSignalOptimizationPhaseText() {
  switch (autoState) {
    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_START:
    case AUTO_STATE_SIGNAL_OPT_AZ_EAST_WAIT:
      return "Azimut Richtung Ost pruefen";
    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_START:
    case AUTO_STATE_SIGNAL_OPT_AZ_WEST_WAIT:
      return "Azimut Richtung West pruefen";
    case AUTO_STATE_SIGNAL_OPT_EL_UP_START:
    case AUTO_STATE_SIGNAL_OPT_EL_UP_WAIT:
      return "Winkel Plus pruefen";
    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_START:
    case AUTO_STATE_SIGNAL_OPT_EL_DOWN_WAIT:
      return "Winkel Minus pruefen";
    case AUTO_STATE_COMPLETE:
      return "Signal optimiert";
    default:
      return "Optimierung bereit";
  }
}

const char* liveGetSignalOptimizationInfoText() {
  if (liveSignalOptimizationActive()) {
    return "Die Anlage testet kleine Azimut- und Winkelschritte. Kleinerer RF-ADC bedeutet besseres Signal.";
  }
  if (autoState == AUTO_STATE_COMPLETE) {
    return "Die Optimierung ist beendet. Die Anlage steht am besten gefundenen Punkt.";
  }
  return "Diese Ansicht ist eine versteckte Diagnoseansicht. Sie wird automatisch geoeffnet, wenn Signal optimieren wirklich laeuft.";
}

float liveGetSignalOptimizationStartAdc() { return signalOptStartAdc; }
float liveGetSignalOptimizationBestAdc()  { return signalOptBestAdc; }
float liveGetSignalOptimizationCurrentAdc() { return rfGetFilteredAdc(); }
int liveGetSignalOptimizationStepsInPhase() { return signalOptStepsInPhase; }

bool liveHallCenter() { return azimuthIsCenterDetected(); }
bool liveHallEast()   { return azimuthIsEastLimitDetected(); }
bool liveHallWest()   { return azimuthIsWestLimitDetected(); }

bool liveRawHallCenter() { return azimuthIsRawCenterPinDetected(); }
bool liveRawHallEast()   { return azimuthIsRawEastLimitPinDetected(); }
bool liveRawHallWest()   { return azimuthIsRawWestLimitPinDetected(); }

// =====================================================
// Public Commands
// =====================================================
// Kommentarstand: V3
//
// Diese Runtime-Kommandos sind die gemeinsame Schnittstelle fuer:
// - lokale Tastenbedienung am Geraet
// - Web-UI-Bedienung ueber web_server.cpp
// - spaetere Test- oder Diagnosefunktionen
//
// Wichtig fuer Wartung:
// Die Web-UI ruft hier nur klare Kommandos auf. Sie soll keine eigene
// Motorlogik, AUTO-Zustandsmaschine oder RF-Bewertung nachbauen. Wenn ein
// Web-Button ein Problem verursacht, ist zuerst zu pruefen, welcher
// liveCommand...() Handler hier aufgerufen wird.
//
// Kandidaten werden nur noch ueber OK/FALSCH/ABBRUCH bedient; alte
// Kandidaten-Feinmodi wurden im Cleanup entfernt.

// Forward-Deklarationen fuer interne Center-/EZ-Pulsfunktionen.
// Kommentarstand: V3
//
// Hintergrund:
// Die Web-UI-Kommandos liveCommandSetupElevationUpPulse() und
// liveCommandSetupElevationDownPulse() stehen im Public-Command-Block weiter
// oben als die eigentlichen Hilfsfunktionen. C++ muss die Funktionen vor dem
// ersten Aufruf kennen.
//
// Wichtig:
// Diese Deklarationen stehen bewusst im globalen Datei-Scope und NICHT im
// anonymen Namespace. Genau dadurch wird der fruehere Compilefehler vermieden,
// bei dem der Compiler zwei gleichnamige Kandidaten gesehen hat:
// - eine Funktion im anonymen Namespace
// - eine globale static-Funktion
//
// Hier gibt es jetzt nur noch eine eindeutige Deklaration und spaeter genau
// eine passende Definition.
static void startCenterElevationPulseUp();
static void startCenterElevationPulseDown();

void liveCommandEnterManual() {
  clearAzLimitWarning();
  abortAutoSequence();
  enterManualMode();
  stopManualActuators();
}

void liveCommandOpenMainMenu() {
  clearAzLimitWarning();
  abortAutoSequence();
  stopManualActuators();
  enterMainMenuMode();
}

void liveCommandStartCentering() {
  if (azReferenceEndLimitActive()) {
    activateAzLimitWarning("MITTE");
    return;
  }

  abortAutoSequence();
  stopManualActuators();
  azLimitWarningActive = false;

  enterCenterMode();
  centerHomingStarted = false;
  centerTimingState = CENTER_TIMING_IDLE;
  centerTimingDir = AZ_DIR_NONE;
  centerReturnDir = AZ_DIR_NONE;
  centerStateStartedAtMs = 0;
  centerEntryAtMs = 0;
  centerZoneWidthMs = 0;
  centerReturnMs = 0;
  centerSearchReverseCount = 0;

  Serial.println("MENUE 1: Ausrichten. Nach Sueden ausrichten, +/- Elevation, MODE startet.");
}

void liveCommandStartCenterRun() {
  // Web-UI-Start fuer die Ausrichten-Seite.
  //
  // Abgrenzung zur lokalen Tastenbedienung:
  // - Am Geraet wird erst Menuepunkt 1 gewaehlt und danach mit MODE gestartet.
  // - Im Web soll ein einziger Button die komplette Center-/Mittenfahrt starten,
  //   weil dort kein physisches MODE-Konzept sichtbar ist.
  //
  // Deshalb ruft diese Funktion zuerst liveCommandStartCentering() auf und startet
  // danach sofort die echte Center-Homing-Routine.
  //
  // Web-UI: Der Button "Ausrichten starten" soll nicht nur den
  // Ausrichten-Modus anzeigen, sondern die echte Mitten-/Centerfahrt
  // ausloesen. Die Tastenlogik bleibt unveraendert: Am Geraet wird
  // weiterhin erst der Menuepunkt ausgewaehlt und danach mit MODE gestartet.
  liveCommandStartCentering();

  if (!isCenterMode()) {
    return;
  }

  startCenterHomingFromAlignMode();
}


void liveCommandAbortCentering() {
  centerSuccessNoticeActive = false;

  // V3: Web-UI-Abbruch fuer die Ausrichten-/Mittenfahrt.
  // Die Center-Zeitmessung schaltet den Azimut teilweise ueber
  // azimuthDriveRawLogical(), damit die Hall-Bereichsvermessung ohne
  // Puls-/Step-Automatiken laufen kann. Deshalb muss ein Web-Abbruch
  // zusaetzlich den direkten Raw-Ausgang ueber centerAzStopOnly()
  // abschalten. Nur azimuthStop() reicht hier nicht in jedem Zustand aus.
  // Ergebnis: Der rote ABBRUCH-Button stoppt die reale Mittenfahrt sofort
  // und setzt danach die Center-Zustandsvariablen zurueck.
  centerAzStopOnly();

  if (isCenterMode() || centerHomingStarted) {
    abortCenterModeToMenu();
  } else {
    azimuthStop();
    elevationStop();
    enterMainMenuMode();
  }
}

bool liveCenteringActive() {
  // V3: "Ausrichten laeuft" darf nur gemeldet werden, wenn die
  // Center-/Mittenfahrt wirklich gestartet wurde.
  // Vorher wurde hier isCenterMode() zurueckgegeben. Dadurch zeigte die
  // Web-UI schon beim blossen Oeffnen des Menuepunkts "Ausrichten/Mitte"
  // faelschlich "Ausrichtung laeuft", obwohl noch kein Start gedrueckt
  // wurde. Das verwirrte auch die Tastenbedienung, weil Anzeige und echter
  // Ablauf nicht zusammenpassten.
  //
  // Richtig ist:
  // - Center-Menue nur geoeffnet: false -> Startbutton anzeigen.
  // - Mittenfahrt aktiv: true -> Abbruchbutton anzeigen.
  // - Fehlerzustand: false -> Fehler/Neustart anzeigen, nicht "laeuft".
  return centerHomingStarted && centerTimingState != CENTER_TIMING_FAILED;
}

bool liveCenteringSuccessNoticeActive() {
  return centerSuccessNoticeActive;
}

const char* liveGetCenteringPhaseText() {
  // V3: Menschlicher Phasentext fuer die Web-UI.
  // Die interne Mittenfahrt arbeitet mit technischen Zustandsnamen
  // (LEAVE_ACTIVE, SEARCH_ENTER, CROSS_EXIT, RETURN_HALF). Diese Funktion
  // uebersetzt sie bewusst in kurze Bedienbegriffe, damit der Nutzer waehrend
  // der Fahrt versteht, was gerade passiert.
  // Die Funktion aendert keine Motorlogik, sondern liefert nur Anzeige-Text.
  if (!centerHomingStarted) {
    if (centerSuccessNoticeActive || centerTimingState == CENTER_TIMING_DONE) return "Mitte erreicht";
    if (centerTimingState == CENTER_TIMING_FAILED) return "Nicht erfolgreich";
    return "Bereit";
  }

  switch (centerTimingState) {
    case CENTER_TIMING_LEAVE_ACTIVE:
      return "Fahrt nach Osten";
    case CENTER_TIMING_SEARCH_ENTER:
      return (centerTimingDir == AZ_DIR_WEST) ? "Fahrt nach Westen" : "Fahrt nach Osten";
    case CENTER_TIMING_CROSS_EXIT:
      return "Mitte vermessen";
    case CENTER_TIMING_RETURN_HALF:
      return "Fahrt zum Zentrum";
    case CENTER_TIMING_DONE:
      return "Mitte erreicht";
    case CENTER_TIMING_FAILED:
      return "Nicht erfolgreich";
    case CENTER_TIMING_IDLE:
    default:
      return "Bereit";
  }
}

const char* liveGetCenteringInfoText() {
  // V3: Zusatztext zur Center-Phase fuer die Web-UI.
  // Wichtig fuer den Live-Test: Phase, RF und Winkel koennen zyklisch
  // aktualisiert werden, ohne die komplette Webseite neu zu laden und ohne
  // dadurch die Mittenfahrt zu beeinflussen.
  if (!centerHomingStarted) {
    if (centerSuccessNoticeActive || centerTimingState == CENTER_TIMING_DONE) {
      return "Die Mitte wurde gesetzt. Danach die Antenne grob nach Sueden ausrichten und Suche starten.";
    }
    if (centerTimingState == CENTER_TIMING_FAILED) {
      return centerLastFailText && centerLastFailText[0] ? centerLastFailText : "Ausrichten wurde nicht erfolgreich beendet.";
    }
    return "Bereit zum Ausrichten. Start setzt die mechanische Mitte als Referenz.";
  }

  switch (centerTimingState) {
    case CENTER_TIMING_LEAVE_ACTIVE:
      return "Center-Bereich wird verlassen.";
    case CENTER_TIMING_SEARCH_ENTER:
      return "Center-Sensor wird gesucht.";
    case CENTER_TIMING_CROSS_EXIT:
      return "Center-Bereich wird durchfahren und vermessen.";
    case CENTER_TIMING_RETURN_HALF:
      return "Rueckfahrt in die berechnete Mitte.";
    case CENTER_TIMING_FAILED:
      return centerLastFailText && centerLastFailText[0] ? centerLastFailText : "Ausrichten wurde nicht erfolgreich beendet.";
    case CENTER_TIMING_DONE:
      return "Die Mitte wurde gesetzt.";
    case CENTER_TIMING_IDLE:
    default:
      return "Ausrichten laeuft.";
  }
}

void liveCommandEnterAuto() {
  // Einstieg in AUTO aus dem Web oder vom Menue.
  // Der Start fuehrt bewusst NICHT sofort in den Suchlauf, sondern zuerst in
  // das Such-Setup. Dort kann der Benutzer die Hoehe mit kurzen Pulsen
  // korrigieren und danach den Suchlauf starten.
  if (!mpuOk) {
    return;
  }

  // Suche soll sich im Web genauso verhalten wie am TFT-Menue:
  // zuerst Hoehe pruefen/korrigieren, erst danach Start.
  startAutoSetupSelection();
}

void liveCommandAzEast() {
  // Semantischer Web-Befehl EAST als manueller Override ohne AZ-Endsensor-Sperre.
  // Web-Befehl bleibt aktiv, bis im Web STOP gedrueckt oder der Modus gewechselt wird.
  abortAutoSequence();
  enterManualMode();
  stopManualActuators();
  setManualAxis(MANUAL_AXIS_AZ);
  webManualAzHoldActive = true;
  manualAzIntent = AZ_DIR_EAST;
  azimuthDriveManualOverride(AZ_DIR_EAST);
}

void liveCommandAzWest() {
  // Semantischer Web-Befehl WEST als manueller Override ohne AZ-Endsensor-Sperre.
  // Web-Befehl bleibt aktiv, bis im Web STOP gedrueckt oder der Modus gewechselt wird.
  abortAutoSequence();
  enterManualMode();
  stopManualActuators();
  setManualAxis(MANUAL_AXIS_AZ);
  webManualAzHoldActive = true;
  manualAzIntent = AZ_DIR_WEST;
  azimuthDriveManualOverride(AZ_DIR_WEST);
}

void liveCommandAzStop() {
  abortAutoSequence();
  enterManualMode();
  azimuthStop();
  manualAzIntent = AZ_DIR_NONE;
  webManualAzHoldActive = false;
}

void liveCommandElUp() {
  // Web-Elevation analog zur manuellen Taste: Override ohne Softlimit-Sperre.
  // Web-Befehl bleibt aktiv, bis im Web STOP gedrueckt oder der Modus gewechselt wird.
  abortAutoSequence();
  enterManualMode();
  stopManualActuators();
  setManualAxis(MANUAL_AXIS_EL);
  webManualElHoldActive = true;
  elevationUp();
  manualElDirection = EL_DIR_UP;
}

void liveCommandElDown() {
  // Web-Elevation analog zur manuellen Taste: Override ohne Softlimit-Sperre.
  // Web-Befehl bleibt aktiv, bis im Web STOP gedrueckt oder der Modus gewechselt wird.
  abortAutoSequence();
  enterManualMode();
  stopManualActuators();
  setManualAxis(MANUAL_AXIS_EL);
  webManualElHoldActive = true;
  elevationDown();
  manualElDirection = EL_DIR_DOWN;
}

void liveCommandElStop() {
  abortAutoSequence();
  enterManualMode();
  elevationStop();
  manualElDirection = EL_DIR_STOP;
  webManualElHoldActive = false;
}

void liveCommandSetupElevationUpPulse() {
  // Web-UI AUTO SETUP: kurze EZ-Korrektur wie im Ausrichten-Menue.
  // Kein Wechsel in den manuellen Dauerfahrtmodus und kein Abbruch des AUTO-Setups.
  if (!mpuOk) return;
  startCenterElevationPulseUp();
}

void liveCommandSetupElevationDownPulse() {
  // Web-UI AUTO SETUP: kurze EZ-Korrektur wie im Ausrichten-Menue.
  // Kein Wechsel in den manuellen Dauerfahrtmodus und kein Abbruch des AUTO-Setups.
  if (!mpuOk) return;
  startCenterElevationPulseDown();
}

void liveCommandToggleAutoFineSearch() {
  if (!autoSetupActive) {
    startAutoSetupSelection();
    return;
  }
  toggleAutoFineSearchOption();
}

void liveCommandStartAutoFromSetup() {
  if (!mpuOk) {
    return;
  }

  if (!autoSetupActive) {
    // V3: Nach einem abgeschlossenen Suchdurchlauf steht AUTO in
    // AUTO_STATE_EZ_ADJUST_HINT. Von dort soll der Button "SUCHE WIEDERHOLEN"
    // direkt denselben Ablauf erneut starten, ohne erst wieder ein Setup zu
    // erzwingen. In allen anderen Faellen wird weiterhin das Such-Setup
    // geoeffnet.
    if (isAutoMode() && autoState == AUTO_STATE_EZ_ADJUST_HINT) {
      startAutoSequence();
      return;
    }

    startAutoSetupSelection();
    return;
  }

  startAutoSequence();
}

void liveCommandAbortAuto() {
  abortAutoSequence();
  enterMainMenuMode();
  Serial.println("WEB: AUTO Abbruch / zurueck ins Hauptmenue.");
}

void liveCommandAzButtonPlus() {
  // PLUS im manuellen Azimut-Menue = Richtung WEST nach getestetem Aufbau.
  // Web-Befehl bleibt aktiv, bis im Web STOP gedrueckt oder der Modus gewechselt wird.
  abortAutoSequence();
  enterManualMode();
  stopManualActuators();
  setManualAxis(MANUAL_AXIS_AZ);
  webManualAzHoldActive = true;
  manualAzIntent = AZ_DIR_WEST;
  azimuthDriveManualOverride(AZ_DIR_WEST);
}

void liveCommandAzButtonMinus() {
  // MINUS im manuellen Azimut-Menue = Richtung EAST nach getestetem Aufbau.
  // Web-Befehl bleibt aktiv, bis im Web STOP gedrueckt oder der Modus gewechselt wird.
  abortAutoSequence();
  enterManualMode();
  stopManualActuators();
  setManualAxis(MANUAL_AXIS_AZ);
  webManualAzHoldActive = true;
  manualAzIntent = AZ_DIR_EAST;
  azimuthDriveManualOverride(AZ_DIR_EAST);
}

void liveCommandElButtonPlus() {
  // Manueller Elevation-Override analog zur physischen PLUS-Taste.
  // Web-Befehl bleibt aktiv, bis im Web STOP gedrueckt oder der Modus gewechselt wird.
  abortAutoSequence();
  enterManualMode();
  stopManualActuators();
  setManualAxis(MANUAL_AXIS_EL);
  webManualElHoldActive = true;
  elevationUp();
  manualElDirection = EL_DIR_UP;
}

void liveCommandElButtonMinus() {
  // Manueller Elevation-Override analog zur physischen MINUS-Taste.
  // Web-Befehl bleibt aktiv, bis im Web STOP gedrueckt oder der Modus gewechselt wird.
  abortAutoSequence();
  enterManualMode();
  stopManualActuators();
  setManualAxis(MANUAL_AXIS_EL);
  webManualElHoldActive = true;
  elevationDown();
  manualElDirection = EL_DIR_DOWN;
}

// =====================================================
// Web-UI Status fuer manuelle Start-/Stop-Bedienung
// =====================================================
// Die Web-UI arbeitet anders als die physischen Tasten:
// - Taste am Geraet: gedrueckt halten = Motor laeuft, loslassen = Stop.
// - Web-Button: AZ+/AZ-/EZ+/EZ- startet, STOP beendet.
// Diese Statusfunktionen spiegeln diesen Web-Zustand optisch in der Web-UI.

bool liveWebManualAzActive() {
  return isManualMode() && manualAxis == MANUAL_AXIS_AZ && webManualAzHoldActive && manualAzIntent != AZ_DIR_NONE;
}

bool liveWebManualElActive() {
  return isManualMode() && manualAxis == MANUAL_AXIS_EL && webManualElHoldActive && manualElDirection != EL_DIR_STOP;
}

const char* liveWebManualAzDirectionText() {
  if (!liveWebManualAzActive()) return "STOP";
  if (manualAzIntent == AZ_DIR_EAST) return "AZ-";
  if (manualAzIntent == AZ_DIR_WEST) return "AZ+";
  return "STOP";
}

const char* liveWebManualElDirectionText() {
  if (!liveWebManualElActive()) return "STOP";
  if (manualElDirection == EL_DIR_UP) return "EZ+";
  if (manualElDirection == EL_DIR_DOWN) return "EZ-";
  return "STOP";
}

void liveCommandCandidateFalse() {
  if (!isCandidateInteractionState()) {
    Serial.println("WEB: Kein Kandidat aktiv, MINUS/Falsch ignoriert.");
    return;
  }

  handleFalseSatelliteAtCurrentCandidate();
  autoState = (autoResumeAfterCandidateState == AUTO_STATE_INACTIVE)
              ? AUTO_STATE_NEW_SCAN_EAST_START
              : autoResumeAfterCandidateState;
  Serial.println("WEB: Kandidat als falsch markiert, Suche wird fortgesetzt.");
}

void liveCommandConfirmSatellite() {
  if (!isCandidateInteractionState()) {
    Serial.println("WEB: Kein Kandidat aktiv, OK ignoriert.");
    return;
  }

  confirmRightSatellite();
}


static void stopCenterElevationAdjust() {
  elevationStop();
  manualElDirection = EL_DIR_STOP;
}

static bool centerCanPulseElevationUp() {
  if (!mpuOk) return false;
  if (elevationIsPulseActive()) return false;
  return canMoveElevationUp();
}

static bool centerCanPulseElevationDown() {
  if (!mpuOk) return false;
  if (elevationIsPulseActive()) return false;
  return canMoveElevationDown();
}

static void startCenterElevationPulseUp() {
  if (!centerCanPulseElevationUp()) {
    stopCenterElevationAdjust();
    Serial.println("MITTE EZ: Hoch nicht erlaubt / Softlimit / MPU");
    return;
  }

  elevationPulseUp(CENTER_EL_ADJUST_PULSE_MS, WEB_EL_PWM);
  manualElDirection = EL_DIR_UP;
  Serial.println("MITTE EZ: kurzer Puls hoch");
}

static void startCenterElevationPulseDown() {
  if (!centerCanPulseElevationDown()) {
    stopCenterElevationAdjust();
    Serial.println("MITTE EZ: Runter nicht erlaubt / Softlimit / MPU");
    return;
  }

  elevationPulseDown(CENTER_EL_ADJUST_PULSE_MS, WEB_EL_PWM);
  manualElDirection = EL_DIR_DOWN;
  Serial.println("MITTE EZ: kurzer Puls runter");
}

static void serviceCenterElevationAdjustSafety() {
  if (!isCenterMode()) return;

  if (!elevationIsPulseActive() && manualElDirection != EL_DIR_STOP) {
    manualElDirection = EL_DIR_STOP;
  }

  if (!mpuOk) {
    if (manualElDirection != EL_DIR_STOP || elevationIsPulseActive()) {
      stopCenterElevationAdjust();
      Serial.println("MITTE EZ: Stop, MPU nicht bereit");
    }
    return;
  }

  const float rel = currentRelativeAngle();

  if (manualElDirection == EL_DIR_UP && rel >= ELEVATION_MAX_SOFT) {
    stopCenterElevationAdjust();
    Serial.println("MITTE EZ: Softlimit MAX erreicht");
  }

  if (manualElDirection == EL_DIR_DOWN && rel <= ELEVATION_MIN_SOFT) {
    stopCenterElevationAdjust();
    Serial.println("MITTE EZ: Softlimit MIN erreicht");
  }
}

static AzimuthDirection oppositeAzimuthDirection(AzimuthDirection dir) {
  if (dir == AZ_DIR_EAST) return AZ_DIR_WEST;
  if (dir == AZ_DIR_WEST) return AZ_DIR_EAST;
  return AZ_DIR_NONE;
}

// =====================================================
// AZ-Mitte-Funktion: neu geschriebene Center-Zeitmessung
// =====================================================
// Dieser Abschnitt ersetzt die vorherigen, mehrfach korrigierten
// Center-/Homing-Hilfsroutinen fuer den Menuepunkt "Mitte einstellen".
// Die alte Logik wurde bewusst nicht weiter geflickt, sondern durch einen
// einfachen, nachvollziehbaren Ablauf ersetzt.
//
// Kernidee:
// Der Center-Hall-Sensor hat keinen mathematischen Punkt, sondern einen
// Einflussbereich. Die Mitte wird deshalb ueber die gemessene Aktivzeit des
// Center-Hall-Bereichs bestimmt:
//
//   Eintritt CENTER  -> Zeitmessung Start
//   Austritt CENTER  -> Zeitmessung Ende
//   Rueckfahrt       -> halbe gemessene Zeit
//
// Daraus ergibt sich die rechnerische Mitte des Hall-Felds.
//
// Verbindlicher Ablauf:
// - Grundrichtung ist immer logisch OSTEN.
// - Wenn CENTER beim Start aktiv ist: nach OSTEN aus dem Center-Bereich
//   herausfahren, dann nach WESTEN zurueck und die CENTER-Breite messen.
// - Wenn kein CENTER aktiv ist: nach OSTEN bis zum naechsten Hall fahren.
//   Falls zuerst ein Endsensor kommt, einmal Richtung wechseln und CENTER
//   aus der Gegenrichtung suchen.
// - Nach kompletter CENTER-Durchfahrt wird um die halbe gemessene Zeit
//   zurueckgefahren.
// - Es gibt nur einen zentralen Hall-Suchtimeout: AZ_HALL_SEARCH_TIMEOUT_MS.
//
// Sicherheitsprinzip:
// Der Timeout ist nur eine Notbremse gegen endloses Drehen. Er soll nicht
// den normalen Ablauf steuern. Normale Entscheidungen werden durch die
// Hall-Sensoren getroffen.

static bool centerHallWest() {
  // Die funktionierende Mittenfunktion ist die Referenz.
  // Dort lag die logische WEST-Seite am historischen EAST_LIMIT-Pin.
  // Deshalb wird hier bewusst direkt der RAW-EAST-Pin gelesen.
  // Das ist kein Verdrahtungstausch, sondern die getestete mechanische
  // Zuordnung fuer die Center-Referenz.
  return azimuthIsRawEastLimitPinDetected();
}

static bool centerHallCenter() {
  // Center-Hall bleibt eindeutig der mittlere Hall-Sensor.
  // Dieser Sensor definiert den Bereich, dessen Aktivzeit gemessen wird.
  return azimuthIsRawCenterPinDetected();
}

static bool centerHallEast() {
  // Gegenstueck zu centerHallWest().
  // Die funktionierende Mitte nutzt den historischen WEST_LIMIT-Pin
  // als logische EAST-Seite. Deshalb wird hier direkt RAW-WEST gelesen.
  return azimuthIsRawWestLimitPinDetected();
}

static bool centerEndLimitActiveForDir(AzimuthDirection dir) {
  // Sicherheitsabfrage vor jeder AZ-Fahrt der Center-Routine.
  // Wenn in der geplanten Fahrtrichtung bereits der passende Endsensor aktiv
  // ist, wird nicht losgefahren. So kann die Routine nicht aktiv in einen
  // bekannten mechanischen Anschlag hineinfahren.
  if (dir == AZ_DIR_EAST) return centerHallEast();
  if (dir == AZ_DIR_WEST) return centerHallWest();
  return true;
}

static void centerAzStopOnly() {
  // Die Mitte schaltet die AZ-Pins nicht mehr direkt selbst.
  // Gestoppt wird ueber die zentrale Azimutsteuerung, damit Ausgangszustand
  // und interne AZ-Statuswerte zusammenpassen.
  azimuthDriveRawLogical(AZ_DIR_NONE);
}

static bool centerAzDrive(AzimuthDirection dir) {
  // Direkte AZ-Ansteuerung nur fuer die Center-Zeitmessung.
  //
  // Die reale EAST/WEST-Korrektur wurde aus dieser Funktion herausgenommen
  // und zentral in azimuth_control.cpp gelegt. Dadurch gilt die getestete
  // Richtung jetzt auch fuer AUTO, manuelle AZ-Steuerung, Homing,
  // Scan und alle weiteren Azimut-Funktionen.
  if (dir == AZ_DIR_NONE) {
    centerAzStopOnly();
    return false;
  }

  if (centerEndLimitActiveForDir(dir)) {
    centerAzStopOnly();
    Serial.print("MITTE FEHLER: Endsensor blockiert Richtung ");
    Serial.println(dirText(dir));
    return false;
  }

  return azimuthDriveRawLogical(dir);
}

static void centerTimingFail(const char* msg) {
  // Einheitlicher Fehlerausgang der Center-Zeitmessung.
  // Jeder Fehler muss zuerst beide AZ-Ausgaenge und die normale
  // Azimutsteuerung stoppen. Danach bleibt der Fehlertext sichtbar, damit
  // die Ursache ueber Display/Serial nachvollziehbar ist.
  centerAzStopOnly();
  azimuthStop();
  centerTimingState = CENTER_TIMING_FAILED;
  centerHomingStarted = true;
  centerTimingDir = AZ_DIR_NONE;
  centerReturnDir = AZ_DIR_NONE;
  centerEntryAtMs = 0;
  centerZoneWidthMs = 0;
  centerReturnMs = 0;
  centerLastFailText = msg ? msg : "unbekannt";
  centerSuccessNoticeActive = false;

  Serial.print("MITTE FEHLER: ");
  Serial.println(centerLastFailText);

  if (centerOwner == CENTER_OWNER_AUTO) {
    centerSuccessNoticeActive = false;
    centerOwner = CENTER_OWNER_MENU;
    failAutoDetailed(shortCenterAutoFailText(centerLastFailText), centerLastFailText);
  }
}

static void centerTimingDone() {
  // Erfolgreicher Abschluss:
  // Nach der halben Rueckfahrzeit steht die Antenne rechnerisch in der
  // Mitte des Center-Hall-Feldes. AZPOS wird deshalb auf 0 gesetzt.
  centerAzStopOnly();
  azimuthStop();
  azPositionSteps = 0;
  centerTimingState = CENTER_TIMING_DONE;
  centerHomingStarted = false;
  centerTimingDir = AZ_DIR_NONE;
  centerReturnDir = AZ_DIR_NONE;
  centerEntryAtMs = 0;
  centerZoneWidthMs = 0;
  centerReturnMs = 0;
  centerSearchReverseCount = 0;

  if (centerOwner == CENTER_OWNER_AUTO) {
    centerSuccessNoticeActive = false;
    centerOwner = CENTER_OWNER_MENU;
    enterAutoMode();
    autoState = (autoReturnAfterCenterState == AUTO_STATE_INACTIVE)
                  ? AUTO_STATE_NEW_MOVE_ASTRA_EAST_START
                  : autoReturnAfterCenterState;
    autoReturnAfterCenterState = AUTO_STATE_INACTIVE;
    Serial.println("AUTO V3: Mitte gesetzt, AUTO-Suchzyklus laeuft weiter.");
    return;
  }

  centerSuccessNoticeActive = true;
  enterMainMenuMode();
  Serial.println("MITTE: Center-Bereich vermessen, halbe Zeit zurueck, Mitte gesetzt.");
}

static void centerStartSearchEnter(AzimuthDirection dir) {
  // Startet eine Suchfahrt bis zum naechsten Eintritt in den Center-Bereich.
  // Diese Fahrt nutzt den zentralen 60s-Sicherheits-Timeout.
  centerAzStopOnly();
  delay(CENTER_DIRECTION_PAUSE_MS);

  centerTimingDir = dir;
  centerReturnDir = AZ_DIR_NONE;
  centerEntryAtMs = 0;
  centerZoneWidthMs = 0;
  centerReturnMs = 0;
  centerStateStartedAtMs = millis();
  centerTimingState = CENTER_TIMING_SEARCH_ENTER;

  if (!centerAzDrive(centerTimingDir)) {
    centerTimingFail("Suchrichtung blockiert");
    return;
  }

  Serial.print("MITTE: Suche CENTER Richtung ");
  Serial.println(dirText(centerTimingDir));
}

static void centerStartLeaveActive() {
  // Sonderfall: Beim Start ist CENTER bereits aktiv.
  // Dann darf die Mitte nicht sofort gesetzt werden, weil wir nur wissen,
  // dass wir irgendwo im Einflussbereich stehen. Deshalb fahren wir zuerst
  // logisch nach OSTEN aus dem Bereich heraus und messen anschließend beim
  // Zurueckfahren die komplette Breite.
  centerAzStopOnly();
  delay(CENTER_DIRECTION_PAUSE_MS);

  // Immer zuerst logisch nach OSTEN aus dem Center-Bereich herausfahren.
  centerTimingDir = AZ_DIR_EAST;
  centerReturnDir = AZ_DIR_NONE;
  centerEntryAtMs = 0;
  centerZoneWidthMs = 0;
  centerReturnMs = 0;
  centerStateStartedAtMs = millis();
  centerTimingState = CENTER_TIMING_LEAVE_ACTIVE;

  if (!centerAzDrive(centerTimingDir)) {
    centerTimingFail("Ausfahrrichtung OST blockiert");
    return;
  }

  Serial.println("MITTE: Start im Center-Bereich. Fahre nach OSTEN heraus.");
}

static void centerStartCrossExit() {
  // Eintritt in den Center-Bereich wurde gerade erkannt.
  // Ab hier beginnt die eigentliche Zeitmessung fuer die Hall-Feld-Breite.
  // Wichtig: Es wird NICHT beim Eintritt gestoppt. Die Antenne faehrt weiter,
  // bis CENTER wieder frei wird. Erst Eintritt + Austritt ergeben die Breite.
  // Jetzt ohne Richtungswechsel weiterfahren, bis CENTER wieder frei ist.
  centerEntryAtMs = millis();
  centerStateStartedAtMs = centerEntryAtMs;
  centerTimingState = CENTER_TIMING_CROSS_EXIT;

  if (!centerAzDrive(centerTimingDir)) {
    centerTimingFail("Center-Durchfahrt blockiert");
    return;
  }

  Serial.print("MITTE: Eintritt CENTER. Messe Breite Richtung ");
  Serial.println(dirText(centerTimingDir));
}

static void centerStartReturnHalf() {
  // CENTER wurde verlassen: Die Breite des aktiven Center-Bereichs ist jetzt
  // bekannt. Die Rueckfahrt um die halbe gemessene Zeit bringt die Antenne
  // rechnerisch in die Mitte dieses Bereichs.
  const unsigned long now = millis();
  centerAzStopOnly();
  delay(CENTER_DIRECTION_PAUSE_MS);

  centerZoneWidthMs = now - centerEntryAtMs;
  centerReturnMs = centerZoneWidthMs / 2;

  if (centerReturnMs < CENTER_RETURN_MIN_MS) {
    centerReturnMs = CENTER_RETURN_MIN_MS;
  }

  centerReturnDir = oppositeAzimuthDirection(centerTimingDir);
  centerStateStartedAtMs = millis();
  centerTimingState = CENTER_TIMING_RETURN_HALF;

  if (!centerAzDrive(centerReturnDir)) {
    centerTimingFail("Rueckfahrt halbe Zeit blockiert");
    return;
  }

  Serial.print("MITTE: Austritt CENTER. Breite ms=");
  Serial.print(centerZoneWidthMs);
  Serial.print(" | Rueckfahrt ms=");
  Serial.print(centerReturnMs);
  Serial.print(" | Richtung ");
  Serial.println(dirText(centerReturnDir));
}

static void startCenterHomingFromAlignMode() {
  // Einstieg aus Menuepunkt 1 "Mitte einstellen".
  // Vor dem Start wurde die Elevation ggf. mit +/- korrigiert.
  // MODE kurz startet diese AZ-Mitte-Zeitmessung.
  if (centerHomingStarted) {
    return;
  }

  stopManualActuators();
  centerAzStopOnly();

  centerOwner = CENTER_OWNER_MENU;
  centerHomingStarted = true;
  centerTimingState = CENTER_TIMING_IDLE;
  centerTimingDir = AZ_DIR_NONE;
  centerReturnDir = AZ_DIR_NONE;
  centerStateStartedAtMs = 0;
  centerEntryAtMs = 0;
  centerZoneWidthMs = 0;
  centerReturnMs = 0;
  centerSearchReverseCount = 0;
  centerLastFailText = "";

  Serial.println("AUSRICHTEN: Center-Zeitmessung gestartet.");
  Serial.println("MITTE: Erste Suchrichtung = EAST / OSTEN");
  Serial.print("MITTE: Hall C/E/W = ");
  Serial.print(centerHallCenter() ? 1 : 0);
  Serial.print("/");
  Serial.print(centerHallEast() ? 1 : 0);
  Serial.print("/");
  Serial.println(centerHallWest() ? 1 : 0);

  if (centerHallCenter()) {
    centerStartLeaveActive();
    return;
  }

  // Kein CENTER aktiv: immer zuerst nach OSTEN zum naechsten Hall-Sensor.
  centerStartSearchEnter(AZ_DIR_EAST);
}


static void startCenterHomingFromAuto() {
  // AUTO-Abschluss nach erfolgreicher Mitte:
  // AUTO nutzt bewusst dieselbe getestete Mitten-Zeitmessung wie Menuepunkt 1.
  // Unterschied nur im Abschluss: Nach erfolgreicher Mitte springt die Routine
  // nicht ins Hauptmenue, sondern zur neuen AUTO-Suche weiter.
  if (centerHomingStarted) {
    return;
  }

  stopManualActuators();
  centerAzStopOnly();

  centerOwner = CENTER_OWNER_AUTO;
  centerHomingStarted = true;
  centerTimingState = CENTER_TIMING_IDLE;
  centerTimingDir = AZ_DIR_NONE;
  centerReturnDir = AZ_DIR_NONE;
  centerStateStartedAtMs = 0;
  centerEntryAtMs = 0;
  centerZoneWidthMs = 0;
  centerReturnMs = 0;
  centerSearchReverseCount = 0;
  centerLastFailText = "";

  Serial.println("AUTO V3: Mittenfunktion gestartet. Erste Suchrichtung = EAST / OSTEN");
  Serial.print("AUTO V3: Hall C/E/W = ");
  Serial.print(centerHallCenter() ? 1 : 0);
  Serial.print("/");
  Serial.print(centerHallEast() ? 1 : 0);
  Serial.print("/");
  Serial.println(centerHallWest() ? 1 : 0);

  if (centerHallCenter()) {
    centerStartLeaveActive();
    return;
  }

  centerStartSearchEnter(AZ_DIR_EAST);
}

static void abortCenterModeToMenu() {
  // Abbruch ueber MODE lang oder Web-UI ABBRUCH.
  // Stoppt AZ und EZ sicher und setzt alle Center-Zeitmessungsflags zurueck.
  // V3: Die Center-Routine nutzt fuer die Hall-Zeitmessung teilweise
  // direkte Raw-Ausgaenge. Darum wird centerAzStopOnly() hier bewusst
  // vor azimuthStop() aufgerufen, damit auch diese direkte Ansteuerung
  // sicher abgeschaltet wird.
  centerAzStopOnly();
  azimuthAbortHoming();
  azimuthStop();
  elevationStop();
  manualElDirection = EL_DIR_STOP;
  centerHomingStarted = false;
  centerTimingState = CENTER_TIMING_IDLE;
  centerTimingDir = AZ_DIR_NONE;
  centerReturnDir = AZ_DIR_NONE;
  centerStateStartedAtMs = 0;
  centerEntryAtMs = 0;
  centerZoneWidthMs = 0;
  centerReturnMs = 0;
  centerSearchReverseCount = 0;
  centerOwner = CENTER_OWNER_MENU;
  enterMainMenuMode();
  Serial.println("MITTE: Abbruch, zurueck ins Hauptmenue.");
}

// =====================================================
// Button Logic
// =====================================================

static void selectNextMainMenuItem() {
  if (mainMenuSelection == MENU_ITEM_CENTER) {
    mainMenuSelection = MENU_ITEM_AUTO;
  } else if (mainMenuSelection == MENU_ITEM_AUTO) {
    mainMenuSelection = MENU_ITEM_MANUAL_AZ;
  } else if (mainMenuSelection == MENU_ITEM_MANUAL_AZ) {
    mainMenuSelection = MENU_ITEM_MANUAL_EL;
  } else {
    mainMenuSelection = MENU_ITEM_CENTER;
  }

  Serial.print("HAUPTMENUE Auswahl: ");
  Serial.println((int)mainMenuSelection);
}

static void selectPreviousMainMenuItem() {
  if (mainMenuSelection == MENU_ITEM_CENTER) {
    mainMenuSelection = MENU_ITEM_MANUAL_EL;
  } else if (mainMenuSelection == MENU_ITEM_MANUAL_EL) {
    mainMenuSelection = MENU_ITEM_MANUAL_AZ;
  } else if (mainMenuSelection == MENU_ITEM_MANUAL_AZ) {
    mainMenuSelection = MENU_ITEM_AUTO;
  } else {
    mainMenuSelection = MENU_ITEM_CENTER;
  }

  Serial.print("HAUPTMENUE Auswahl: ");
  Serial.println((int)mainMenuSelection);
}

static void executeMainMenuSelection() {
  switch (mainMenuSelection) {
    case MENU_ITEM_CENTER:
      liveCommandStartCentering();
      break;

    case MENU_ITEM_AUTO:
      startAutoSetupSelection();
      break;

    case MENU_ITEM_MANUAL_AZ:
      liveCommandEnterManual();
      setManualAxis(MANUAL_AXIS_AZ);
      Serial.println("MENUE 3: Manuell AZ.");
      break;

    case MENU_ITEM_MANUAL_EL:
    default:
      liveCommandEnterManual();
      setManualAxis(MANUAL_AXIS_EL);
      Serial.println("MENUE 4: Manuell EZ.");
      break;
  }
}

static void handleModeShortPressManual() {
  if (isMainMenuMode()) {
    executeMainMenuSelection();
    return;
  }

  if (isCenterMode()) {
    if (!centerHomingStarted) {
      startCenterHomingFromAlignMode();
    }
    return;
  }

  if (!isManualMode()) {
    return;
  }

  stopManualActuators();

  if (manualAxis == MANUAL_AXIS_AZ) {
    setManualAxis(MANUAL_AXIS_EL);
    Serial.println("MODE KURZ -> MANUAL_ELEVATION");
  } else {
    setManualAxis(MANUAL_AXIS_AZ);
    Serial.println("MODE KURZ -> MANUAL_AZIMUT");
  }
}

static void handleModeLongPressNormal() {
  stopManualActuators();

  if (isAutoMode()) {
    abortAutoSequence();
    enterMainMenuMode();
    manualAxis = lastManualAxis;
    Serial.println("MODE LANG -> HAUPTMENUE");
    return;
  }

  if (isCenterMode()) {
    abortCenterModeToMenu();
    Serial.println("MODE LANG -> HAUPTMENUE");
    return;
  }

  if (isMainMenuMode()) {
    executeMainMenuSelection();
    return;
  }

  enterMainMenuMode();
  Serial.println("MODE LANG -> HAUPTMENUE");
}

static void confirmRightSatellite() {
  stopManualActuators();
  clearModeMultiClick();

  // V3: PLUS bedeutet: Der Nutzer bestaetigt den richtigen Satelliten.
  // Danach wird nicht sofort "fertig" gemeldet, sondern die neue Funktion
  // "Signal optimieren" gestartet. Diese sucht mit kleinen AZ- und Winkel-
  // Schritten nach dem lokalen RF-Minimum.
  //
  // Wichtig: Die alten MINUS-/Falsch-Satellitenbereiche bleiben unveraendert.
  // Nur der PLUS-Pfad wurde erweitert.
  autoHasPeak = false;

  // V3: PLUS wird nicht durch die RF-Ampel gesperrt. Der Nutzer sieht
  // ggf. bereits ein TV-Bild und darf die Optimierung daher auch starten,
  // wenn der gemessene RF-Wert nach Aussentest-Grenze noch als schwach gilt.
  // Die Ampel wird nur protokolliert, die Optimierungsroutine bleibt gleich.
  rfUpdate();
  Serial.print("AUTO: Richtiger Satellit bestaetigt -> Signal optimieren | RF_ADC=");
  Serial.print(rfGetFilteredAdc(), 1);
  Serial.print(" | Bewertung=");
  Serial.println(liveGetRfQualityText());
  signalOptStartFromCandidate();
}

static void evaluateCandidateModeClicks() {
  // MODE-Kurzklick hat im Kandidatenmodus keine Funktion mehr.
  // Richtig/falsch liegt eindeutig auf PLUS/MINUS; MODE lang bricht AUTO ab.
  clearModeMultiClick();
}

static void processButtonLogic() {
  updateButton(btnMode);
  updateButton(btnMinus);
  updateButton(btnPlus);

  const int count = pressedCount();

  // Mehrfachtasten-Sperre bleibt grundsätzlich erhalten, weil sie Fehlbedienung
  // und gleichzeitige widersprüchliche Fahrbefehle verhindert.
  if (count > 1) {
    if (!buttonLock) {
      buttonLock = true;
      stopManualActuators();
      clearModeMultiClick();
      Serial.println("TASTENSPERRE: Mehr als eine Taste gedrueckt");
    }
    return;
  }

  if (buttonLock) {
    if (count == 0) {
      buttonLock = false;
      modeLongHandled = false;
      clearModeMultiClick();
      Serial.println("TASTENSPERRE AUFGEHOBEN");
    }
    return;
  }

  // -------------------------------------------------
  // Menuepunkt 1: Mitte einstellen.
  // Vor der eigentlichen Mittenfahrt kann die Elevation mit +/- korrigiert
  // werden. Wichtig: PLUS/MINUS starten hier nur kurze Einzelpulse,
  // keine Dauerfahrt. MODE stoppt zuerst die Elevation und startet dann
  // die Mittenfahrt bzw. bricht bei langem Druck ab.
  // -------------------------------------------------
  if (isCenterMode()) {
    serviceCenterElevationAdjustSafety();

    if (btnMode.pressEvent) {
      stopCenterElevationAdjust();
      modePressStartMs = millis();
      modeLongHandled = false;
    }

    if (btnMode.stablePressed && !modeLongHandled && count == 1) {
      stopCenterElevationAdjust();
      if (millis() - modePressStartMs >= BTN_MODE_LONGPRESS_MS) {
        abortCenterModeToMenu();
        modeLongHandled = true;
        return;
      }
    }

    if (btnMode.releaseEvent && !modeLongHandled) {
      stopCenterElevationAdjust();
      if (!centerHomingStarted) {
        startCenterHomingFromAlignMode();
      }
      return;
    }

    if (!centerHomingStarted && !btnMode.stablePressed) {
      if (btnMinus.pressEvent) {
        centerElButtonPressStartMs = millis();
        startCenterElevationPulseDown();
        return;
      }

      if (btnPlus.pressEvent) {
        centerElButtonPressStartMs = millis();
        startCenterElevationPulseUp();
        return;
      }

      // Watchdog: Falls ein Taster elektrisch haengen bleibt, wird keine
      // weitere Bewegung zugelassen. Die Sperre loest sich erst, wenn alle
      // Tasten wieder losgelassen sind.
      if ((btnMinus.stablePressed || btnPlus.stablePressed) &&
          centerElButtonPressStartMs > 0 &&
          millis() - centerElButtonPressStartMs >= CENTER_EL_BUTTON_HOLD_MAX_MS) {
        stopCenterElevationAdjust();
        buttonLock = true;
        centerElButtonPressStartMs = 0;
        Serial.println("MITTE EZ: Taster zu lange gedrueckt, Stop und Sperre bis Loslassen");
        return;
      }

      if (!btnMinus.stablePressed && !btnPlus.stablePressed) {
        centerElButtonPressStartMs = 0;
      }
    }

    return;
  }

  // -------------------------------------------------
  // Hauptmenue: PLUS/MINUS waehlt Punkt 1-4, MODE bestaetigt.
  // Wenn Suche ausgewaehlt wurde, erscheint zuerst ein Such-Setup.
  // Dort korrigieren PLUS/MINUS die Hoehe, MODE startet den Suchlauf.
  // -------------------------------------------------
  if (isMainMenuMode()) {
    if (azLimitWarningActive) {
      if (btnMode.pressEvent) {
        modePressStartMs = millis();
        modeLongHandled = false;
      }

      if (btnMode.releaseEvent && !modeLongHandled) {
        clearAzLimitWarning();
        return;
      }

      if (btnMode.stablePressed && !modeLongHandled && count == 1) {
        if (millis() - modePressStartMs >= BTN_MODE_LONGPRESS_MS) {
          clearAzLimitWarning();
          modeLongHandled = true;
          return;
        }
      }
      return;
    }

    if (autoSetupActive) {
      if (btnMinus.pressEvent) {
        startCenterElevationPulseDown();
        return;
      }

      if (btnPlus.pressEvent) {
        startCenterElevationPulseUp();
        return;
      }

      if (btnMode.pressEvent) {
        modePressStartMs = millis();
        modeLongHandled = false;
      }

      if (btnMode.stablePressed && !modeLongHandled && count == 1) {
        if (millis() - modePressStartMs >= BTN_MODE_LONGPRESS_MS) {
          cancelAutoSetupSelection();
          modeLongHandled = true;
          return;
        }
      }

      if (btnMode.releaseEvent && !modeLongHandled) {
        startAutoSequence();
        return;
      }

      return;
    }

    if (btnPlus.pressEvent) {
      selectNextMainMenuItem();
      return;
    }

    if (btnMinus.pressEvent) {
      selectPreviousMainMenuItem();
      return;
    }

    if (btnMode.pressEvent) {
      modePressStartMs = millis();
      modeLongHandled = false;
    }

    if (btnMode.stablePressed && !modeLongHandled && count == 1) {
      if (millis() - modePressStartMs >= BTN_MODE_LONGPRESS_MS) {
        executeMainMenuSelection();
        modeLongHandled = true;
        return;
      }
    }

    if (btnMode.releaseEvent && !modeLongHandled) {
      executeMainMenuSelection();
      return;
    }

    return;
  }

  // -------------------------------------------------
  // Kandidatenmodus im AUTO-Betrieb: PLUS = OK, MINUS = falsch, MODE lang = Abbruch
  // -------------------------------------------------
  if (isAutoMode() && autoState == AUTO_STATE_CANDIDATE_HOLD) {
    if (btnMode.pressEvent) {
      modePressStartMs = millis();
      modeLongHandled = false;
    }

    // MODE lang = AUTO-Abbruch. Richtig/falsch liegt nicht mehr auf MODE.
    if (btnMode.stablePressed && !modeLongHandled && count == 1) {
      if (millis() - modePressStartMs >= BTN_MODE_LONGPRESS_MS) {
        abortAutoSequence();
        enterMainMenuMode();
        modeLongHandled = true;
        Serial.println("AUTO V3: Abbruch aus Kandidatenmodus -> Hauptmenue");
        return;
      }
    }

    if (!btnMode.stablePressed) {
      if (btnPlus.pressEvent) {
        confirmRightSatellite();
        return;
      }

      if (btnMinus.pressEvent) {
        handleFalseSatelliteAtCurrentCandidate();
        autoState = (autoResumeAfterCandidateState == AUTO_STATE_INACTIVE)
                    ? AUTO_STATE_NEW_SCAN_EAST_START
                    : autoResumeAfterCandidateState;
        Serial.println("AUTO V3: Kandidat als falsch markiert, Suche wird fortgesetzt.");
        return;
      }
    }

    return;
  }


  // -------------------------------------------------
  // Suchlauf beendet: Mitte erreicht, Hoehe manuell pruefen
  // -------------------------------------------------
  if (isAutoMode() && autoState == AUTO_STATE_EZ_ADJUST_HINT) {
    if (btnMode.pressEvent) {
      modePressStartMs = millis();
      modeLongHandled = false;
    }

    if (btnMode.stablePressed && !modeLongHandled && count == 1) {
      if (millis() - modePressStartMs >= BTN_MODE_LONGPRESS_MS) {
        abortAutoSequence();
        enterMainMenuMode();
        modeLongHandled = true;
        Serial.println("SUCHE V3: Hoehenpruefung beendet -> Hauptmenue");
        return;
      }
    }

    // V3: In dieser Endphase veraendert der Nutzer nur die Hoehe.
    // Danach kann er mit MODE kurz denselben Ablauf Mitte->Ost->West->Mitte
    // erneut starten und am RF-Wert beurteilen, ob die Hoehe besser wurde.
    if (!btnMode.stablePressed) {
      if (btnMinus.pressEvent) {
        startCenterElevationPulseDown();
        return;
      }

      if (btnPlus.pressEvent) {
        startCenterElevationPulseUp();
        return;
      }

      if (btnMode.releaseEvent && !modeLongHandled) {
        Serial.println("SUCHE V3: Wiederholung nach Hoehenkorrektur gestartet.");
        startAutoSequence();
        return;
      }
    }

    return;
  }

  // -------------------------------------------------
  // Bisherige MODE-Logik außerhalb der Kandidatenzustände
  // -------------------------------------------------
  if (btnMode.pressEvent) {
    modePressStartMs = millis();
    modeLongHandled = false;
  }

  if (btnMode.stablePressed && !modeLongHandled && count == 1) {
    if (millis() - modePressStartMs >= BTN_MODE_LONGPRESS_MS) {
      handleModeLongPressNormal();
      modeLongHandled = true;
    }
  }

  if (btnMode.releaseEvent && !modeLongHandled) {
    handleModeShortPressManual();
  }

  // Nur im manuellen Modus Bewegungsbefehle auswerten
  if (!isManualMode()) {
    return;
  }

  if (btnMode.stablePressed) {
    return;
  }

  if (manualAxis == MANUAL_AXIS_AZ) {
    // Manueller Override:
    // Im manuellen Modus hat der Nutzer die volle Kontrolle. Deshalb werden
    // hier keine softwareseitigen AZ-Endsensor-Sperren verwendet. Die Hall-
    // Sensoren bleiben im Serial Monitor sichtbar, blockieren aber nicht.
    // Taste gedrueckt = Motor faehrt, Taste losgelassen = Motor stoppt.
    // Manuelle AZ-Tastenbelegung entspricht dem getesteten Aufbau.
    // Die zentrale EAST/WEST-Motorlogik bleibt unveraendert; nur die Bedienung
    // im manuellen AZ-Menue wird an den realen Test angepasst.
    // PLUS  = Richtung WEST
    // MINUS = Richtung EAST
    if (btnMinus.stablePressed) {
      webManualAzHoldActive = false;
      manualAzIntent = AZ_DIR_EAST;
      azimuthDriveManualOverride(AZ_DIR_EAST);
    } else if (btnPlus.stablePressed) {
      webManualAzHoldActive = false;
      manualAzIntent = AZ_DIR_WEST;
      azimuthDriveManualOverride(AZ_DIR_WEST);
    } else if (webManualAzHoldActive && manualAzIntent != AZ_DIR_NONE) {
      // Web-Befehl ist ein Start-/Stop-Kommando. Ohne diese Ausnahme wuerde
      // die Tastenlogik den Motor sofort stoppen, weil keine physische Taste
      // dauerhaft gedrueckt ist.
      azimuthDriveManualOverride(manualAzIntent);
    } else {
      if (manualAzIntent != AZ_DIR_NONE) {
        azimuthDriveManualOverride(AZ_DIR_NONE);
        manualAzIntent = AZ_DIR_NONE;
      }
    }

    return;
  }

  if (manualAxis == MANUAL_AXIS_EL) {
    // Manueller Override:
    // Keine Softlimit-Pruefung in der manuellen EZ-Steuerung. Der Linearantrieb
    // besitzt eigene Endabschalter; die Software zeigt Grenzen nur noch an.
    if (btnMinus.stablePressed) {
      webManualElHoldActive = false;
      elevationDown();
      manualElDirection = EL_DIR_DOWN;
    } else if (btnPlus.stablePressed) {
      webManualElHoldActive = false;
      elevationUp();
      manualElDirection = EL_DIR_UP;
    } else if (webManualElHoldActive && manualElDirection != EL_DIR_STOP) {
      // Web-Befehl ist ein Start-/Stop-Kommando. Ohne diese Ausnahme wuerde
      // die Tastenlogik den Motor sofort stoppen, weil keine physische Taste
      // dauerhaft gedrueckt ist.
      if (manualElDirection == EL_DIR_UP) {
        elevationUp();
      } else if (manualElDirection == EL_DIR_DOWN) {
        elevationDown();
      }
    } else {
      if (manualElDirection != EL_DIR_STOP) {
        elevationStop();
        manualElDirection = EL_DIR_STOP;
      }
    }
  }
}

static void updateCenterMode() {
  if (!isCenterMode() && !(isAutoMode() && centerHomingStarted)) {
    return;
  }

  // Im ersten Schritt von Menuepunkt 1 wird nur grob nach Sueden ausgerichtet
  // und die Elevation per +/- korrigiert. Die Azimut-Mittenroutine startet
  // erst nach MODE kurz.
  if (!centerHomingStarted) {
    return;
  }

  const unsigned long now = millis();

  switch (centerTimingState) {
    case CENTER_TIMING_LEAVE_ACTIVE:
      // Start im Center-Bereich: in der festen ersten Suchrichtung so lange
      // herausfahren, bis der Center-Hall nicht mehr aktiv ist. Danach wird
      // in Gegenrichtung zurueckgefahren und der Center-Bereich vermessen.
      if (!centerHallCenter()) {
        centerAzStopOnly();
        Serial.println("MITTE: CENTER verlassen. Fahre nach WESTEN zurueck zur Vermessung.");
        centerStartSearchEnter(AZ_DIR_WEST);
        return;
      }

      if (centerEndLimitActiveForDir(centerTimingDir)) {
        centerAzStopOnly();
        Serial.println("MITTE: Endsensor beim Herausfahren erreicht. Wechsle zur Vermessung in Gegenrichtung.");
        centerStartSearchEnter(oppositeAzimuthDirection(centerTimingDir));
        return;
      }

      if (now - centerStateStartedAtMs >= AZ_HALL_SEARCH_TIMEOUT_MS) {
        centerTimingFail("Timeout: Center nicht verlassen");
        return;
      }
      return;

    case CENTER_TIMING_SEARCH_ENTER:
      // Suche den Eintritt in den Center-Hall-Bereich.
      if (centerHallCenter()) {
        centerStartCrossExit();
        return;
      }

      // Falls zuerst ein Endsensor erreicht wird, genau einmal umkehren.
      if (centerEndLimitActiveForDir(centerTimingDir)) {
        centerAzStopOnly();

        if (centerSearchReverseCount == 0) {
          centerSearchReverseCount++;
          Serial.println("MITTE: Endsensor vor CENTER erreicht. Suche in Gegenrichtung.");
          centerStartSearchEnter(oppositeAzimuthDirection(centerTimingDir));
          return;
        }

        centerTimingFail("Center nicht gefunden: beide Richtungen erfolglos");
        return;
      }

      if (now - centerStateStartedAtMs >= AZ_HALL_SEARCH_TIMEOUT_MS) {
        centerAzStopOnly();

        if (centerSearchReverseCount == 0) {
          centerSearchReverseCount++;
          Serial.println("MITTE: Such-Timeout. Suche in Gegenrichtung.");
          centerStartSearchEnter(oppositeAzimuthDirection(centerTimingDir));
          return;
        }

        centerTimingFail("Timeout: Center nicht gefunden");
        return;
      }
      return;

    case CENTER_TIMING_CROSS_EXIT:
      // Nach Eintritt weiterfahren, bis der Center-Hall-Bereich verlassen ist.
      if (!centerHallCenter()) {
        centerStartReturnHalf();
        return;
      }

      if (centerEndLimitActiveForDir(centerTimingDir)) {
        centerTimingFail("Endsensor waehrend Center-Durchfahrt");
        return;
      }

      if (now - centerStateStartedAtMs >= AZ_HALL_SEARCH_TIMEOUT_MS) {
        centerTimingFail("Timeout: Center-Bereich zu breit");
        return;
      }
      return;

    case CENTER_TIMING_RETURN_HALF:
      // Rueckfahrt um die halbe gemessene Center-Breite.
      if (now - centerStateStartedAtMs >= centerReturnMs) {
        centerTimingDone();
        return;
      }

      if (centerEndLimitActiveForDir(centerReturnDir)) {
        centerTimingFail("Endsensor waehrend halber Rueckfahrt");
        return;
      }
      return;

    case CENTER_TIMING_DONE:
    case CENTER_TIMING_FAILED:
    case CENTER_TIMING_IDLE:
    default:
      return;
  }
}


void initLiveRuntime() {
  Serial.println("LIVE RUNTIME MODE");

  // Tasten aktiv LOW.
  // Optional koennen zusaetzlich interne Pull-ups aktiviert werden.
  // Das stabilisiert die Eingänge auch dann, wenn externe Pull-ups vorhanden sind
  // oder ein Tastereingang beim Start nicht sauber HIGH wird.
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_MINUS, INPUT_PULLUP);
  pinMode(PIN_BTN_PLUS, INPUT_PULLUP);

  initElevation();
  elevationStop();

  initAzimuth();
  azimuthStop();

  if (!initMPU6050()) {
    Serial.println("MPU6050 nicht gefunden.");
    mpuOk = false;
  } else {
    Serial.println("Kalibriere Gyro...");

    if (!mpuCalibrateGyro()) {
      Serial.println("Gyro Kalibrierung fehlgeschlagen.");
      mpuOk = false;
    } else {
      // V3: MPU-Filter NICHT mehr kuenstlich auf 0.0 Grad setzen.
      //
      // Hintergrund:
      // Frueher wurde hier mpuResetFilter(0.0f) aufgerufen. Dadurch war
      // filterInitialized sofort true, obwohl 0.0 Grad nicht der echte
      // Sensorwinkel war. Der Komplementaerfilter musste sich danach erst
      // langsam an den realen Winkel annaehren.
      //
      // Genau das war fuer die Boot-EZ-Anfahrt problematisch:
      // Direkt nach dem Einschalten wurde auf dem TFT z. B. ein falscher
      // Istwert um ca. 66 Grad angezeigt, waehrend der spaeter stabile
      // Live-Wert bei ca. 27-30 Grad lag. Die automatische Standard-EZ-
      // Anfahrt hat dadurch mit einem falschen Istwert entschieden.
      //
      // Korrektur:
      // Der Filter bleibt nach initMPU6050() absichtlich uninitialisiert.
      // Der erste erfolgreiche mpuUpdateFilteredAngle()-Aufruf setzt den
      // Filter direkt auf den echten Beschleunigungswinkel. Danach folgen
      // mehrere Updates zur kurzen Stabilisierung.
      for (int i = 0; i < 60; i++) {
        mpuUpdateFilteredAngle();
        delay(10);
      }

      mpuOk = true;
      // Keine automatische Session-Referenz setzen.
    }
  }

  enterMainMenuMode();
  manualAxis = MANUAL_AXIS_AZ;
  lastManualAxis = MANUAL_AXIS_AZ;
  manualAzIntent = AZ_DIR_NONE;
  manualElDirection = EL_DIR_STOP;
  azPositionSteps = 0;

  resetAutoState();
  clearBlockedRanges();

  initOk = true;

  Serial.println("System bereit.");
  Serial.println("LIVE: Start im HAUPTMENUE");
  Serial.println("LIVE: Hauptmenue: +/- Auswahl, MODE bestaetigt");
  Serial.println("LIVE: 1=Mitte einstellen | 2=Automatik | 3=Manuell AZ | 4=Manuell EZ");
  Serial.println("LIVE: In Mitte einstellen: +/- kurze EZ-Pulse, MODE startet Mittenfahrt");
  Serial.println("AUTO V3 KANDIDAT: PLUS=richtig | MINUS=falsch | MODE lang=Abbruch");
  Serial.println("LIVE: Mehrfachtasten = Sperre bis alle losgelassen");
}

void runLiveRuntime() {
  if (!initOk) {
    return;
  }

  azimuthUpdate();
  elevationUpdate();


  if (mpuOk) {
    mpuUpdateFilteredAngle();
  }

  processButtonLogic();
  serviceCenterElevationAdjustSafety();
  updateCenterMode();
  updateAutoStrategy();

  // Manueller EZ-Override:
  // Die Softlimit-Stoppruefung darf im manuellen EZ-Modus nicht mehr eingreifen.
  // Im manuellen Modus hat der User bewusst volle Kontrolle; die internen
  // Endabschalter des Linearantriebs bleiben die mechanische Schutzebene.
  // Fuer den Center-/Mitte-Modus bleibt die Softlimit-Pruefung erhalten, damit
  // automatische bzw. halbautomatische EZ-Bewegungen weiterhin begrenzt werden.
  if (mpuOk && isCenterMode()) {
    const float rel = currentRelativeAngle();

    if (manualElDirection == EL_DIR_UP && rel >= ELEVATION_MAX_SOFT) {
      elevationStop();
      manualElDirection = EL_DIR_STOP;
      Serial.println("SOFTLIMIT MAX ERREICHT (CENTER).");
    }

    if (manualElDirection == EL_DIR_DOWN && rel <= ELEVATION_MIN_SOFT) {
      elevationStop();
      manualElDirection = EL_DIR_STOP;
      Serial.println("SOFTLIMIT MIN ERREICHT (CENTER).");
    }
  }

  if (millis() - lastPrint < 500) {
    return;
  }

  lastPrint = millis();

  Serial.print("MODE=");
  Serial.print(liveGetModeText());

  Serial.print(" | AXIS=");
  Serial.print(liveGetAxisText());

  Serial.print(" | AUTO_STATE=");
  Serial.print(liveGetAutoStateText());

  Serial.print(" | FILTER=");
  Serial.print(mpuGetFilteredAngleDeg(), 2);

  Serial.print(" | REL=");
  Serial.print(currentRelativeAngle(), 2);

  Serial.print(" | RF=");
  Serial.print(rfGetPinVoltage(), 3);

  Serial.print(" | AZPOS=");
  Serial.print(azPositionSteps);

  // Hall-Diagnose:
  // HREF = Center-Referenz C/E/W, wird fuer die logische Bewegungsseite genutzt.
  // HRAW = echte Eingangspins laut pins.h C/E/W, hilft beim Verdrahtungs-Test.
  Serial.print(" | HREF C/E/W=");
  Serial.print(azimuthIsCenterDetected() ? 1 : 0);
  Serial.print("/");
  Serial.print(azimuthIsEastLimitDetected() ? 1 : 0);
  Serial.print("/");
  Serial.print(azimuthIsWestLimitDetected() ? 1 : 0);

  Serial.print(" | HRAW C/E/W=");
  Serial.print(azimuthIsRawCenterPinDetected() ? 1 : 0);
  Serial.print("/");
  Serial.print(azimuthIsRawEastLimitPinDetected() ? 1 : 0);
  Serial.print("/");
  Serial.print(azimuthIsRawWestLimitPinDetected() ? 1 : 0);

  Serial.print(" | BLOCKED=");
  Serial.print(blockedRangeCount());

  Serial.print(" | AZ=");
  Serial.print(liveGetAzimuthStateText());

  if (isCenterMode() && centerHomingStarted) {
    const unsigned long now = millis();
    const unsigned long elapsed = (centerStateStartedAtMs > 0) ? (now - centerStateStartedAtMs) : 0;
    const unsigned long timeoutMs = centerTimingTimeoutMs(centerTimingState);
    const AzimuthDirection shownDir = (centerTimingState == CENTER_TIMING_RETURN_HALF) ? centerReturnDir : centerTimingDir;
    Serial.print(" | CENTER_PHASE=");
    Serial.print(centerTimingStateText(centerTimingState));
    Serial.print(" | DIR=");
    Serial.print(dirText(shownDir));
    Serial.print(" | T=");
    Serial.print(elapsed);
    Serial.print("/");
    Serial.print(timeoutMs);
    Serial.print(" | HALL_REF C/E/W=");
    Serial.print(centerHallCenter() ? 1 : 0);
    Serial.print("/");
    Serial.print(centerHallEast() ? 1 : 0);
    Serial.print("/");
    Serial.print(centerHallWest() ? 1 : 0);
    if (centerTimingState == CENTER_TIMING_FAILED) {
      Serial.print(" | ERR=");
      Serial.print(centerLastFailText);
    }
  }

  Serial.print(" | EL=");
  Serial.print(liveGetElevationStateText());

  Serial.print(" | LOCK=");
  Serial.println(buttonLock ? "JA" : "NEIN");
}
