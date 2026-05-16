/*
  SatAlign V3 - Azimutsteuerung
  ------------------------------------------------------------
  Kapselt horizontale Motorbefehle und Hall-Sensor-Auswertung fuer Center,
  Ost und West. Die Hall-Sensoren sind die primaeren Grenzen fuer Azimut.
*/

/*
  SatAlign ESP32 - azimuth_control.cpp
  ---------------------------------------------------------------------------
  Kapselt die Azimutachse: Relais-/Optokoppler-Ausgaenge, EAST/WEST-Logik,
  Hall-Endsensoren, Puls- und Stepbetrieb sowie manuellen Override.

  WICHTIG: Die reale Pin-Zuordnung wurde nach der getesteten Mittenfunktion
  zentralisiert. Logische Richtung und historische Pin-Namen koennen deshalb
  unterschiedlich wirken. Nicht lokal in anderen Dateien erneut tauschen.
*/
#include <Arduino.h>

#include "pins.h"
#include "config.h"
#include "azimuth_control.h"

namespace {
  // Interner Laufmodus der Azimut-Achse.
  //
  // AZ_RUN_IDLE         -> keine Bewegung aktiv
  // AZ_RUN_SINGLE_PULSE -> ein einzelner zeitlich begrenzter Puls läuft
  // AZ_RUN_STEPPED      -> gestufter Wiederholbetrieb läuft
  enum AzimuthRunMode {
    AZ_RUN_IDLE = 0,
    AZ_RUN_SINGLE_PULSE,
    AZ_RUN_STEPPED
  };

  // Aktueller Laufmodus
  AzimuthRunMode runMode = AZ_RUN_IDLE;

  // Zeitmarke für Beginn der aktuellen Phase
  unsigned long phaseStartedAtMs = 0;

  // Aktuelle Einschaltzeit des laufenden Pulses
  unsigned long currentOnTimeMs = 0;

  // true = Ausgang aktuell aktiv
  // false = Ausgang aktuell aus / Pausenphase
  bool outputOn = false;

  // Merkt im Step-Modus, ob der erste längere Puls noch aussteht
  bool firstSteppedPulsePending = false;

  // Zählt abgeschlossene Schritte / Pulse
  unsigned long stepCounter = 0;

  // Aktuelle Bewegungsrichtung
  AzimuthDirection currentDirection = AZ_DIR_NONE;

  // ---------------------------------------------------
  // Homing / Referenzfahrt
  // ---------------------------------------------------

  // true = Referenzfahrt zur Mitte läuft
  bool homingActive = false;

  // true = Mitte erfolgreich gefunden
  bool homed = false;

  // true = Referenzfahrt fehlgeschlagen
  bool homingFailed = false;

  // Aktuelle Suchrichtung der Referenzfahrt
  AzimuthDirection homingSearchDirection = AZ_DIR_NONE;
}

// Prüft einen Hall-Pin unter Berücksichtigung der konfigurierten Logik.
//
// Je nach Aufbau gilt:
// - active low  -> LOW bedeutet Sensor aktiv
// - active high -> HIGH bedeutet Sensor aktiv
static bool hallPinActive(int pin) {
  const int raw = digitalRead(pin);

  if (AZ_HALL_ACTIVE_LOW) {
    return (raw == LOW);
  }

  return (raw == HIGH);
}

// ---------------------------------------------------
// Hall-Sensoren: Rohpins und logische EAST/WEST-Seiten
// ---------------------------------------------------
//
// Hall-/Richtungsreferenz nach getesteter Mittenfunktion:
// Die funktionierende Mittenfunktion ist hier die verbindliche
// Referenz. Deshalb wird nicht nach historischen Pin-Namen entschieden,
// sondern nach der mechanischen Wirkung am Aufbau.
//
// Begriffe:
// - RAW/PIN: tatsaechlicher Eingang gemaess pins.h
// - REF/LOGISCH: mechanische Seite EAST/WEST nach der getesteten Mitte
//
// Referenz der getesteten Mitte:
// - REF EAST liegt am historischen WEST_LIMIT-Pin.
// - REF WEST liegt am historischen EAST_LIMIT-Pin.
//
// WICHTIG:
// Wenn im Serial Monitor HRAW-E aktiv ist, bedeutet das nur, dass der
// physische EAST_LIMIT-Pin aktiv ist. Nach Center-Referenz ist das die
// logische WEST-Seite. Entscheidend fuer AUTO/Mitte ist HREF, nicht HRAW.

bool azimuthIsRawCenterPinDetected() {
  return hallPinActive(PIN_AZ_HALL_CENTER);
}

bool azimuthIsRawEastLimitPinDetected() {
  return hallPinActive(PIN_AZ_HALL_EAST_LIMIT);
}

bool azimuthIsRawWestLimitPinDetected() {
  return hallPinActive(PIN_AZ_HALL_WEST_LIMIT);
}

// Liefert true, wenn der Mittensensor aktiv ist.
bool azimuthIsCenterDetected() {
  return azimuthIsRawCenterPinDetected();
}

// Liefert true, wenn die LOGISCHE Ost-Seite aktiv ist.
bool azimuthIsEastLimitDetected() {
  return azimuthIsRawWestLimitPinDetected();
}

// Liefert true, wenn die LOGISCHE West-Seite aktiv ist.
bool azimuthIsWestLimitDetected() {
  return azimuthIsRawEastLimitPinDetected();
}

// Darf nach EAST gefahren werden?
// Nein, wenn der Ost-Endanschlag bereits aktiv ist.
bool azimuthCanMoveEast() {
  return !azimuthIsEastLimitDetected();
}

// Darf nach WEST gefahren werden?
// Nein, wenn der West-Endanschlag bereits aktiv ist.
bool azimuthCanMoveWest() {
  return !azimuthIsWestLimitDetected();
}

// Schaltet beide Azimut-Ausgänge aus.
//
// WICHTIG:
// Das ist der softwareseitig stromlose Endzustand der Azimut-Ansteuerung.
static void azimuthOutputsOff() {
  digitalWrite(PIN_AZ_EAST, LOW);
  digitalWrite(PIN_AZ_WEST, LOW);
}

// Setzt den kompletten Bewegungszustand zurück.
//
// WICHTIG:
// Diese Funktion beendet keine Homing-Zustände explizit,
// sondern nur den Bewegungs-/Pulszustand.
// Homing-spezifische Flags werden an anderen Stellen behandelt.
static void resetMotionState() {
  azimuthOutputsOff();

  runMode = AZ_RUN_IDLE;
  outputOn = false;
  firstSteppedPulsePending = false;
  currentOnTimeMs = 0;
  currentDirection = AZ_DIR_NONE;
}

// Legt die aktuelle LOGISCHE Richtung auf die physischen Ausgaenge,
// aber ohne Endsensor-/Softwarefreigabe.
//
// Manueller Override:
// Diese Funktion ist ausschliesslich die technische Basis fuer den manuellen
// AZ-Modus. Dort soll der Nutzer bewusst volle Kontrolle haben. Die reale
// EAST/WEST-Korrektur bleibt erhalten, nur die Sperrlogik wird uebersprungen.
static void azimuthApplyDirectionNoLimit(AzimuthDirection direction) {
  if (direction == AZ_DIR_EAST) {
    digitalWrite(PIN_AZ_EAST, LOW);
    digitalWrite(PIN_AZ_WEST, HIGH);
    return;
  }

  if (direction == AZ_DIR_WEST) {
    digitalWrite(PIN_AZ_EAST, HIGH);
    digitalWrite(PIN_AZ_WEST, LOW);
    return;
  }

  azimuthOutputsOff();
}

// Legt die aktuelle LOGISCHE Richtung auf die physischen Ausgänge.
//
// WICHTIG - zentrale EAST/WEST-Korrektur:
// Die funktionierende Mitten-Funktion hat im realen Aufbau gezeigt:
// - PIN_AZ_WEST bewegt die Antenne mechanisch nach OSTEN.
// - PIN_AZ_EAST bewegt die Antenne mechanisch nach WESTEN.
//
// Deshalb wird diese Zuordnung zentral hier festgelegt.
// Alle anderen Projektteile sollen logisch weiter mit EAST/WEST arbeiten,
// aber nicht mehr selbst die Pins vertauschen. Dadurch fahren AUTO, manuelle
// Steuerung, Homing, Scan und Mitte nach derselben getesteten Richtung.
//
// Dabei wird IMMER nochmal geprüft, ob die jeweilige LOGISCHE Richtung
// laut Endanschlag-Sensor überhaupt noch erlaubt ist.
static void azimuthApplyDirection(AzimuthDirection direction) {
  if (direction == AZ_DIR_EAST) {
    if (!azimuthCanMoveEast()) {
      azimuthOutputsOff();
      return;
    }

    // Logisch EAST = real nach OSTEN.
    // Am aktuellen Aufbau wird dafuer der historisch als WEST benannte Pin geschaltet.
    digitalWrite(PIN_AZ_EAST, LOW);
    digitalWrite(PIN_AZ_WEST, HIGH);
    return;
  }

  if (direction == AZ_DIR_WEST) {
    if (!azimuthCanMoveWest()) {
      azimuthOutputsOff();
      return;
    }

    // Logisch WEST = real nach WESTEN.
    // Am aktuellen Aufbau wird dafuer der historisch als EAST benannte Pin geschaltet.
    digitalWrite(PIN_AZ_EAST, HIGH);
    digitalWrite(PIN_AZ_WEST, LOW);
    return;
  }

  azimuthOutputsOff();
}

// Direkte logische AZ-Ansteuerung fuer Routinen, die selbst eine eigene
// Zustandsmaschine besitzen, z. B. die Center-Zeitmessung.
//
// Diese Funktion nutzt dieselbe zentrale EAST/WEST-Korrektur wie alle
// normalen Azimut-Funktionen, startet aber keinen Puls- oder Step-Modus.
// Damit kann die Center-Zeitmessung ihre Zeiten weiterhin selbst messen,
// ohne eine zweite, abweichende Pin-Logik in live_runtime.cpp zu pflegen.
bool azimuthDriveRawLogical(AzimuthDirection direction) {
  if (direction == AZ_DIR_NONE) {
    resetMotionState();
    return false;
  }

  if (direction == AZ_DIR_EAST && !azimuthCanMoveEast()) {
    resetMotionState();
    return false;
  }

  if (direction == AZ_DIR_WEST && !azimuthCanMoveWest()) {
    resetMotionState();
    return false;
  }

  // Kein Pulsmodus: Die aufrufende Routine entscheidet selbst, wann gestoppt wird.
  runMode = AZ_RUN_IDLE;
  currentDirection = direction;
  outputOn = true;
  firstSteppedPulsePending = false;
  currentOnTimeMs = 0;

  azimuthApplyDirection(direction);
  return true;
}

// Direkte manuelle AZ-Ansteuerung ohne Endsensor-/Software-Sperre.
//
// Hinweis:
// Nur fuer den manuellen Modus. AUTO, Mitte und Suchlogik verwenden weiterhin
// die normalen, abgesicherten Funktionen.
bool azimuthDriveManualOverride(AzimuthDirection direction) {
  if (direction == AZ_DIR_NONE) {
    resetMotionState();
    return false;
  }

  runMode = AZ_RUN_IDLE;
  currentDirection = direction;
  outputOn = true;
  firstSteppedPulsePending = false;
  currentOnTimeMs = 0;

  azimuthApplyDirectionNoLimit(direction);
  return true;
}

// Startet einen einzelnen Azimut-Puls.
//
// Ablauf:
// - Bewegungszustand zurücksetzen
// - Richtung auf Plausibilität prüfen
// - Einzelpulsmodus aktivieren
// - Ausgang direkt einschalten
static bool startSinglePulseInternal(AzimuthDirection direction, unsigned long pulseMs) {
  resetMotionState();

  if (direction == AZ_DIR_EAST && !azimuthCanMoveEast()) {
    Serial.println("AZ BLOCKIERT: OST-ENDSENSOR AKTIV");
    return false;
  }

  if (direction == AZ_DIR_WEST && !azimuthCanMoveWest()) {
    Serial.println("AZ BLOCKIERT: WEST-ENDSENSOR AKTIV");
    return false;
  }

  runMode = AZ_RUN_SINGLE_PULSE;
  currentDirection = direction;
  stepCounter = 0;

  outputOn = true;
  firstSteppedPulsePending = false;
  currentOnTimeMs = pulseMs;
  phaseStartedAtMs = millis();

  azimuthApplyDirection(direction);
  return true;
}

// Startet den gestuften Azimut-Betrieb.
//
// Ablauf:
// - Bewegungszustand zurücksetzen
// - Endanschlag prüfen
// - Step-Modus aktivieren
// - erster Puls wird als "längerer Startpuls" behandelt
static bool startSteppedInternal(AzimuthDirection direction) {
  resetMotionState();

  if (direction == AZ_DIR_EAST && !azimuthCanMoveEast()) {
    Serial.println("AZ BLOCKIERT: OST-ENDSENSOR AKTIV");
    return false;
  }

  if (direction == AZ_DIR_WEST && !azimuthCanMoveWest()) {
    Serial.println("AZ BLOCKIERT: WEST-ENDSENSOR AKTIV");
    return false;
  }

  runMode = AZ_RUN_STEPPED;
  currentDirection = direction;
  stepCounter = 0;

  outputOn = true;
  firstSteppedPulsePending = true;
  currentOnTimeMs = AZ_FIRST_PULSE_ON_MS;
  phaseStartedAtMs = millis();

  azimuthApplyDirection(direction);
  return true;
}

// Initialisiert die Azimut-Hardware.
//
// Aufgaben:
// - EAST/WEST-Ausgänge vorbereiten
// - Hall-Sensor-Eingänge vorbereiten
// - je nach Konfiguration mit oder ohne Pull-up
// - Bewegungs- und Homing-Zustände zurücksetzen
void initAzimuth() {
  pinMode(PIN_AZ_EAST, OUTPUT);
  pinMode(PIN_AZ_WEST, OUTPUT);

  if (AZ_HALL_USE_INTERNAL_PULLUP) {
    pinMode(PIN_AZ_HALL_CENTER, INPUT_PULLUP);
    pinMode(PIN_AZ_HALL_EAST_LIMIT, INPUT_PULLUP);
    pinMode(PIN_AZ_HALL_WEST_LIMIT, INPUT_PULLUP);
  } else {
    pinMode(PIN_AZ_HALL_CENTER, INPUT);
    pinMode(PIN_AZ_HALL_EAST_LIMIT, INPUT);
    pinMode(PIN_AZ_HALL_WEST_LIMIT, INPUT);
  }

  resetMotionState();
  homingActive = false;
  homed = false;
  homingFailed = false;
  homingSearchDirection = AZ_DIR_NONE;
}

// Stoppt jede Azimutbewegung sofort und beendet ggf. auch Homing.
void azimuthStop() {
  resetMotionState();
  homingActive = false;
  homingSearchDirection = AZ_DIR_NONE;
}

// Schaltet den Azimut roh nach EAST, falls erlaubt.
void azimuthEastOn() {
  if (!azimuthCanMoveEast()) {
    resetMotionState();
    return;
  }

  currentDirection = AZ_DIR_EAST;
  azimuthApplyDirection(AZ_DIR_EAST);
}

// Schaltet den Azimut roh nach WEST, falls erlaubt.
void azimuthWestOn() {
  if (!azimuthCanMoveWest()) {
    resetMotionState();
    return;
  }

  currentDirection = AZ_DIR_WEST;
  azimuthApplyDirection(AZ_DIR_WEST);
}

// Startet einen einzelnen Puls nach EAST.
// Ggf. laufendes Homing wird dabei verlassen.
void azimuthPulseEast(unsigned long pulseMs) {
  homingActive = false;
  homingSearchDirection = AZ_DIR_NONE;
  startSinglePulseInternal(AZ_DIR_EAST, pulseMs);
}

// Startet einen einzelnen Puls nach WEST.
// Ggf. laufendes Homing wird dabei verlassen.
void azimuthPulseWest(unsigned long pulseMs) {
  homingActive = false;
  homingSearchDirection = AZ_DIR_NONE;
  startSinglePulseInternal(AZ_DIR_WEST, pulseMs);
}

// Startet den gestuften Lauf nach EAST.
// Ggf. laufendes Homing wird dabei verlassen.
void azimuthStartEastStepped() {
  homingActive = false;
  homingSearchDirection = AZ_DIR_NONE;
  startSteppedInternal(AZ_DIR_EAST);
}

// Startet den gestuften Lauf nach WEST.
// Ggf. laufendes Homing wird dabei verlassen.
void azimuthStartWestStepped() {
  homingActive = false;
  homingSearchDirection = AZ_DIR_NONE;
  startSteppedInternal(AZ_DIR_WEST);
}

// Startet die Referenzfahrt zur Mitte.
//
// Logik:
// 1. Wenn Mitte schon erkannt -> sofort fertig
// 2. Sonst bevorzugte Startsuchrichtung aus den Settings
// 3. Wenn diese blockiert ist -> Gegenrichtung versuchen
// 4. Wenn keine Richtung möglich -> Homing-Fehler
void azimuthStartHomingToCenter() {
  resetMotionState();

  homingActive = false;
  homed = false;
  homingFailed = false;
  homingSearchDirection = AZ_DIR_NONE;

  if (azimuthIsCenterDetected()) {
    homed = true;
    Serial.println("AZ HOME: Mitte bereits erkannt.");
    return;
  }

  homingActive = true;

  if (AZ_HOME_FIRST_SEARCH_DIRECTION == AZ_DIR_EAST) {
    if (startSteppedInternal(AZ_DIR_EAST)) {
      homingSearchDirection = AZ_DIR_EAST;
      Serial.println("AZ HOME: Suche Mitte zuerst nach OST.");
      return;
    }

    if (startSteppedInternal(AZ_DIR_WEST)) {
      homingSearchDirection = AZ_DIR_WEST;
      Serial.println("AZ HOME: OST blockiert, suche Mitte nach WEST.");
      return;
    }
  } else {
    if (startSteppedInternal(AZ_DIR_WEST)) {
      homingSearchDirection = AZ_DIR_WEST;
      Serial.println("AZ HOME: Suche Mitte zuerst nach WEST.");
      return;
    }

    if (startSteppedInternal(AZ_DIR_EAST)) {
      homingSearchDirection = AZ_DIR_EAST;
      Serial.println("AZ HOME: WEST blockiert, suche Mitte nach OST.");
      return;
    }
  }

  homingActive = false;
  homingFailed = true;
  Serial.println("AZ HOME FEHLER: Keine Suchrichtung moeglich.");
}

// Bricht eine laufende Referenzfahrt ab.
void azimuthAbortHoming() {
  if (homingActive) {
    Serial.println("AZ HOME: Abgebrochen.");
  }

  resetMotionState();
  homingActive = false;
  homingSearchDirection = AZ_DIR_NONE;
}

// Statusabfrage: Homing gerade aktiv?
bool azimuthIsHomingActive() {
  return homingActive;
}

// Statusabfrage: Homing erfolgreich abgeschlossen?
bool azimuthIsHomed() {
  return homed;
}

// Statusabfrage: Homing in Fehler gelaufen?
bool azimuthDidHomingFail() {
  return homingFailed;
}

// Zyklische Aktualisierung der Azimut-Logik.
//
// Hier passieren:
// - Mittenerkennung während Homing
// - Endanschlag-Reaktionen
// - Pulsende
// - Step-Fortsetzung nach Pause
void azimuthUpdate() {
  // Wenn während Homing die Mitte erkannt wird,
  // ist die Referenzfahrt erfolgreich abgeschlossen.
  if (homingActive && azimuthIsCenterDetected()) {
    resetMotionState();
    homingActive = false;
    homed = true;
    homingFailed = false;
    homingSearchDirection = AZ_DIR_NONE;
    Serial.println("AZ HOME: Mitte gefunden.");
    return;
  }

  // EAST-Endanschlag erreicht
  if (currentDirection == AZ_DIR_EAST && azimuthIsEastLimitDetected()) {
    // Spezialfall: Homing suchte gerade nach EAST,
    // jetzt auf WEST ausweichen.
    if (homingActive && homingSearchDirection == AZ_DIR_EAST) {
      if (startSteppedInternal(AZ_DIR_WEST)) {
        homingSearchDirection = AZ_DIR_WEST;
        Serial.println("AZ HOME: OST-Ende erreicht, suche weiter nach WEST.");
        return;
      }

      resetMotionState();
      homingActive = false;
      homingFailed = true;
      homingSearchDirection = AZ_DIR_NONE;
      Serial.println("AZ HOME FEHLER: OST erreicht, WEST nicht fahrbar.");
      return;
    }

    Serial.println("AZ STOP: OST-ENDSENSOR ERREICHT");
    resetMotionState();
    return;
  }

  // WEST-Endanschlag erreicht
  if (currentDirection == AZ_DIR_WEST && azimuthIsWestLimitDetected()) {
    // Spezialfall: Homing suchte gerade nach WEST,
    // Mitte wurde bis dahin nicht gefunden -> Fehler.
    if (homingActive && homingSearchDirection == AZ_DIR_WEST) {
      resetMotionState();
      homingActive = false;
      homingFailed = true;
      homingSearchDirection = AZ_DIR_NONE;
      Serial.println("AZ HOME FEHLER: WEST-Ende erreicht, Mitte nicht gefunden.");
      return;
    }

    Serial.println("AZ STOP: WEST-ENDSENSOR ERREICHT");
    resetMotionState();
    return;
  }

  // Wenn kein Laufmodus aktiv ist, gibt es nichts zu tun.
  if (runMode == AZ_RUN_IDLE) {
    return;
  }

  const unsigned long now = millis();

  // ---------------------------------------------------
  // ON-Phase läuft
  // ---------------------------------------------------
  if (outputOn) {
    if (now - phaseStartedAtMs >= currentOnTimeMs) {
      // Puls ist beendet -> Ausgang aus
      azimuthOutputsOff();
      outputOn = false;
      phaseStartedAtMs = now;

      stepCounter++;

      // Einzelpuls ist damit komplett beendet
      if (runMode == AZ_RUN_SINGLE_PULSE) {
        runMode = AZ_RUN_IDLE;
        currentDirection = AZ_DIR_NONE;
        currentOnTimeMs = 0;
        return;
      }

      // Beim Step-Modus ist danach Pausenphase
      if (firstSteppedPulsePending) {
        firstSteppedPulsePending = false;
      }
    }
    return;
  }

  // ---------------------------------------------------
  // OFF-/Pausenphase im Step-Modus
  // ---------------------------------------------------
  if (runMode == AZ_RUN_STEPPED && (now - phaseStartedAtMs >= AZ_STEP_PULSE_OFF_MS)) {
    // Vor jedem neuen Schritt nochmal Bewegungsfreigabe prüfen.

    if (currentDirection == AZ_DIR_EAST && !azimuthCanMoveEast()) {
      // Spezialfall Homing nach EAST:
      // auf WEST ausweichen, wenn möglich
      if (homingActive && homingSearchDirection == AZ_DIR_EAST) {
        if (startSteppedInternal(AZ_DIR_WEST)) {
          homingSearchDirection = AZ_DIR_WEST;
          Serial.println("AZ HOME: OST blockiert vor neuem Schritt, wechsle nach WEST.");
          return;
        }

        resetMotionState();
        homingActive = false;
        homingFailed = true;
        homingSearchDirection = AZ_DIR_NONE;
        Serial.println("AZ HOME FEHLER: Keine Ausweichrichtung mehr.");
        return;
      }

      Serial.println("AZ STOP VOR NEUEM SCHRITT: OST-ENDSENSOR AKTIV");
      resetMotionState();
      return;
    }

    if (currentDirection == AZ_DIR_WEST && !azimuthCanMoveWest()) {
      // Spezialfall Homing nach WEST:
      // direkt Fehler, wenn vor neuem Schritt blockiert
      if (homingActive && homingSearchDirection == AZ_DIR_WEST) {
        resetMotionState();
        homingActive = false;
        homingFailed = true;
        homingSearchDirection = AZ_DIR_NONE;
        Serial.println("AZ HOME FEHLER: WEST blockiert vor neuem Schritt.");
        return;
      }

      Serial.println("AZ STOP VOR NEUEM SCHRITT: WEST-ENDSENSOR AKTIV");
      resetMotionState();
      return;
    }

    // Nächsten Step starten
    outputOn = true;
    phaseStartedAtMs = now;
    currentOnTimeMs = AZ_STEP_PULSE_ON_MS;

    azimuthApplyDirection(currentDirection);
  }
}

// Liefert true, wenn gerade irgendein Azimutlauf aktiv ist.
bool azimuthIsPulseActive() {
  return runMode != AZ_RUN_IDLE;
}

// Liefert true, wenn der Ausgang aktuell eingeschaltet ist.
bool azimuthIsOutputOn() {
  return outputOn;
}

// Liefert true, wenn ein gestufter Run aktiv ist.
bool azimuthIsSteppedRunActive() {
  return runMode == AZ_RUN_STEPPED;
}

// Liefert die aktuelle Bewegungsrichtung.
AzimuthDirection azimuthGetDirection() {
  return currentDirection;
}

// Liefert die bisher gezählten Schritte / Pulse des aktuellen Laufs.
unsigned long azimuthGetStepCounter() {
  return stepCounter;
}

// Liefert die aktuelle Pulsdauer der laufenden ON-Phase.
unsigned long azimuthGetCurrentOnTimeMs() {
  return currentOnTimeMs;
}