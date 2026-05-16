/*
  SatAlign V3 - Winkel-/Elevationssteuerung
  ------------------------------------------------------------
  Kapselt die Bewegung des Linearantriebs fuer den Winkel. Softwarelimits,
  Schritt-/Pulsfahrten und Sonderfaelle werden hier zentral behandelt.
*/

/*
  SatAlign ESP32 - elevation_control.cpp
  ---------------------------------------------------------------------------
  Kapselt die Elevationsachse am L298N: Rohfahrt hoch/runter, Pulse, Stop
  und einfache Ziel-/Pulsverwaltung.

  Die manuelle Bedienung darf als Override fahren. Mechanische Endabschalter
  des Linearantriebs bleiben die letzte Schutzebene.
*/
#include <Arduino.h>
#include <math.h>

#include "pins.h"
#include "config.h"
#include "elevation_control.h"

namespace {
  // Startzeit des aktuell laufenden Pulses
  unsigned long pulseStartTime = 0;

  // Dauer des aktuell laufenden Pulses in Millisekunden
  unsigned long currentPulseDurationMs = 0;

  // true = es läuft gerade ein zeitgesteuerter Elevationspuls
  bool pulseActive = false;

  // Zuletzt verwendeter PWM-Wert.
  // Praktisch vor allem für Diagnose / Nachvollziehbarkeit.
  int activePwm = 0;

  // Aktuelle semantische Bewegungsrichtung:
  // EL_DIR_UP / EL_DIR_DOWN / EL_DIR_STOP
  ElevationDirection currentDirection = EL_DIR_STOP;
}

// Entspricht exakt der in deinem funktionierenden Test-Sketch bewiesenen Rohlogik.
//
// Wirkung:
// - beide Richtungsleitungen LOW
// - PWM/Enable = 0
//
// Das ist der softwareseitig stromlose Stop-Zustand der Elevationsachse.
void elevationStopRaw() {
  digitalWrite(PIN_EL_IN1, LOW);
  digitalWrite(PIN_EL_IN2, LOW);
  analogWrite(PIN_EL_ENA, 0);
}

// Rohbewegung in eine elektrische Richtung.
//
// WICHTIG:
// Diese Funktion bedeutet NICHT automatisch "angezeigter Winkel wird größer".
// Die semantische Zuordnung erfolgt weiter unten in applySemanticDirection().
void elevationUpRaw(int pwm) {
  digitalWrite(PIN_EL_IN1, HIGH);
  digitalWrite(PIN_EL_IN2, LOW);
  analogWrite(PIN_EL_ENA, pwm);
}

// Rohbewegung in die entgegengesetzte elektrische Richtung.
//
// Auch hier gilt:
// Raw-Richtung != automatisch semantische Höhenrichtung.
void elevationDownRaw(int pwm) {
  digitalWrite(PIN_EL_IN1, LOW);
  digitalWrite(PIN_EL_IN2, HIGH);
  analogWrite(PIN_EL_ENA, pwm);
}

// Setzt eine semantische Bewegungsrichtung auf die reale Roh-Ansteuerung um.
//
// WICHTIG:
// In deinem funktionierenden Testaufbau war die Mechanik gegenüber den
// Rohfunktionen invertiert:
//
// manualDriveUp()   -> elevationDownRaw(...)
// manualDriveDown() -> elevationUpRaw(...)
//
// Deshalb bleibt diese bewiesene Zuordnung hier erhalten.
// Das ist wichtig, damit die späteren Funktionen "hoch" und "runter"
// fachlich korrekt bleiben, auch wenn die elektrische Richtung invertiert ist.
static void applySemanticDirection(ElevationDirection direction, int pwm) {
  if (direction == EL_DIR_STOP) {
    elevationStopRaw();
    currentDirection = EL_DIR_STOP;
    activePwm = 0;
    return;
  }

  // Semantisch "hoch":
  // relativer Winkel soll größer werden
  if (direction == EL_DIR_UP) {
    elevationDownRaw(pwm);
    currentDirection = EL_DIR_UP;
    activePwm = pwm;
    return;
  }

  // Semantisch "runter":
  // relativer Winkel soll kleiner werden
  elevationUpRaw(pwm);
  currentDirection = EL_DIR_DOWN;
  activePwm = pwm;
}

// Initialisiert die Elevations-Hardware.
//
// Aufgaben:
// - Richtungsleitungen als Ausgänge setzen
// - PWM/Enable-Pin als Ausgang setzen
// - sicheren Stop-Zustand herstellen
void initElevation() {
  pinMode(PIN_EL_IN1, OUTPUT);
  pinMode(PIN_EL_IN2, OUTPUT);
  pinMode(PIN_EL_ENA, OUTPUT);
  elevationStop();
}

// Startet eine kontinuierliche semantische Bewegung nach oben.
//
// WICHTIG:
// Diese Funktion beendet einen evtl. aktiven Pulsmodus und
// schaltet in eine direkte Dauerbewegung um.
void elevationUp() {
  pulseActive = false;
  currentPulseDurationMs = 0;
  applySemanticDirection(EL_DIR_UP, EL_PWM_FAST);
}

// Startet eine kontinuierliche semantische Bewegung nach unten.
void elevationDown() {
  pulseActive = false;
  currentPulseDurationMs = 0;
  applySemanticDirection(EL_DIR_DOWN, EL_PWM_FAST);
}

// Stoppt die Elevation vollständig.
void elevationStop() {
  pulseActive = false;
  currentPulseDurationMs = 0;
  applySemanticDirection(EL_DIR_STOP, 0);
}

// Startet einen zeitlich begrenzten Puls nach oben.
//
// Die Bewegung endet nicht in dieser Funktion,
// sondern später über elevationUpdate().
void elevationPulseUp(unsigned long pulseMs, int pwm) {
  pulseStartTime = millis();
  currentPulseDurationMs = pulseMs;
  pulseActive = true;
  applySemanticDirection(EL_DIR_UP, pwm);
}

// Startet einen zeitlich begrenzten Puls nach unten.
void elevationPulseDown(unsigned long pulseMs, int pwm) {
  pulseStartTime = millis();
  currentPulseDurationMs = pulseMs;
  pulseActive = true;
  applySemanticDirection(EL_DIR_DOWN, pwm);
}

// Muss zyklisch aufgerufen werden, damit zeitgesteuerte Pulse sauber enden.
//
// Wenn die Pulsdauer abgelaufen ist, wird automatisch gestoppt.
void elevationUpdate() {
  if (!pulseActive) {
    return;
  }

  if (millis() - pulseStartTime >= currentPulseDurationMs) {
    elevationStop();
  }
}

// Liefert true, wenn aktuell ein zeitgesteuerter Puls aktiv ist.
bool elevationIsPulseActive() {
  return pulseActive;
}

// Prüft, ob ein gegebener Winkel innerhalb der softwareseitigen Softlimits liegt.
//
// WICHTIG:
// Das ist nur die normale Arbeitsgrenze der Software.
// Der Linearantrieb hat zusätzlich noch interne Endschalter als
// mechanische letzte Sicherheitsstufe.
bool elevationWithinSoftLimits(float currentAngle) {
  return (currentAngle >= ELEVATION_MIN_SOFT &&
          currentAngle <= ELEVATION_MAX_SOFT);
}

// Liefert die aktuelle semantische Bewegungsrichtung.
ElevationDirection elevationGetDirection() {
  return currentDirection;
}

// Interne Hilfsfunktion für die Zielregelung.
//
// Ablauf:
// 1. Fehler zum Ziel berechnen
// 2. prüfen, ob aktiver Puls noch läuft
// 3. prüfen, ob Ziel schon innerhalb der Toleranz liegt
// 4. Softlimits absichern
// 5. je nach Fehlergröße langsamen oder schnellen Puls auslösen
//
// Rückgabewert:
// - true  = Ziel erreicht
// - false = Ziel noch nicht erreicht / Puls läuft noch / Softlimit blockiert
static bool moveToPulseInternal(float currentAngleDeg,
                                float targetAngleDeg,
                                float toleranceDeg,
                                bool respectSoftLimits) {
  const float errorDeg = targetAngleDeg - currentAngleDeg;
  const float absErrorDeg = fabsf(errorDeg);

  // Solange ein Puls noch aktiv ist, wird nur dessen Ende abgewartet.
  if (pulseActive) {
    elevationUpdate();
    return false;
  }

  // Ziel liegt innerhalb der Toleranz -> fertig
  if (absErrorDeg <= toleranceDeg) {
    elevationStop();
    return true;
  }

  // V3: Softlimit-Pruefung nur dann anwenden, wenn der Aufrufer sie
  // ausdruecklich verlangt. Das bleibt der Normalfall fuer AUTO-/Suchlogik.
  //
  // Fuer die erste Standard-EZ-Anfahrt nach dem Einschalten gibt es bewusst
  // einen separaten Aufruf ohne diese Softwarelimits. Grund: Zu diesem fruehen
  // Zeitpunkt kann der MPU-/Filterwert noch grob oder unplausibel sein. Die
  // Elevation soll trotzdem auf den zentralen Standardwert fahren duerfen.
  // Die internen Endabschalter des Linearantriebs bleiben dabei weiterhin die
  // mechanische Schutzebene; es werden nur die softwareseitigen Winkelgrenzen
  // dieser Zielregelung ignoriert.
  if (respectSoftLimits) {
    // Unteres Softlimit würde verletzt -> abbrechen
    if (currentAngleDeg <= ELEVATION_MIN_SOFT && errorDeg < 0.0f) {
      elevationStop();
      return false;
    }

    // Oberes Softlimit würde verletzt -> abbrechen
    if (currentAngleDeg >= ELEVATION_MAX_SOFT && errorDeg > 0.0f) {
      elevationStop();
      return false;
    }
  }

  // Je näher am Ziel, desto feiner regeln:
  // innerhalb des Slow-Bands werden kürzere/langsamere Pulse verwendet.
  const bool useSlowMode = (absErrorDeg <= ELEVATION_SLOW_BAND_DEG);
  const unsigned long pulseMs = useSlowMode ? EL_PULSE_SLOW_MS : EL_PULSE_FAST_MS;
  const int pwm = useSlowMode ? EL_PWM_SLOW : EL_PWM_FAST;

  // Fehler > 0:
  // Ziel liegt "über" der aktuellen Lage -> semantisch hoch fahren
  if (errorDeg > 0.0f) {
    elevationPulseUp(pulseMs, pwm);
  } else {
    // Fehler < 0:
    // Ziel liegt "unter" der aktuellen Lage -> semantisch runter fahren
    elevationPulseDown(pulseMs, pwm);
  }

  return false;
}

// Öffentliche Zielregelung mit "absolutem/anzeigeorientiertem" Namen.
//
// Im aktuellen Projekt ist die Logik identisch zur relativen Variante.
bool moveToElevationPulse(float currentAngleDeg,
                          float targetAngleDeg,
                          float toleranceDeg) {
  return moveToPulseInternal(currentAngleDeg, targetAngleDeg, toleranceDeg, true);
}



// V3: Sondervariante fuer die erste Standard-Elevation beim Einschalten.
//
// Diese Funktion verwendet dieselbe Pulsregelung wie die normale Zielanfahrt,
// ignoriert dabei aber bewusst die softwareseitigen Elevations-Softlimits.
// Das ist nur fuer den Boot-Start gedacht: Die Anlage soll zuerst grob auf den
// zentralen Standardwinkel fahren duerfen, auch wenn der fruehe Sensor-/
// Filterwert noch nicht perfekt plausibel ist.
//
// Wichtig: Es werden nur die Software-Winkelgrenzen dieser Regelung ignoriert.
// Die mechanischen Endabschalter des Linearantriebs bleiben davon unabhaengig
// als harte Schutzebene erhalten. Fuer normale Suche-/AUTO-Bewegungen bleibt
// moveToElevationPulse(...) bzw. moveToRelativeElevationPulse(...) mit
// Softlimit-Pruefung der richtige Standard.
bool moveToElevationPulseNoSoftLimits(float currentAngleDeg,
                                      float targetAngleDeg,
                                      float toleranceDeg) {
  return moveToPulseInternal(currentAngleDeg, targetAngleDeg, toleranceDeg, false);
}

// Kompatibilitätsname für relative Zielregelung.
//
// Im aktuellen Projekt ebenfalls dieselbe interne Logik.
bool moveToRelativeElevationPulse(float currentAngleDeg,
                                  float targetAngleDeg,
                                  float toleranceDeg) {
  return moveToPulseInternal(currentAngleDeg, targetAngleDeg, toleranceDeg, true);
}