/*
  SatAlign V3 - RF-Detector
  ------------------------------------------------------------
  Liest den AD8317/AD8318-Ausgang am ESP32-ADC. In diesem Aufbau gilt:
  kleinerer ADC-/Spannungswert bedeutet staerkeres RF-Signal.
*/

/*
  SatAlign ESP32 - rf_detector.cpp
  ---------------------------------------------------------------------------
  Liest den AD8317/AD8318 RF-Detektor am ESP32-ADC ein und stellt Rohwert,
  gefilterten ADC-Wert, Spannung und einfache Signalmetriken bereit.

  Im aktuellen Aufbau gilt: niedrigere Spannung / niedrigerer ADC-Wert bedeutet
  staerkeres Satellitensignal.
*/
#include <Arduino.h>

#include "pins.h"
#include "config.h"
#include "settings.h"   // V3_01: RF-Referenz- und Drop-Schwellwerte liegen zentral in settings.cpp.
#include "rf_detector.h"

namespace {
  // Referenzspannung für die Umrechnung ADC -> Volt.
  // Im aktuellen Projekt wird von 3.3 V ADC-Referenz ausgegangen.
  static const float RF_ADC_REFERENCE_V = 3.3f;

  // Maximalwert des ADC bei 12 Bit Auflösung.
  // 12 Bit bedeutet: 0 ... 4095
  static const int   RF_ADC_MAX_VALUE   = 4095;

  // Merkt, ob der RF-Detektor initialisiert wurde.
  // Vor initRFDetector() sollen keine RF-Werte als gültig gelten.
  bool initialized = false;

  // Letzter gemessener ungefilterter ADC-Rohwert
  uint16_t rawAdc = 0;

  // Letzter gefilterter ADC-Wert.
  // Wird als float geführt, damit die Filterung sauber mit
  // Zwischenwerten arbeiten kann.
  float filteredAdc = 0.0f;

  // Merkt, ob der Filter bereits einen ersten gültigen Startwert hat.
  // Beim allerersten Zyklus wird filteredAdc direkt auf rawAdc gesetzt,
  // damit der Filter nicht von 0.0 aus anlaufen muss.
  bool filterInitialized = false;

  // Kleinster bisher beobachteter Rohwert seit dem letzten Reset.
  // Nützlich für Diagnose / Spannungsbereich / Testläufe.
  uint16_t minRawAdc = 4095;

  // Größter bisher beobachteter Rohwert seit dem letzten Reset.
  uint16_t maxRawAdc = 0;

  // RAM-Baseline fuer die laufende Session.
  // Wird bewusst nicht in Preferences/NVS gespeichert.
  float noSignalBaselineAdc = 1883.0f;
  float noSignalMinAdc = 1883.0f;
  float noSignalMaxAdc = 1883.0f;
  bool baselineInitialized = false;

  // V3_01:
  // Die RF-Referenzpunkte und der Mindest-DROP liegen jetzt zentral in
  // settings.cpp/settings.h. Dadurch koennen sie nach Aussentests manuell
  // angepasst werden, ohne diese RF-Auswertelogik selbst zu veraendern.
  // Verwendete Variablen:
  // - RF_WEAK_REFERENCE_ADC
  // - RF_STRONG_REFERENCE_ADC
  // - RF_VALID_DROP_ADC
}

// Liest einen ADC-Pin mehrfach ein und bildet daraus den Mittelwert.
//
// Zweck:
// - Signal beruhigen
// - Rauschen reduzieren
// - bessere Basis für die spätere Filterung
//
// samples < 1 wird automatisch auf 1 korrigiert.
static uint16_t readAdcAverage(int pin, int samples) {
  if (samples < 1) {
    samples = 1;
  }

  uint32_t sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);

    // Kleine Pause zwischen den Einzelsamples.
    // Hilft, mehrere leicht unterschiedliche ADC-Werte statt identischer
    // Direktaufrufe zu bekommen.
    delayMicroseconds(300);
  }

  return (uint16_t)(sum / samples);
}

// Rechnet einen ADC-Wert (0 ... 4095) in eine Spannung um.
static float adcToVoltage(float adcValue) {
  return (adcValue / (float)RF_ADC_MAX_VALUE) * RF_ADC_REFERENCE_V;
}

// Initialisiert den RF-Detektor.
//
// Aufgaben:
// - ADC-Pin vorbereiten
// - ADC auf 12 Bit setzen
// - Attenuation passend einstellen
// - interne Laufvariablen zurücksetzen
void initRFDetector() {
  // ADC-Pin explizit als Eingang setzen
  pinMode(PIN_AD8318_ADC, INPUT);

  // 12 Bit ADC-Auflösung
  analogReadResolution(12);

  // Attenuation 11 dB:
  // sinnvoll, um den Messbereich für Spannungen im typischen AD8318-Ausgangs-
  // fenster besser zu nutzen.
  analogSetPinAttenuation(PIN_AD8318_ADC, ADC_11db);

  // Interne Zustände zurücksetzen
  rawAdc = 0;
  filteredAdc = 0.0f;

  // V3_01:
  // Auch die Default-Baseline folgt jetzt dem zentralen RF-Referenzwert
  // aus settings.cpp. Solange noch kein echtes Zero/No-Signal-Fenster
  // gemessen wurde, verwendet die RF-Logik damit denselben manuellen
  // Einstellwert wie die Prozent- und DROP-Auswertung.
  noSignalBaselineAdc = RF_WEAK_REFERENCE_ADC;
  noSignalMinAdc = RF_WEAK_REFERENCE_ADC;
  noSignalMaxAdc = RF_WEAK_REFERENCE_ADC;
  baselineInitialized = false;
  filterInitialized = false;
  minRawAdc = 4095;
  maxRawAdc = 0;
  initialized = true;

  Serial.println("RF-Detektor initialisiert.");
}

// Führt eine neue RF-Messung durch.
//
// Ablauf:
// 1. Mehrfachmessung mit Mittelwertbildung
// 2. Initialisierung oder Aktualisierung des Filters
// 3. Min/Max-Verlauf mitführen
void rfUpdate() {
  // Ohne Initialisierung keine Messung durchführen
  if (!initialized) {
    return;
  }

  // Neuen Rohwert aus gemittelten ADC-Samples bestimmen
  rawAdc = readAdcAverage(PIN_AD8318_ADC, RF_ADC_SAMPLES_PER_CYCLE);

  // Filterstart:
  // Beim ersten Zyklus den Filter direkt auf den Rohwert setzen,
  // damit kein künstlicher Einschwingvorgang von 0 aus entsteht.
  if (!filterInitialized) {
    filteredAdc = (float)rawAdc;
    filterInitialized = true;
  } else {
    // Exponentielle Glättung / IIR-Filter:
    // filtered_new = alpha * filtered_old + (1-alpha) * raw
    //
    // Großer alpha-Wert:
    // ruhiger, aber träger
    //
    // Kleiner alpha-Wert:
    // schneller, aber nervöser
    filteredAdc =
      (RF_FILTER_ALPHA * filteredAdc) +
      ((1.0f - RF_FILTER_ALPHA) * (float)rawAdc);
  }

  // Verlaufs-Minimum aktualisieren
  if (rawAdc < minRawAdc) minRawAdc = rawAdc;

  // Verlaufs-Maximum aktualisieren
  if (rawAdc > maxRawAdc) maxRawAdc = rawAdc;
}

// Liefert, ob der RF-Detektor betriebsbereit ist.
bool rfIsReady() {
  return initialized;
}

// Liefert den zuletzt gemessenen ungefilterten ADC-Rohwert.
uint16_t rfGetRawAdc() {
  return rawAdc;
}

// Liefert den zuletzt gefilterten ADC-Wert.
float rfGetFilteredAdc() {
  return filteredAdc;
}

// Liefert die aktuelle, aus dem gefilterten ADC-Wert berechnete Spannung.
//
// Diese Spannung ist im Projekt aktuell die Basis für:
// - Displayanzeige
// - RF-Bewertung
// - Peak-Suche
float rfGetPinVoltage() {
  return adcToVoltage(filteredAdc);
}

// Liefert den kleinsten Rohwert seit letztem Reset.
uint16_t rfGetMinRawAdc() {
  return minRawAdc;
}

// Liefert den größten Rohwert seit letztem Reset.
uint16_t rfGetMaxRawAdc() {
  return maxRawAdc;
}

// Setzt die Verlaufsmessung für Min/Max zurück.
//
// Sinnvoll z. B. vor:
// - neuem Testlauf
// - neuem Suchvorgang
// - gezielter Diagnose
void rfResetMinMax() {
  minRawAdc = 4095;
  maxRawAdc = 0;
}

// Misst ein aktuelles Rausch-/Baselinefenster fuer die laufende Session.
//
// WICHTIG:
// Diese Werte werden absichtlich nur im RAM gehalten. Die No-NVS-Version
// speichert keine RF-Kalibrierung dauerhaft im Flash.
void rfCalibrateNoSignalBaseline(int sampleBlocks, int delayMs) {
  if (!initialized) {
    return;
  }

  if (sampleBlocks < 1) {
    sampleBlocks = 1;
  }

  uint32_t sum = 0;
  uint16_t localMin = 4095;
  uint16_t localMax = 0;

  for (int i = 0; i < sampleBlocks; i++) {
    const uint16_t v = readAdcAverage(PIN_AD8318_ADC, RF_ADC_SAMPLES_PER_CYCLE);
    sum += v;
    if (v < localMin) localMin = v;
    if (v > localMax) localMax = v;
    if (delayMs > 0) delay(delayMs);
  }

  noSignalBaselineAdc = (float)sum / (float)sampleBlocks;
  noSignalMinAdc = (float)localMin;
  noSignalMaxAdc = (float)localMax;
  baselineInitialized = true;

  // Den laufenden Filter mit der Baseline starten, damit die Anzeige nach
  // dem Boot nicht von 0 hochlaufen muss.
  rawAdc = (uint16_t)noSignalBaselineAdc;
  filteredAdc = noSignalBaselineAdc;
  filterInitialized = true;

  Serial.print("RF-Baseline gesetzt | ADC=");
  Serial.print(noSignalBaselineAdc, 1);
  Serial.print(" | MIN=");
  Serial.print(noSignalMinAdc, 1);
  Serial.print(" | MAX=");
  Serial.println(noSignalMaxAdc, 1);
}



// Misst ein RF-Fenster, ohne die Baseline automatisch zu veraendern.
// Rueckgabe true bedeutet: Mittelwert, Minimum und Maximum sind gueltig.
bool rfMeasureAdcWindow(int sampleBlocks, int delayMs, float* meanAdc, float* minAdc, float* maxAdc) {
  if (!initialized) {
    return false;
  }

  if (sampleBlocks < 1) {
    sampleBlocks = 1;
  }

  uint32_t sum = 0;
  uint16_t localMin = 4095;
  uint16_t localMax = 0;

  for (int i = 0; i < sampleBlocks; i++) {
    const uint16_t v = readAdcAverage(PIN_AD8318_ADC, RF_ADC_SAMPLES_PER_CYCLE);
    sum += v;
    if (v < localMin) localMin = v;
    if (v > localMax) localMax = v;
    if (delayMs > 0) delay(delayMs);
  }

  const float localMean = (float)sum / (float)sampleBlocks;

  if (meanAdc) *meanAdc = localMean;
  if (minAdc)  *minAdc = (float)localMin;
  if (maxAdc)  *maxAdc = (float)localMax;

  return true;
}

// Setzt die No-Signal-Baseline gezielt aus einem vorher gemessenen Fenster.
// Diese Funktion schreibt bewusst nichts in NVS/Flash.
void rfSetNoSignalBaselineAdc(float meanAdc, float minAdc, float maxAdc) {
  noSignalBaselineAdc = meanAdc;
  noSignalMinAdc = minAdc;
  noSignalMaxAdc = maxAdc;
  baselineInitialized = true;

  rawAdc = (uint16_t)noSignalBaselineAdc;
  filteredAdc = noSignalBaselineAdc;
  filterInitialized = true;

  Serial.print("RF-Zero gesetzt | ADC=");
  Serial.print(noSignalBaselineAdc, 1);
  Serial.print(" | MIN=");
  Serial.print(noSignalMinAdc, 1);
  Serial.print(" | MAX=");
  Serial.println(noSignalMaxAdc, 1);
}

bool rfHasNoSignalBaseline() {
  return baselineInitialized;
}

// Differenz zwischen Baseline und aktuellem ADC-Wert.
// Beim AD8317/AD8318 in deinem Aufbau gilt: staerkeres Signal -> kleinerer ADC.
float rfGetDropAdc() {
  const float baseline = baselineInitialized ? noSignalBaselineAdc : RF_WEAK_REFERENCE_ADC;
  return baseline - filteredAdc;
}

// Normiert das Signal anhand der bekannten Test-Referenzen.
// 0.0 = schwach/kein Signal, 1.0 = starkes Signal.
float rfGetSignalNorm() {
  const float span = RF_WEAK_REFERENCE_ADC - RF_STRONG_REFERENCE_ADC;
  if (span <= 1.0f) {
    return 0.0f;
  }

  float norm = (RF_WEAK_REFERENCE_ADC - filteredAdc) / span;
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;
  return norm;
}

float rfGetSignalPercent() {
  return rfGetSignalNorm() * 100.0f;
}

bool rfHasValidSignalDrop() {
  if (!initialized || !filterInitialized) {
    return false;
  }
  return rfGetDropAdc() >= RF_VALID_DROP_ADC;
}
