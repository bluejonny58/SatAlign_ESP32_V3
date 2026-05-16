/*
  SatAlign V3 - MPU6050 Winkelmessung
  ------------------------------------------------------------
  Liest den MPU6050/GY-521 und berechnet den Winkel der Antenne. Der Sensor
  sitzt direkt an der Antenne; Offset und Filterung werden fuer den praktischen
  Aufbau kalibriert.
*/

/*
  SatAlign ESP32 - sensor_mpu6050.cpp
  ---------------------------------------------------------------------------
  Initialisiert und filtert den MPU6050 fuer die Elevationsanzeige.
  Der angezeigte Elevationswinkel entsteht aus dem gefilterten Sensorwinkel
  plus DISPLAY_ANGLE_OFFSET_DEG aus settings.cpp.
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "pins.h"
#include "config.h"
#include "sensor_mpu6050.h"

// I2C-Adresse des MPU6050
static const uint8_t MPU_ADDR = 0x68;

// Gewichtung des Komplementärfilters.
// Hoher Wert = Gyro dominiert stärker, Winkel wird ruhiger,
// aber Drift wird langsamer korrigiert.
// Kleinerer Wert = Beschleunigung wirkt stärker, schneller stabil,
// aber oft unruhiger.
static const float COMPLEMENTARY_ALPHA = 0.98f;

// Umrechnungsfaktor für Gyro-Rohdaten bei ±250°/s Bereich.
// Datenblattwert des MPU6050 für diesen Messbereich.
static const float GYRO_SCALE_250DPS = 131.0f;

namespace {
  // Aktueller gefilterter Winkel des Komplementärfilters.
  float filteredAngleDeg = 0.0f;

  // Letzter reiner Beschleunigungswinkel.
  float lastAccelAngleDeg = 0.0f;

  // Letzte berechnete Gyro-Drehgeschwindigkeit in °/s.
  float lastGyroRateDegPerSec = 0.0f;

  // Roh-Offset des Gyros nach Kalibrierung.
  // Dieser Wert wird bei der Gyro-Auswertung abgezogen.
  float gyroOffsetXRaw = 0.0f;

  // Aktuell gesetzter Referenzwinkel.
  // Relativwinkel = gefilterter Winkel - Referenzwinkel
  float referenceAngleDeg = 0.0f;

  // Zeitstempel der letzten Filteraktualisierung in Mikrosekunden.
  unsigned long lastUpdateMicros = 0;

  // Merkt, ob der Filter schon einen gültigen Startwert hat.
  bool filterInitialized = false;
}

// Schreibt ein einzelnes Register im MPU6050.
static bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

// Prüft, ob der MPU6050 grundsätzlich auf dem I2C-Bus antwortet.
bool mpuIsConnected() {
  Wire.beginTransmission(MPU_ADDR);
  return (Wire.endTransmission() == 0);
}

// Initialisiert den MPU6050.
//
// Ablauf:
// - I2C starten
// - Verfügbarkeit prüfen
// - Sleep-Modus deaktivieren
// - Messbereiche setzen
// - interne Zustände zurücksetzen
bool initMPU6050() {
  // I2C mit den in pins.h definierten Pins starten
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  delay(50);

  // Prüfen, ob der Sensor überhaupt antwortet
  if (!mpuIsConnected()) {
    return false;
  }

  // Sleep-Modus aus
  if (!writeRegister(0x6B, 0x00)) return false;

  // Beschleunigung auf ±2 g
  if (!writeRegister(0x1C, 0x00)) return false;

  // Gyro auf ±250 °/s
  if (!writeRegister(0x1B, 0x00)) return false;

  delay(50);

  // Interne Zustände zurücksetzen
  filterInitialized = false;
  lastUpdateMicros = micros();
  gyroOffsetXRaw = 0.0f;
  referenceAngleDeg = 0.0f;

  return true;
}

// Liest die Rohdaten für Beschleunigung und Gyro aus dem Sensor.
bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  // Startadresse des Datenblocks
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  // Restart ohne STOP, damit direkt gelesen werden kann
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  // 14 Byte lesen:
  // Accel XYZ (6), Temp (2), Gyro XYZ (6)
  const uint8_t bytesToRead = 14;
  uint8_t received = Wire.requestFrom((int)MPU_ADDR, (int)bytesToRead);

  if (received != bytesToRead) {
    return false;
  }

  // Beschleunigung
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  // Temperatur wird aktuell nicht verwendet -> überspringen
  Wire.read();
  Wire.read();

  // Gyro
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

// Berechnet aus den Beschleunigungswerten einen Elevationswinkel.
//
// Dieser Winkel ist grundsätzlich driftfrei, aber rauschanfälliger.
// Deshalb wird er im Komplementärfilter mit dem Gyro kombiniert.
float mpuGetAccelElevationAngleDeg(int16_t ax, int16_t ay, int16_t az) {
  float fax = (float)ax;
  float fay = (float)ay;
  float faz = (float)az;

  // Projektion für die Winkelberechnung
  float denominator = sqrtf((fay * fay) + (faz * faz));

  // Winkel in Grad
  float angleDeg = atan2f(fax, denominator) * 180.0f / PI;

  return angleDeg;
}

// Führt die Gyro-Nullpunktkalibrierung durch.
//
// Dabei wird der Sensor im Ruhezustand mehrfach gelesen
// und der Mittelwert von gx als Offset gespeichert.
bool mpuCalibrateGyro(uint16_t samples, uint16_t delayMs) {
  long sumGx = 0;

  for (uint16_t i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    if (!mpuReadRaw(ax, ay, az, gx, gy, gz)) {
      return false;
    }

    sumGx += gx;
    delay(delayMs);
  }

  gyroOffsetXRaw = (float)sumGx / (float)samples;
  return true;
}

// Liefert den zuletzt bestimmten Gyro-Offset.
float mpuGetGyroOffsetXRaw() {
  return gyroOffsetXRaw;
}

// Setzt den Filter auf einen definierten Startwinkel zurück.
void mpuResetFilter(float initialAngleDeg) {
  filteredAngleDeg = initialAngleDeg;
  lastAccelAngleDeg = initialAngleDeg;
  lastGyroRateDegPerSec = 0.0f;
  lastUpdateMicros = micros();
  filterInitialized = true;
}

// Führt einen Komplementärfilter-Update-Schritt aus.
bool mpuUpdateFilteredAngle() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Neue Rohdaten lesen
  if (!mpuReadRaw(ax, ay, az, gx, gy, gz)) {
    return false;
  }

  // Reinen Beschleunigungswinkel berechnen
  float accelAngleDeg = mpuGetAccelElevationAngleDeg(ax, ay, az);

  // Zeitdifferenz seit letztem Update in Sekunden
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastUpdateMicros) / 1000000.0f;
  lastUpdateMicros = nowMicros;

  // Gyro-Rohwert in °/s umrechnen und Offset abziehen
  float gyroRateDegPerSec = ((float)gx - gyroOffsetXRaw) / GYRO_SCALE_250DPS;

  // Erstinitialisierung:
  // Wenn der Filter noch keinen Startwert hat,
  // wird direkt mit dem Beschleunigungswinkel begonnen.
  if (!filterInitialized) {
    filteredAngleDeg = accelAngleDeg;
    filterInitialized = true;
  } else {
    // Komplementärfilter:
    // - Gyro integriert kurzfristige Bewegung sauber
    // - Beschleunigung korrigiert langfristig Drift
    filteredAngleDeg =
        (COMPLEMENTARY_ALPHA * (filteredAngleDeg + gyroRateDegPerSec * dt)) +
        ((1.0f - COMPLEMENTARY_ALPHA) * accelAngleDeg);
  }

  // Letzte Teilwerte für Diagnose / Debug merken
  lastAccelAngleDeg = accelAngleDeg;
  lastGyroRateDegPerSec = gyroRateDegPerSec;

  return true;
}

// Liefert den aktuellen gefilterten Winkel.
float mpuGetFilteredAngleDeg() {
  return filteredAngleDeg;
}

// Liefert den für Anzeigezwecke korrigierten Winkel.
//
// Hier wird zusätzlich DISPLAY_ANGLE_OFFSET_DEG addiert,
// damit kleine systematische Anzeigenfehler korrigiert werden können.
float mpuGetDisplayedAngleDeg() {
  return 90.0f - filteredAngleDeg + DISPLAY_ANGLE_OFFSET_DEG;
}

// Liefert den zuletzt berechneten reinen Beschleunigungswinkel.
float mpuGetLastAccelAngleDeg() {
  return lastAccelAngleDeg;
}

// Liefert die zuletzt berechnete Gyro-Drehgeschwindigkeit.
float mpuGetLastGyroRateDegPerSec() {
  return lastGyroRateDegPerSec;
}

// Setzt den Referenzwinkel explizit von außen.
void mpuSetReferenceAngleDeg(float angleDeg) {
  referenceAngleDeg = angleDeg;
}

// Setzt den aktuell gefilterten Winkel als neue Referenz.
//
// Praktisch nützlich beim Start oder nach einer bewussten Nullsetzung.
void mpuSetCurrentAngleAsReference() {
  referenceAngleDeg = filteredAngleDeg;
}

// Liefert den aktuell gesetzten Referenzwinkel.
float mpuGetReferenceAngleDeg() {
  return referenceAngleDeg;
}

// Liefert den relativen Winkel gegenüber der aktuellen Referenz.
float mpuGetRelativeAngleDeg() {
  return filteredAngleDeg - referenceAngleDeg;
}