/*
  SatAlign V3 - Pinbelegung
  ------------------------------------------------------------
  Diese Datei ist die verbindliche Hardware-Zuordnung fuer ESP32, TFT,
  MPU6050, RF-Detector, Hall-Sensoren und Motoransteuerung.
  Aenderungen an Pins immer hier zentral dokumentieren.

  V3_01 / PCB-Hinweis:
  Ab dieser Version ist die KiCad-/Leiterplatten-Pinbelegung die aktive
  Entwicklungsgrundlage. Die fruehere Breadboard-/Testbelegung bleibt nur
  noch als Dokumentation/Referenz erhalten und soll nicht mehr als aktive
  Codebasis verwendet werden.

  Wichtig:
  Einige Pins wurden bewusst gegenueber der alten Testbelegung geaendert,
  damit das Leiterplattenrouting sauberer und mechanisch sinnvoller wird.
*/

#pragma once

// =====================================================
// AD8317 / AD8318 ADC
// =====================================================
static const int PIN_AD8318_ADC = 35;

// =====================================================
// I2C / MPU6050
// =====================================================
// PCB-V3:
// SDA bleibt auf GPIO32.
// SCL wurde bewusst von GPIO21 auf GPIO26 gelegt.
static const int PIN_I2C_SDA = 32;
static const int PIN_I2C_SCL = 26;

// =====================================================
// Azimut ueber Optokoppler / SatAlign ESP32 Tastensimulation
// =====================================================
// Diese Pins simulieren ueber Optokoppler EAST/WEST am DiSEqC-Motor.
// Es werden keine Leistungs-Motoren direkt vom ESP32 angesteuert.
static const int PIN_AZ_EAST = 22;
static const int PIN_AZ_WEST = 17;

// =====================================================
// Hall-Sensoren Azimut
// =====================================================
// GPIO34, GPIO36 und GPIO39 sind reine Eingangs-Pins.
// Externe Pull-ups sind deshalb weiterhin erforderlich.
static const int PIN_AZ_HALL_CENTER     = 39;
static const int PIN_AZ_HALL_EAST_LIMIT = 34;
static const int PIN_AZ_HALL_WEST_LIMIT = 36;

// =====================================================
// Elevation ueber L298N
// =====================================================
// PCB-V3:
// IN1 wurde bewusst von GPIO26 auf GPIO21 gelegt,
// weil GPIO26 jetzt fuer MPU-SCL verwendet wird.
static const int PIN_EL_IN1 = 21;
static const int PIN_EL_IN2 = 27;
static const int PIN_EL_ENA = 25;

// =====================================================
// TFT ST7735
// =====================================================
// PCB-V3:
// TFT_CS wurde bewusst von GPIO14 auf GPIO5 gelegt.
// MOSI/SCK/DC bleiben unveraendert.
//
// Display-Beschriftung:
// - SDA entspricht hier MOSI
// - A0 entspricht hier DC
// - RST liegt direkt am ESP32-Reset und hat deshalb keinen GPIO.
static const int PIN_TFT_MOSI = 19;
static const int PIN_TFT_SCK  = 18;
static const int PIN_TFT_CS   = 5;
static const int PIN_TFT_DC   = 23;

// =====================================================
// 3-Taster-Bedienung
// =====================================================
// Gruen   = MODE
// Weiss   = MINUS
// Schwarz = PLUS
//
// Fuer die Taster sind externe 10k Pull-ups vorgesehen.
static const int PIN_BTN_MODE  = 13;
static const int PIN_BTN_MINUS = 16;
static const int PIN_BTN_PLUS  = 33;