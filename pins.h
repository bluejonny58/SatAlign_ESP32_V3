/*
  SatAlign V3 - Pinbelegung
  ------------------------------------------------------------
  Diese Datei ist die verbindliche Hardware-Zuordnung fuer ESP32, TFT,
  MPU6050, RF-Detector, Hall-Sensoren und Motoransteuerung.
  Aenderungen an Pins immer hier zentral dokumentieren.
*/

#pragma once

// =====================================================
// AD8317 / AD8318 ADC
// =====================================================
// Analog-Eingang für den RF-Detektor.
// Der AD8317/AD8318 liefert eine analoge Spannung,
// die im Projekt zur Signalbewertung verwendet wird.
//
// WICHTIG:
// Dieser Pin ist ein reiner ADC-Eingang und wird nicht als
// digitaler Ausgang verwendet.
static const int PIN_AD8318_ADC = 35;

// =====================================================
// I2C / MPU6050
// =====================================================
// I2C-Datenleitung für den MPU6050
static const int PIN_I2C_SDA = 32;

// I2C-Taktleitung für den MPU6050
static const int PIN_I2C_SCL = 21;

// =====================================================
// Azimut über Optokoppler / SatAlign ESP32 Tastensimulation
// =====================================================
// Diese beiden Pins simulieren über Optokoppler die Tasten
// EAST und WEST am Azimut-/DiSEqC-Motor.
//
// WICHTIG:
// Hier wird nicht direkt ein Leistungs-Motor angesteuert,
// sondern nur ein Steuersignal über den Optokoppler gegeben.
static const int PIN_AZ_EAST = 22;
static const int PIN_AZ_WEST = 17;

// =====================================================
// Hall-Sensoren Azimut
// =====================================================
// Drei Hall-Sensoren für:
// - Mitte / Südrichtung
// - Ost-Endanschlag
// - West-Endanschlag
//
// Diese Eingänge werden von der Azimut-Logik verwendet für:
// - Referenzfahrt
// - Endanschlag-Schutz
// - Mittenerkennung
//
// HINWEIS:
// Aktuell ist die Software auf active-low vorbereitet.
// Je nach Verdrahtung / Pull-up-Konzept kann das in den Settings
// angepasst werden.
static const int PIN_AZ_HALL_CENTER     = 39;
static const int PIN_AZ_HALL_EAST_LIMIT = 34;
static const int PIN_AZ_HALL_WEST_LIMIT = 36;

// =====================================================
// Elevation über L298N
// =====================================================
// Ansteuerung des 12-V-Linearantriebs über einen L298N.
//
// IN1 / IN2 legen die Richtung fest.
// ENA bekommt ein PWM-Signal zur Freigabe bzw. Geschwindigkeitsvorgabe.
//
// WICHTIG:
// Die tatsächliche mechanische Bedeutung von "hoch" und "runter"
// ist im Code bewusst semantisch umgesetzt, da die Roh-Richtung
// im funktionierenden Testaufbau invertiert war.
static const int PIN_EL_IN1 = 26;
static const int PIN_EL_IN2 = 27;
static const int PIN_EL_ENA = 25;

// =====================================================
// TFT ST7735
// =====================================================
// SPI-Pins für das kleine ST7735-TFT.
//
// Beschriftung am Displaymodul / in vielen ST7735-Anleitungen:
// - SDA entspricht hier der SPI-Datenleitung MOSI
// - A0 entspricht hier der Steuerleitung DC
//
// Im Code bleiben die bestehenden Variablennamen bewusst unverändert,
// weil die verwendete Adafruit-ST7735-Bibliothek technisch mit MOSI/DC arbeitet.
//
// WICHTIG:
// TFT RST ist hier absichtlich nicht als GPIO definiert,
// weil das Display direkt mit dem ESP32-Reset verbunden ist.
static const int PIN_TFT_MOSI = 19;
static const int PIN_TFT_SCK  = 18;
static const int PIN_TFT_CS   = 14;
static const int PIN_TFT_DC   = 23;

// TFT RST direkt an ESP32 RST -> kein GPIO-Pin hier

// =====================================================
// 3-Taster-Bedienung
// =====================================================
// Grün   = MODE
// Weiß   = MINUS
// Schwarz= PLUS
//
// Bedienkonzept im Hauptmenue:
// - PLUS/MINUS -> Menuepunkt 1-3 waehlen
// - MODE kurz  -> gewaehlten Menuepunkt starten
//
// Bedienkonzept im manuellen Modus:
// - MODE kurz  -> AZIMUT / ELEVATION wechseln
// - PLUS/MINUS -> je nach gewaehlter Achse bewegen
//
// WICHTIG:
// GPIO35 und GPIO39 sind input-only.
// Diese Pins besitzen keine normale Ausgangsfunktion.
//
// Zusätzlich wichtig:
// Für die Taster sind externe 10k Pull-ups vorgesehen.
// Das steht hier ausdrücklich dabei, damit bei der späteren
// Leiterplatte oder Verdrahtung keine Unsicherheit entsteht.
static const int PIN_BTN_MODE  = 13;   // Grün
static const int PIN_BTN_MINUS = 16;   // Weiß
static const int PIN_BTN_PLUS  = 33;    // Schwarz