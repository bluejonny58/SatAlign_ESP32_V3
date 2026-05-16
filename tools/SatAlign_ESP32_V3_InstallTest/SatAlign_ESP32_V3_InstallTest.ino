#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <math.h>

// =====================================================
// SatAlign ESP32 - Erstinstallations-Testversion
// V3_01 InstallTest - Hardwarediagnose und Erstinstallation
// =====================================================
// Zweck:
// Diese Version ist NICHT die normale Satelliten-Suchsoftware.
// Sie dient zur Erstinstallation und zur Hardwareprüfung.
// Alle wichtigen Baugruppen können einzeln getestet werden:
// - Tasten
// - Display
// - GY-521 / MPU6050
// - RF-Detector an GPIO35 inklusive Receiver-/DROP_ADC-Hinweis
// - Azimut-Ausgänge
// - Azimut-Hall-Sensoren mit aktivem Anfahren und Center-Mitte
// - Elevationsmotor
// - Web-Erreichbarkeit
//
// Bedienung am Gerät:
// +      = nächster Menüpunkt
// -      = vorheriger Menüpunkt
// MODE   = Test starten / Test beenden
// MODE lang im Test = zurück ins Installationsmenü
//
// Sicherheitsprinzip:
// - Motoren werden beim Wechsel des Tests immer gestoppt.
// - Hall-Test hat 30 Sekunden Timeout pro Sensorfahrt.
// - Hall-Test startet nach Auswahl NICHT sofort, sondern erst nach Startbefehl.
// - Hall-Timeout startet erst nach dem Verlassen des zuletzt gefundenen Sensors neu.
// - Hall+Mitte-Test prueft WEST/CENTER/EAST und endet in der ungefaehren CENTER-Mitte.
// - Die CENTER-Mitte wird ueber die gemessene CENTER-Hall-Aktivzeit bestimmt.
// - Elevationstest ist nur Tastbetrieb; beim Loslassen stoppt der Motor.
// =====================================================

// =====================================================
// WLAN-Daten
// =====================================================
// Für die Erstinstallation werden dieselben Werte wie im Projekt verwendet.
// Falls WLAN nicht erreichbar ist, laufen alle lokalen Tests trotzdem weiter.
static const char* WIFI_SSID     = "GeniusHome";
static const char* WIFI_PASSWORD = "63758058560720115296";
static const char* WIFI_HOSTNAME = "sat-tracker-install";
static const uint16_t WEB_PORT   = 80;

// =====================================================
// Pinbelegung aus dem Hauptprojekt
// =====================================================
static const int PIN_AD8318_ADC = 35;

static const int PIN_I2C_SDA = 32;
static const int PIN_I2C_SCL = 21;

static const int PIN_AZ_EAST = 22;
static const int PIN_AZ_WEST = 17;

static const int PIN_AZ_HALL_CENTER     = 39;
static const int PIN_AZ_HALL_EAST_LIMIT = 34;
static const int PIN_AZ_HALL_WEST_LIMIT = 36;

static const int PIN_EL_IN1 = 26;
static const int PIN_EL_IN2 = 27;
static const int PIN_EL_ENA = 25;

static const int PIN_TFT_MOSI = 19;
static const int PIN_TFT_SCK  = 18;
static const int PIN_TFT_CS   = 14;
static const int PIN_TFT_DC   = 23;

static const int PIN_BTN_MODE  = 13;
static const int PIN_BTN_MINUS = 16;
static const int PIN_BTN_PLUS  = 33;

// =====================================================
// Konfiguration
// =====================================================
static const bool BUTTON_ACTIVE_LOW = true;
static const bool HALL_ACTIVE_LOW   = true;

// Laut bisherigem Projekt: externe Pullups fuer die Tasten vorgesehen.
// INPUT_PULLUP schadet bei normalem Taster gegen GND nicht und macht den
// Test robuster, falls ein externer Pullup fehlt.
static const bool USE_INTERNAL_BUTTON_PULLUPS = true;

// Hall-Sensoren wurden im Hauptprojekt ohne interne Pullups betrieben.
// Bei Bedarf zum Testen auf true setzen.
static const bool USE_INTERNAL_HALL_PULLUPS = false;

static const unsigned long BUTTON_DEBOUNCE_MS = 35;
static const unsigned long LONG_PRESS_MS      = 1200;
static const unsigned long SCREEN_REFRESH_MS  = 250;
static const unsigned long SERIAL_REFRESH_MS  = 1000;

// Wichtig: vom Nutzer gewünscht.
// Pro Hall-Sensor-Suchfahrt wird nach 30 Sekunden sicher gestoppt.
// Hintergrund: Du hast gemessen, dass CENTER -> naechster Hall-Sensor
// ca. 26 Sekunden dauern kann. 30 Sekunden geben etwas Reserve, stoppen
// aber deutlich frueher als ein unkontrollierter Dauerlauf.
static const unsigned long INSTALL_HALL_SEARCH_TIMEOUT_MS = 30000;

// Kurze Pulse für den Azimut-Motortest über Optokoppler.
static const unsigned long AZ_TEST_PULSE_MS = 500;

// Elevationstest: Tastbetrieb mit bewusst moderatem PWM.
static const int EL_TEST_PWM = 110;

// RF-Kalibrierwerte aus deinem Test an GPIO35.
// Ohne Signal: RAW ca. 1883 -> Anzeige 5 %
// Mit Signal:  RAW ca. 1287 -> Anzeige 95 %
static const float RF_RAW_AT_5_PERCENT  = 1883.10f;
static const float RF_RAW_AT_95_PERCENT = 1286.57f;
static const float RF_ADC_REF_V         = 3.3f;
static const float RF_ADC_MAX           = 4095.0f;

// RF-Signalweg-Pruefung:
// DROP_ADC = RAW ohne Signal - aktueller RAW-Wert.
// Beim AD8317/AD8318 bedeutet ein kleinerer RAW-Wert mehr RF-Signal.
// Bleibt DROP_ADC trotz eingeschaltetem Sat-Receiver unter 30, kommt
// am Detector kein verwertbares Signal an oder der Signalweg ist falsch.
static const float RF_DROP_MIN_OK_ADC = 30.0f;

// MPU6050
static const uint8_t MPU_ADDR = 0x68;

// =====================================================
// Display
// =====================================================
// TFT-Rotation:
// Frueher war der Text auf dem Display
// um 90 Grad im Uhrzeigersinn verdreht. Mit Rotation 3 wird die
// Anzeige um 90 Grad gegen den Uhrzeigersinn korrigiert.
// Falls dein konkretes TFT-Modul anders montiert ist:
// 0, 1, 2 oder 3 testen.
static const uint8_t TFT_ROTATION = 3;

static Adafruit_ST7735 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_MOSI, PIN_TFT_SCK, -1);

static const uint16_t C_BG     = ST77XX_BLACK;
static const uint16_t C_TEXT   = ST77XX_WHITE;
static const uint16_t C_GREEN  = ST77XX_GREEN;
static const uint16_t C_RED    = ST77XX_RED;
static const uint16_t C_YELLOW = ST77XX_YELLOW;
static const uint16_t C_CYAN   = ST77XX_CYAN;
static const uint16_t C_BLUE   = ST77XX_BLUE;

WebServer server(WEB_PORT);

// =====================================================
// Menüstruktur
// =====================================================
enum InstallMenuItem {
  MENU_BUTTONS = 0,
  MENU_DISPLAY,
  MENU_GY521,
  MENU_RF,
  MENU_AZ_MOTOR,
  MENU_HALL_ACTIVE,
  MENU_EL_MOTOR,
  MENU_WEB,
  MENU_COUNT
};

static int currentMenu = MENU_BUTTONS;
static bool testRunning = false;
static unsigned long lastScreenMs = 0;
static unsigned long lastSerialMs = 0;
static String lastStatus = "Bereit";

// Merker fuer den Tastentest.
// Der Tastentest wird bewusst NICHT mehr alle 250 ms komplett neu aufgebaut,
// weil das Display dadurch flackert und schwer lesbar wird.
static bool buttonTestScreenDirty = true;
static bool lastModeShown = false;
static bool lastPlusShown = false;
static bool lastMinusShown = false;

// Merker fuer den GY-521-Test.
// Auch dieser Test baut den Bildschirm nur einmal statisch auf und
// aktualisiert danach nur die Messwert-Zeilen. Dadurch bleibt der Text
// lesbar und laeuft nicht permanent ueber das Display.
static bool gy521TestScreenDirty = true;
static bool gy521LastOkShown = false;
static int gy521LastElevTenths = 99999;

// Merker fuer den RF-Test.
// Der RF-Bildschirm wird wie der GY-521-Test nur einmal statisch aufgebaut.
// Danach werden nur RAW/DROP/Qualitaet/Status aktualisiert, damit der Text
// nicht ueber das Display laeuft und lesbar bleibt.
static bool rfTestScreenDirty = true;
static int rfLastRawShown = -1;
static int rfLastDropShown = 99999;
static int rfLastQualityShown = -1;
static bool rfLastSignalOkShown = false;

// Merker fuer die Motor-/Hall-/Web-Tests ab Test 5.
// Diese Tests haben anfangs ebenfalls den kompletten TFT alle 250 ms neu
// aufgebaut. Dadurch lief/flackerte der Text und war schwer lesbar.
// Auch hier wird nur einmal der statische Bildschirm gezeichnet;
// danach werden nur die wirklich geaenderten Statuszeilen aktualisiert.
static bool azMotorTestScreenDirty = true;
static int azLastWestShown = -1;
static int azLastCenterShown = -1;
static int azLastEastShown = -1;
static int azLastMotorShown = -1; // 0=STOP, 1=EAST, 2=WEST

// Azimut-Motortest: Damit der Test auch bei einem kurzen Tastendruck sichtbar
// reagiert, loesen PLUS/MINUS zusaetzlich einen kurzen Testpuls aus.
// Halten der Taste bleibt weiterhin Dauerfahrt, Loslassen stoppt.
static unsigned long azPulseUntilMs = 0;
static int azPulseDirection = 0; // 0=kein Puls, 1=EAST, 2=WEST
static const unsigned long AZ_MANUAL_TEST_PULSE_MS = 800;

static bool hallTestScreenDirty = true;
static int hallLastWestShown = -1;
static int hallLastCenterShown = -1;
static int hallLastEastShown = -1;
static int hallLastStateShown = -1;
static String hallLastTextShown = "";

static bool elMotorTestScreenDirty = true;
static int elLastOkShown = -1;
static int elLastElevTenths = 99999;
static int elLastMotorShown = -1; // 0=STOP, 1=HOCH, 2=RUNTER

static bool webTestScreenDirty = true;

// =====================================================
// Button-Entprellung
// =====================================================
struct ButtonState {
  int pin;
  const char* name;
  bool rawPressed;
  bool stablePressed;
  bool lastStablePressed;
  unsigned long lastChangeMs;
  unsigned long pressedAtMs;
  bool shortEvent;
  bool longEvent;
};

ButtonState btnMode  = { PIN_BTN_MODE,  "MODE",  false, false, false, 0, 0, false, false };
ButtonState btnPlus  = { PIN_BTN_PLUS,  "PLUS",  false, false, false, 0, 0, false, false };
ButtonState btnMinus = { PIN_BTN_MINUS, "MINUS", false, false, false, 0, 0, false, false };

static bool readButtonRawPressed(int pin) {
  int raw = digitalRead(pin);
  return BUTTON_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}

static void updateButton(ButtonState &b) {
  b.shortEvent = false;
  b.longEvent = false;

  bool rawNow = readButtonRawPressed(b.pin);
  unsigned long now = millis();

  if (rawNow != b.rawPressed) {
    b.rawPressed = rawNow;
    b.lastChangeMs = now;
  }

  if ((now - b.lastChangeMs) >= BUTTON_DEBOUNCE_MS && b.stablePressed != b.rawPressed) {
    b.lastStablePressed = b.stablePressed;
    b.stablePressed = b.rawPressed;

    if (b.stablePressed) {
      b.pressedAtMs = now;
      Serial.print("TASTE GEDRUECKT: ");
      Serial.println(b.name);
    } else {
      unsigned long dur = now - b.pressedAtMs;
      Serial.print("TASTE LOSGELASSEN: ");
      Serial.print(b.name);
      Serial.print(" nach ");
      Serial.print(dur);
      Serial.println(" ms");

      if (dur >= LONG_PRESS_MS) b.longEvent = true;
      else b.shortEvent = true;
    }
  }
}

static void updateButtons() {
  updateButton(btnMode);
  updateButton(btnPlus);
  updateButton(btnMinus);
}


// =====================================================
// Hilfsfunktionen Display
// =====================================================
static void tftHeader(const char* title) {
  tft.fillScreen(C_BG);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(C_CYAN, C_BG);
  tft.setCursor(2, 2);
  tft.print(title);
  tft.drawLine(0, 13, 127, 13, C_CYAN);
}

static void tftLine(int y, const String &text, uint16_t color = C_TEXT) {
  tft.setTextSize(1);
  tft.setTextColor(color, C_BG);
  tft.setCursor(2, y);
  tft.print(text);
}

static const char* menuName(int idx) {
  switch (idx) {
    case MENU_BUTTONS:     return "1 Tasten testen";
    case MENU_DISPLAY:     return "2 Display testen";
    case MENU_GY521:       return "3 GY-521 testen";
    case MENU_RF:          return "4 RF-Detector";
    case MENU_AZ_MOTOR:    return "5 Azimut Motor";
    case MENU_HALL_ACTIVE: return "6 Hall+Mitte";
    case MENU_EL_MOTOR:    return "7 Elevation Motor";
    case MENU_WEB:         return "8 Web-UI testen";
    default:               return "?";
  }
}

static const char* menuNameShort(int idx) {
  // Kurznamen fuer das Ein-Seiten-Menue.
  // Das 1,44"-TFT hat nur wenig Hoehe. Deshalb werden die Menuepunkte
  // hier bewusst kompakt geschrieben, damit alle 8 Tests gleichzeitig
  // sichtbar sind und Test 5-8 nicht mehr auf einer zweiten Seite liegen.
  switch (idx) {
    case MENU_BUTTONS:     return "1 Tasten";
    case MENU_DISPLAY:     return "2 Display";
    case MENU_GY521:       return "3 GY-521";
    case MENU_RF:          return "4 RF";
    case MENU_AZ_MOTOR:    return "5 Az-Motor";
    case MENU_HALL_ACTIVE: return "6 Hall+Mitte";
    case MENU_EL_MOTOR:    return "7 El-Motor";
    case MENU_WEB:         return "8 Web-UI";
    default:               return "?";
  }
}

static void drawMainMenu() {
  tftHeader("INSTALL TEST");

  // Alle 8 Tests werden auf einer Seite angezeigt.
  // Dafuer sind die Menuebezeichnungen bewusst kurz gehalten und die
  // Zeilen enger gesetzt. Die ausfuehrlichen Namen bleiben weiterhin
  // in menuName() fuer seriellen Monitor und Web-UI erhalten.
  tftLine(16, "+/- waehlen  MODE=start", C_TEXT);

  const int MENU_Y_START = 27;
  const int MENU_LINE_H  = 11;

  for (int i = 0; i < MENU_COUNT; i++) {
    const int y = MENU_Y_START + (i * MENU_LINE_H);
    const bool selected = (i == currentMenu);

    // Ganze Zeile vor dem Neuschreiben leeren, damit bei kuerzeren
    // Texten keine alten Zeichenreste stehen bleiben.
    tft.fillRect(0, y - 1, 128, 10, selected ? C_YELLOW : C_BG);

    tft.setTextColor(selected ? C_BG : C_TEXT, selected ? C_YELLOW : C_BG);
    tft.setCursor(2, y);
    tft.print(selected ? ">" : " ");
    tft.print(menuNameShort(i));
  }

  // Untere Bedienzeile kurz halten, damit sie nicht ueberlappt.
  tft.setTextColor(C_CYAN, C_BG);
  tft.fillRect(0, 118, 128, 10, C_BG);
  tft.setCursor(2, 118);
  tft.print("MODE=Start  lang=zurueck");

  // WICHTIG: Keine IP-Adresse im Hauptmenue anzeigen.
  // Die IP-Adresse wird nur im Web-UI-Test, im seriellen Monitor und
  // ueber die Weboberflaeche selbst angezeigt.
}

// =====================================================
// Motor- und Sensorhilfen
// =====================================================
static bool hallActive(int pin) {
  int raw = digitalRead(pin);
  return HALL_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}

// -----------------------------------------------------
// Logische Hall-Sensorzuordnung
// -----------------------------------------------------
// WICHTIGER INSTALLATIONSBEFUND:
// Beim aktiven Hall-Test ist der Motor physisch nach WESTEN gefahren,
// ausgelöst hat aber der bisher als EAST bezeichnete Eingang.
// Daraus folgt: Die logische Zuordnung EAST/WEST ist vertauscht.
//
// Deshalb werden die beiden Endsensoren hier softwareseitig getauscht:
// - hallWest() liest den bisher als EAST_LIMIT bezeichneten Pin
// - hallEast() liest den bisher als WEST_LIMIT bezeichneten Pin
//
// Damit muss an der Verdrahtung nichts geändert werden. Falls du die
// Sensorleitungen später physisch umsteckst, muss diese Zuordnung wieder
// zurückgetauscht werden.
static bool hallWest()   { return hallActive(PIN_AZ_HALL_EAST_LIMIT); }
static bool hallCenter() { return hallActive(PIN_AZ_HALL_CENTER); }
static bool hallEast()   { return hallActive(PIN_AZ_HALL_WEST_LIMIT); }

static void azStop() {
  digitalWrite(PIN_AZ_EAST, LOW);
  digitalWrite(PIN_AZ_WEST, LOW);
}

// -----------------------------------------------------
// Azimut-Richtungsfunktionen
// -----------------------------------------------------
// WICHTIGER INSTALLATIONSBEFUND:
// Im Hall-Test wurde beobachtet, dass die bisherige WEST-Ausgabe den Motor
// physisch nach OSTEN bewegt hat. Deshalb werden die Ausgaenge hier bewusst
// semantisch gedreht: azEastOn() soll wirklich nach Osten fahren und
// azWestOn() wirklich nach Westen.
//
// Falls die Mechanik bei einem anderen Aufbau wieder anders herum laeuft,
// muessen nur die beiden digitalWrite-Zuordnungen in azEastOn()/azWestOn()
// getauscht werden. Die Displaytexte und die Hall-Testlogik bleiben dann
// unveraendert korrekt.
static void azEastOn() {
  // Nicht weiter nach EAST fahren, wenn der EAST-Endsensor bereits aktiv ist.
  if (hallEast()) { azStop(); return; }

  // Korrigierte reale Richtung: PIN_AZ_WEST schaltet physisch EAST.
  digitalWrite(PIN_AZ_EAST, LOW);
  digitalWrite(PIN_AZ_WEST, HIGH);
}

static void azWestOn() {
  // Nicht weiter nach WEST fahren, wenn der WEST-Endsensor bereits aktiv ist.
  if (hallWest()) { azStop(); return; }

  // Korrigierte reale Richtung: PIN_AZ_EAST schaltet physisch WEST.
  digitalWrite(PIN_AZ_EAST, HIGH);
  digitalWrite(PIN_AZ_WEST, LOW);
}

static void elStop() {
  digitalWrite(PIN_EL_IN1, LOW);
  digitalWrite(PIN_EL_IN2, LOW);
  analogWrite(PIN_EL_ENA, 0);
}

// Semantisch HOCH laut Hauptprojekt: Roh-Richtung war invertiert.
static void elUp() {
  digitalWrite(PIN_EL_IN1, LOW);
  digitalWrite(PIN_EL_IN2, HIGH);
  analogWrite(PIN_EL_ENA, EL_TEST_PWM);
}

// Semantisch RUNTER laut Hauptprojekt: Roh-Richtung war invertiert.
static void elDown() {
  digitalWrite(PIN_EL_IN1, HIGH);
  digitalWrite(PIN_EL_IN2, LOW);
  analogWrite(PIN_EL_ENA, EL_TEST_PWM);
}

static void stopAllMotors() {
  azStop();
  elStop();
}

// =====================================================
// RF-Hilfen
// =====================================================
static uint16_t readRfAverage(uint16_t samples = 32) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += analogRead(PIN_AD8318_ADC);
    delayMicroseconds(300);
  }
  return (uint16_t)(sum / samples);
}

static float rfRawToVoltage(float raw) {
  return (raw / RF_ADC_MAX) * RF_ADC_REF_V;
}

static float rfDropAdc(float raw) {
  return RF_RAW_AT_5_PERCENT - raw;
}

static bool rfSignalPathOk(float raw) {
  return rfDropAdc(raw) >= RF_DROP_MIN_OK_ADC;
}

static float rfQualityPercent(float raw) {
  // Mehr Signal -> kleinerer RAW-Wert.
  // Rohwert 1883.10 entspricht 5 %, Rohwert 1286.57 entspricht 95 %.
  float p = 5.0f + ((RF_RAW_AT_5_PERCENT - raw) / (RF_RAW_AT_5_PERCENT - RF_RAW_AT_95_PERCENT)) * 90.0f;
  if (p < 0.0f) p = 0.0f;
  if (p > 100.0f) p = 100.0f;
  return p;
}

// =====================================================
// GY-521 / MPU6050 Hilfen
// =====================================================
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuConnected() {
  Wire.beginTransmission(MPU_ADDR);
  return Wire.endTransmission() == 0;
}

static bool mpuInitBasic() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  delay(50);
  if (!mpuConnected()) return false;
  if (!mpuWrite(0x6B, 0x00)) return false; // Sleep aus
  if (!mpuWrite(0x1C, 0x00)) return false; // Accel ±2g
  if (!mpuWrite(0x1B, 0x00)) return false; // Gyro ±250 dps
  delay(50);
  return true;
}

static bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

static float mpuAccelElevationDeg(int16_t ax, int16_t ay, int16_t az) {
  float fax = (float)ax;
  float fay = (float)ay;
  float faz = (float)az;
  float internal = atan2f(fax, sqrtf(fay * fay + faz * faz)) * 180.0f / PI;
  return 90.0f - internal;
}

static bool mpuReadElevation(float &elevDeg) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadRaw(ax, ay, az, gx, gy, gz)) return false;
  elevDeg = mpuAccelElevationDeg(ax, ay, az);
  return true;
}

// =====================================================
// Einzeltests
// =====================================================
static void drawButtonStateLine(int y, const char* label, bool pressed) {
  // Nur die jeweilige Zeile loeschen, nicht den ganzen Bildschirm.
  // Dadurch bleibt der Tastentest ruhig und lesbar.
  tft.fillRect(0, y, 128, 10, C_BG);
  tft.setTextSize(1);
  tft.setTextColor(pressed ? C_GREEN : C_TEXT, C_BG);
  tft.setCursor(2, y);
  tft.print(label);
  tft.print(pressed ? "GEDRUECKT" : "frei");
}

static void testButtonsScreen() {
  // Den statischen Teil der Anzeige nur einmal zeichnen.
  // Danach werden nur noch die drei Zustandszeilen aktualisiert.
  if (buttonTestScreenDirty) {
    tftHeader("TEST: TASTEN");
    tftLine(18, "Druecke MODE/+/-", C_TEXT);
    tftLine(30, "MODE 2s halten", C_YELLOW);
    tftLine(40, "= zurueck ins Menue", C_YELLOW);
    tftLine(96, "Taste gedrueckt = gruen", C_CYAN);
    tftLine(110, "LOW = gedrueckt", C_CYAN);

    lastModeShown = !btnMode.stablePressed;
    lastPlusShown = !btnPlus.stablePressed;
    lastMinusShown = !btnMinus.stablePressed;
    buttonTestScreenDirty = false;
  }

  if (lastModeShown != btnMode.stablePressed) {
    drawButtonStateLine(56, "MODE : ", btnMode.stablePressed);
    lastModeShown = btnMode.stablePressed;
  }
  if (lastPlusShown != btnPlus.stablePressed) {
    drawButtonStateLine(68, "PLUS : ", btnPlus.stablePressed);
    lastPlusShown = btnPlus.stablePressed;
  }
  if (lastMinusShown != btnMinus.stablePressed) {
    drawButtonStateLine(80, "MINUS: ", btnMinus.stablePressed);
    lastMinusShown = btnMinus.stablePressed;
  }
}

static void runDisplayTest() {
  tft.fillScreen(ST77XX_RED);   delay(400);
  tft.fillScreen(ST77XX_GREEN); delay(400);
  tft.fillScreen(ST77XX_BLUE);  delay(400);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 20); tft.print("DISPLAY TEST OK");
  tft.setCursor(10, 40); tft.print("Text / Farben");
  tft.drawRect(8, 60, 112, 20, ST77XX_CYAN);
  tft.fillRect(10, 62, 108, 16, ST77XX_YELLOW);
  delay(1500);
  lastStatus = "Displaytest fertig";
}

static void clearTftLineArea(int y) {
  tft.fillRect(0, y, 128, 10, C_BG);
}

static void testGy521Screen() {
  float elev = 0.0f;
  bool ok = mpuReadElevation(elev);

  // Statischen Bildschirm nur einmal zeichnen.
  // Vorher wurde der komplette GY-521-Bildschirm alle 250 ms neu aufgebaut.
  // Das erzeugte auf dem kleinen TFT einen laufenden/flackernden Text.
  if (gy521TestScreenDirty) {
    tftHeader("TEST: GY-521");
    tftLine(18, String("SDA=") + PIN_I2C_SDA + " SCL=" + PIN_I2C_SCL, C_TEXT);
    tftLine(86, "MODE kurz = zurueck", C_CYAN);
    tftLine(98, "MODE 2s = Not-zurueck", C_YELLOW);
    tftLine(112, "Winkel langsam bewegen", C_TEXT);

    gy521LastOkShown = !ok;
    gy521LastElevTenths = 99999;
    gy521TestScreenDirty = false;
  }

  // Nur Statuszeile aktualisieren, wenn sich der Status geaendert hat.
  if (gy521LastOkShown != ok) {
    clearTftLineArea(34);
    tftLine(34, ok ? "Sensor: OK" : "Sensor: FEHLER", ok ? C_GREEN : C_RED);
    gy521LastOkShown = ok;
  }

  // Elevation nur bei echter Aenderung neu schreiben.
  // Zehntelgrad reicht fuer den Installationscheck voellig aus.
  int elevTenths = ok ? (int)roundf(elev * 10.0f) : 99998;
  if (gy521LastElevTenths != elevTenths) {
    clearTftLineArea(50);
    if (ok) {
      tftLine(50, String("Elevation: ") + String(elev, 1) + " deg", C_YELLOW);
    } else {
      tftLine(50, "Keine Messwerte", C_RED);
    }
    gy521LastElevTenths = elevTenths;
  }
}

static void testRfScreen() {
  uint16_t raw = readRfAverage(32);
  float volt = rfRawToVoltage(raw);
  float q = rfQualityPercent(raw);
  float drop = rfDropAdc(raw);
  bool signalOk = rfSignalPathOk(raw);

  // Statischen RF-Bildschirm nur einmal zeichnen.
  // Vorher wurde der komplette Bildschirm bei jeder Aktualisierung neu aufgebaut.
  // Dadurch wirkte der Text auf dem kleinen TFT wie ein laufendes/scrollendes Bild.
  if (rfTestScreenDirty) {
    tftHeader("TEST: RF GPIO35");
    tftLine(16, "RF-Test / AD8317", C_TEXT);
    tftLine(28, "Receiver muss EIN sein", C_YELLOW);
    tft.drawRect(4, 82, 120, 10, C_TEXT);
    tftLine(112, "MODE kurz = zurueck", C_CYAN);

    rfLastRawShown = -1;
    rfLastDropShown = 99999;
    rfLastQualityShown = -1;
    rfLastSignalOkShown = !signalOk;
    rfTestScreenDirty = false;
  }

  // Messwerte nur bei relevanter Aenderung neu schreiben.
  // Dadurch bleibt die Anzeige ruhig, zeigt aber weiterhin Livewerte.
  if (abs((int)raw - rfLastRawShown) >= 2) {
    clearTftLineArea(42);
    tftLine(42, String("RAW: ") + raw + "  V:" + String(volt, 2), C_TEXT);
    rfLastRawShown = raw;
  }

  int dropRounded = (int)roundf(drop);
  if (abs(dropRounded - rfLastDropShown) >= 1) {
    clearTftLineArea(56);
    tftLine(56, String("DROP_ADC: ") + dropRounded, signalOk ? C_GREEN : C_RED);
    rfLastDropShown = dropRounded;
  }

  int qRounded = (int)roundf(q);
  if (abs(qRounded - rfLastQualityShown) >= 1) {
    clearTftLineArea(68);
    tftLine(68, String("Qualitaet: ") + qRounded + " %", C_YELLOW);

    // Balkenbereich vor dem Neuzeichnen loeschen, sonst bleiben alte Balkenreste stehen.
    tft.fillRect(5, 83, 118, 8, C_BG);
    tft.fillRect(5, 83, (int)(118.0f * q / 100.0f), 8, signalOk ? C_GREEN : C_YELLOW);
    rfLastQualityShown = qRounded;
  }

  // Wichtiger Praxishinweis aus dem RF-Test:
  // Ohne eingeschalteten Sat-Receiver liegt oft kein verwertbares RF-Signal an.
  // Bleibt DROP_ADC < 30, zuerst Receiver, Signalweg, Verteiler und Verkabelung pruefen.
  if (rfLastSignalOkShown != signalOk) {
    clearTftLineArea(96);
    if (!signalOk) {
      tftLine(96, "DROP<30: Receiver/Signal", C_RED);
      clearTftLineArea(104);
      tftLine(104, "einschalten/pruefen", C_RED);
    } else {
      tftLine(96, "RF-Signalweg: OK", C_GREEN);
      clearTftLineArea(104);
    }
    rfLastSignalOkShown = signalOk;
  }
}

static void drawSensorStateLine(int y, const char* label, bool active, bool okFlag = false) {
  tft.fillRect(0, y, 128, 10, C_BG);
  uint16_t col = okFlag ? C_GREEN : (active ? C_YELLOW : C_TEXT);
  tft.setTextColor(col, C_BG);
  tft.setTextSize(1);
  tft.setCursor(2, y);
  tft.print(label);
  if (okFlag) tft.print("OK");
  else tft.print(active ? "AKTIV" : "frei");
}

static void drawMotorStateLine(int y, const char* text, uint16_t color) {
  tft.fillRect(0, y, 128, 10, C_BG);
  tftLine(y, text, color);
}

static void testAzMotorScreen() {
  // Statischer Teil nur einmal. Dadurch laeuft die Displayanzeige nicht mehr
  // durch, obwohl der Test alle 250 ms aktualisiert wird.
  if (azMotorTestScreenDirty) {
    tftHeader("TEST: AZIMUT MOTOR");
    tftLine(18, "+ kurz/halten = EAST", C_TEXT);
    tftLine(30, "- kurz/halten = WEST", C_TEXT);
    tftLine(42, "MODE kurz = zurueck", C_CYAN);
    tftLine(112, "Endsensor blockiert", C_YELLOW);

    azLastWestShown = -1;
    azLastCenterShown = -1;
    azLastEastShown = -1;
    azLastMotorShown = -1;
    azPulseUntilMs = 0;
    azPulseDirection = 0;
    azMotorTestScreenDirty = false;
  }

  bool w = hallWest();
  bool c = hallCenter();
  bool e = hallEast();

  if (azLastWestShown != (int)w) {
    drawSensorStateLine(62, "WEST:   ", w);
    azLastWestShown = (int)w;
  }
  if (azLastCenterShown != (int)c) {
    drawSensorStateLine(74, "CENTER: ", c);
    azLastCenterShown = (int)c;
  }
  if (azLastEastShown != (int)e) {
    drawSensorStateLine(86, "EAST:   ", e);
    azLastEastShown = (int)e;
  }

  // Wichtige Bedienkorrektur:
  // In der vorherigen Version musste PLUS/MINUS laenger gehalten werden,
  // weil der Bildschirm nur alle 250 ms aktualisiert wurde. Ein kurzer
  // Druck konnte dadurch scheinbar ohne Wirkung bleiben.
  // Jetzt startet jeder kurze PLUS/MINUS-Druck zusaetzlich einen 800-ms-Puls.
  if (btnPlus.shortEvent) {
    azPulseDirection = 1;
    azPulseUntilMs = millis() + AZ_MANUAL_TEST_PULSE_MS;
  }
  if (btnMinus.shortEvent) {
    azPulseDirection = 2;
    azPulseUntilMs = millis() + AZ_MANUAL_TEST_PULSE_MS;
  }

  int requestedState = 0;
  if (btnPlus.stablePressed) {
    requestedState = 1;
    azPulseDirection = 0;
  } else if (btnMinus.stablePressed) {
    requestedState = 2;
    azPulseDirection = 0;
  } else if (azPulseDirection != 0 && millis() < azPulseUntilMs) {
    requestedState = azPulseDirection;
  } else {
    azPulseDirection = 0;
    requestedState = 0;
  }

  int motorState = 0;
  if (requestedState == 1) {
    if (e) {
      azStop();
      motorState = 3; // EAST durch Endsensor blockiert
    } else {
      azEastOn();
      motorState = 1;
    }
  } else if (requestedState == 2) {
    if (w) {
      azStop();
      motorState = 4; // WEST durch Endsensor blockiert
    } else {
      azWestOn();
      motorState = 2;
    }
  } else {
    azStop();
    motorState = 0;
  }

  if (azLastMotorShown != motorState) {
    if (motorState == 1) drawMotorStateLine(100, "Motor: EAST", C_YELLOW);
    else if (motorState == 2) drawMotorStateLine(100, "Motor: WEST", C_YELLOW);
    else if (motorState == 3) drawMotorStateLine(100, "EAST blockiert", C_RED);
    else if (motorState == 4) drawMotorStateLine(100, "WEST blockiert", C_RED);
    else drawMotorStateLine(100, "Motor: STOP", C_CYAN);
    azLastMotorShown = motorState;
  }
}

// -----------------------------------------------------
// Aktiver Hall-Test mit Center-Mitte
// -----------------------------------------------------
// Korrektur:
// Die vorherige integrierte Version hatte den alten Hall-Kompletttest
// praktisch durch einen reinen Center-Mitte-Test ersetzt. Dadurch wurden
// WEST/EAST nicht mehr sauber als "OK" markiert und der Ablauf wirkte so,
// als ob EAST erkannt wurde, aber nicht angezeigt wurde.
//
// Dieser Ablauf kombiniert beides korrekt:
// 1. WEST-Endsensor suchen und als OK markieren
// 2. CENTER suchen und als OK markieren
// 3. EAST-Endsensor suchen und als OK markieren
// 4. Von EAST zurueck Richtung CENTER fahren
// 5. Beim CENTER-Eintritt die Aktivzeitmessung starten
// 6. CENTER vollstaendig durchfahren, bis CENTER wieder frei ist
// 7. Mit halber gemessener CENTER-Zeit zur ungefaehren Mitte zurueckfahren
// 8. TEST OK - MITTE
//
// Wichtig:
// Der Test endet absichtlich in der CENTER-Mitte und faehrt nach EAST
// nicht weiter bis WEST. WEST wurde bereits am Anfang des Tests geprueft.
enum HallTestState {
  HALL_IDLE = 0,
  HALL_TO_WEST,
  HALL_TO_CENTER,
  HALL_TO_EAST,
  HALL_RETURN_CENTER,
  HALL_MEASURE_CENTER_WIDTH,
  HALL_PAUSE_BEFORE_RETURN,
  HALL_RETURN_TO_CENTER_MIDDLE,
  HALL_OK,
  HALL_ERROR
};

static HallTestState hallState = HALL_IDLE;

// Timer pro echter Sensorfahrt.
// Der Timer wird nach jedem gefundenen Sensor pausiert und erst wieder
// gestartet, sobald der gerade gefundene Sensor verlassen wurde.
static unsigned long hallStepStartedMs = 0;
static bool hallStepTimerActive = false;
static bool hallWaitForWestRelease = false;
static bool hallWaitForCenterRelease = false;
static bool hallWaitForEastRelease = false;

// Center-Mittenmessung.
static unsigned long hallCenterEnterMs = 0;
static unsigned long hallCenterExitMs = 0;
static unsigned long hallCenterActiveMs = 0;
static unsigned long hallReturnDurationMs = 0;
static unsigned long hallReturnStartMs = 0;

// Merker fuer Anzeige: OK bleibt sichtbar, auch wenn der Sensor danach
// wieder verlassen wurde.
static bool hallWestOk = false;
static bool hallCenterOk = false;
static bool hallEastOk = false;
bool hallSearchReverseTried = false;  // wird genutzt, wenn beim Center-Suchen zuerst ein Endsensor erreicht wird
static String hallError = "";

static const unsigned long HALL_CENTER_DIRECTION_PAUSE_MS = 350;
static const float HALL_CENTER_MIDDLE_FACTOR = 0.50f;

static void startHallStepTimeout() {
  hallStepStartedMs = millis();
  hallStepTimerActive = true;
}

static void pauseHallStepTimeoutUntilSensorLeft() {
  hallStepTimerActive = false;
}

static const char* hallStateName() {
  switch (hallState) {
    case HALL_IDLE: return "Bereit";
    case HALL_TO_WEST: return "Suche WEST";
    case HALL_TO_CENTER: return "Suche CENTER";
    case HALL_TO_EAST: return "Suche EAST";
    case HALL_RETURN_CENTER: return "Rueck CENTER";
    case HALL_MEASURE_CENTER_WIDTH: return "Messe CENTER";
    case HALL_PAUSE_BEFORE_RETURN: return "Pause vor Mitte";
    case HALL_RETURN_TO_CENTER_MIDDLE: return "Rueck Mitte";
    case HALL_OK: return "TEST OK - MITTE";
    case HALL_ERROR: return "FEHLER";
  }
  return "?";
}

static void startHallTest() {
  hallWestOk = false;
  hallCenterOk = false;
  hallEastOk = false;
  hallError = "";

  hallWaitForWestRelease = false;
  hallWaitForCenterRelease = false;
  hallWaitForEastRelease = false;

  hallCenterEnterMs = 0;
  hallCenterExitMs = 0;
  hallCenterActiveMs = 0;
  hallReturnDurationMs = 0;
  hallReturnStartMs = 0;

  hallState = HALL_TO_WEST;
  startHallStepTimeout();

  lastStatus = "Hall+Mitte: Suche WEST";

  Serial.println();
  Serial.println("HALL+MITTE TEST START");
  Serial.print("Startstatus WEST/CENTER/EAST = ");
  Serial.print(hallWest() ? "1" : "0");
  Serial.print("/");
  Serial.print(hallCenter() ? "1" : "0");
  Serial.print("/");
  Serial.println(hallEast() ? "1" : "0");
  Serial.println("Ablauf: WEST -> CENTER -> EAST -> CENTER-MITTE");
}

static void hallFail(const String& msg) {
  azStop();
  hallError = msg;
  hallState = HALL_ERROR;
  hallStepTimerActive = false;
  lastStatus = "Hall+Mitte Fehler: " + msg;

  Serial.print("HALL+MITTE FEHLER: ");
  Serial.println(msg);
}

static void updateHallTest() {
  if (hallState == HALL_IDLE || hallState == HALL_OK || hallState == HALL_ERROR) return;

  const unsigned long now = millis();

  // Timeout nur fuer echte Suchfahrten pruefen.
  // Nach dem Finden eines Sensors wird der Timer pausiert, bis dieser
  // Sensor wieder verlassen wurde.
  if (hallStepTimerActive && (now - hallStepStartedMs > INSTALL_HALL_SEARCH_TIMEOUT_MS)) {
    azStop();

    if (hallState == HALL_TO_WEST) hallFail("WEST Timeout");
    else if (hallState == HALL_TO_CENTER) hallFail("CENTER Timeout");
    else if (hallState == HALL_TO_EAST) hallFail("EAST Timeout");
    else if (hallState == HALL_RETURN_CENTER) hallFail("CENTER Rueck Timeout");
    else if (hallState == HALL_MEASURE_CENTER_WIDTH) hallFail("CENTER bleibt aktiv");
    else hallFail("Timeout");

    return;
  }

  if (hallState == HALL_TO_WEST) {
    if (hallWest()) {
      azStop();
      hallWestOk = true;
      hallState = HALL_TO_CENTER;
      hallWaitForWestRelease = true;
      pauseHallStepTimeoutUntilSensorLeft();

      lastStatus = "Hall+Mitte: WEST OK";
      Serial.println("HALL TEST: WEST OK. Fahre danach Richtung CENTER.");
      delay(HALL_CENTER_DIRECTION_PAUSE_MS);
    } else {
      azWestOn();
    }
    return;
  }

  if (hallState == HALL_TO_CENTER) {
    // Nach WEST Richtung EAST losfahren und warten, bis WEST wieder frei ist.
    // Erst ab dann gilt der 30-s-Timeout fuer WEST -> CENTER.
    if (hallWaitForWestRelease) {
      azEastOn();
      if (!hallWest()) {
        hallWaitForWestRelease = false;
        startHallStepTimeout();
        Serial.println("HALL TEST: WEST verlassen, Timeout fuer CENTER gestartet.");
      }
      return;
    }

    if (hallCenter()) {
      azStop();
      hallCenterOk = true;
      hallState = HALL_TO_EAST;
      hallWaitForCenterRelease = true;
      pauseHallStepTimeoutUntilSensorLeft();

      lastStatus = "Hall+Mitte: CENTER OK";
      Serial.println("HALL TEST: CENTER OK. Fahre danach Richtung EAST.");
      delay(HALL_CENTER_DIRECTION_PAUSE_MS);
    } else {
      azEastOn();
    }
    return;
  }

  if (hallState == HALL_TO_EAST) {
    // Nach CENTER weiter Richtung EAST fahren und warten, bis CENTER frei ist.
    // Erst danach startet der Timeout fuer CENTER -> EAST.
    if (hallWaitForCenterRelease) {
      azEastOn();
      if (!hallCenter()) {
        hallWaitForCenterRelease = false;
        startHallStepTimeout();
        Serial.println("HALL TEST: CENTER verlassen, Timeout fuer EAST gestartet.");
      }
      return;
    }

    if (hallEast()) {
      azStop();
      hallEastOk = true;
      hallState = HALL_RETURN_CENTER;
      hallWaitForEastRelease = true;
      pauseHallStepTimeoutUntilSensorLeft();

      lastStatus = "Hall+Mitte: EAST OK";
      Serial.println("HALL TEST: EAST OK. Fahre zurueck Richtung CENTER.");
      delay(HALL_CENTER_DIRECTION_PAUSE_MS);
    } else {
      azEastOn();
    }
    return;
  }

  if (hallState == HALL_RETURN_CENTER) {
    // Von EAST aus Richtung WEST zurueckfahren.
    // Erst wenn EAST verlassen wurde, beginnt der 30-s-Timeout fuer
    // die Rueckfahrt zum CENTER.
    if (hallWaitForEastRelease) {
      azWestOn();
      if (!hallEast()) {
        hallWaitForEastRelease = false;
        startHallStepTimeout();
        Serial.println("HALL TEST: EAST verlassen, Timeout fuer CENTER-Rueckfahrt gestartet.");
      }
      return;
    }

    if (hallCenter()) {
      // Nicht stoppen: Wir wollen die Breite des CENTER-Halls messen.
      hallCenterEnterMs = now;
      hallState = HALL_MEASURE_CENTER_WIDTH;
      hallStepTimerActive = false; // Messung selbst ist kurz; Sensorgrenzen sichern separat.
      lastStatus = "Hall+Mitte: Center-Breite messen";
      Serial.println("HALL TEST: CENTER bei Rueckfahrt erreicht. Breite wird gemessen.");
    } else {
      azWestOn();
    }
    return;
  }

  if (hallState == HALL_MEASURE_CENTER_WIDTH) {
    // Weiter Richtung WEST fahren, solange CENTER aktiv ist.
    // Beim Verlassen kennen wir die zeitliche Breite des CENTER-Bereichs.
    if (!hallCenter()) {
      hallCenterExitMs = now;
      hallCenterActiveMs = hallCenterExitMs - hallCenterEnterMs;
      hallReturnDurationMs = (unsigned long)(hallCenterActiveMs * HALL_CENTER_MIDDLE_FACTOR);

      azStop();
      hallState = HALL_PAUSE_BEFORE_RETURN;
      hallStepTimerActive = false;
      hallStepStartedMs = now;
      lastStatus = "Hall+Mitte: Center-Breite gemessen";

      Serial.print("HALL TEST: CENTER verlassen. Aktivzeit: ");
      Serial.print(hallCenterActiveMs);
      Serial.println(" ms");

      Serial.print("HALL TEST: Rueckfahrt zur Mitte: ");
      Serial.print(hallReturnDurationMs);
      Serial.println(" ms");
    } else {
      // Beim Messen fahren wir weiter Richtung WEST durch den CENTER-Bereich.
      // Wenn dabei WEST-Endsensor aktiv wird, ist die Center-Auswertung unplausibel.
      if (hallWest()) {
        hallFail("WEST waehrend Center");
        return;
      }
      azWestOn();
    }
    return;
  }

  if (hallState == HALL_PAUSE_BEFORE_RETURN) {
    if (now - hallStepStartedMs >= HALL_CENTER_DIRECTION_PAUSE_MS) {
      hallState = HALL_RETURN_TO_CENTER_MIDDLE;
      hallReturnStartMs = now;
      lastStatus = "Hall+Mitte: Rueck zur Mitte";
      Serial.println("HALL TEST: Rueckfahrt zur ungefaehren CENTER-Mitte startet.");

      // Da die Breite Richtung WEST gemessen wurde, geht die Rueckfahrt
      // zur Mitte Richtung EAST.
      azEastOn();
    }
    return;
  }

  if (hallState == HALL_RETURN_TO_CENTER_MIDDLE) {
    if (now - hallReturnStartMs >= hallReturnDurationMs) {
      azStop();
      hallState = HALL_OK;
      hallStepTimerActive = false;
      hallCenterOk = true;
      lastStatus = "Hall+Mitte OK";

      Serial.println("HALL TEST: TEST OK - ungefaehre CENTER-Mitte erreicht.");
      return;
    }

    if (hallEast()) {
      hallFail("EAST bei Rueck-Mitte");
      return;
    }

    azEastOn();
    return;
  }
}

static void testHallScreen() {
  updateHallTest();

  // Bildschirm einmalig aufbauen, danach nur Statuszeilen aktualisieren.
  if (hallTestScreenDirty) {
    tftHeader("TEST: HALL+MITTE");
    tftLine(18, "WEST>CENT>EAST>MITTE", C_TEXT);
    tftLine(100, "MODE=Start/Stop", C_GREEN);
    tftLine(112, "lang=zurueck", C_CYAN);

    hallLastWestShown = -1;
    hallLastCenterShown = -1;
    hallLastEastShown = -1;
    hallLastStateShown = -1;
    hallLastTextShown = "";
    hallTestScreenDirty = false;
  }

  const bool wLive = hallWest();
  const bool cLive = hallCenter();
  const bool eLive = hallEast();

  // Status:
  // 0 = frei / noch nicht OK
  // 1 = Sensor gerade live aktiv
  // 2 = Sensor wurde im Ablauf bereits erfolgreich gefunden
  int wState = hallWestOk ? 2 : (wLive ? 1 : 0);
  int cState = hallCenterOk ? 2 : (cLive ? 1 : 0);
  int eState = hallEastOk ? 2 : (eLive ? 1 : 0);

  if (hallLastWestShown != wState) {
    drawSensorStateLine(36, "WEST:   ", wLive, hallWestOk);
    hallLastWestShown = wState;
  }
  if (hallLastCenterShown != cState) {
    drawSensorStateLine(48, "CENTER: ", cLive, hallCenterOk);
    hallLastCenterShown = cState;
  }
  if (hallLastEastShown != eState) {
    drawSensorStateLine(60, "EAST:   ", eLive, hallEastOk);
    hallLastEastShown = eState;
  }

  String stateText;
  if (hallState == HALL_IDLE) {
    stateText = "Bereit: MODE=Start";
  } else if (hallState == HALL_ERROR) {
    stateText = "FEHLER: " + hallError;
  } else {
    stateText = hallStateName();
  }

  String timeText = String("Hallzeit: ") + String(hallCenterActiveMs) + " ms";
  String backText = String("Rueck: ") + String(hallReturnDurationMs) + " ms";

  if (hallLastStateShown != (int)hallState || hallLastTextShown != stateText) {
    tft.fillRect(0, 74, 128, 10, C_BG);
    tftLine(74, stateText, hallState == HALL_ERROR ? C_RED : (hallState == HALL_OK ? C_GREEN : C_CYAN));

    tft.fillRect(0, 86, 128, 10, C_BG);
    tftLine(86, timeText, C_TEXT);

    tft.fillRect(0, 94, 128, 8, C_BG);
    tftLine(94, backText, C_TEXT);

    hallLastStateShown = (int)hallState;
    hallLastTextShown = stateText;
  }
}


static void testElevationScreen() {
  float elev = 0.0f;
  bool ok = mpuReadElevation(elev);

  // Statischen Bildschirm nur einmal aufbauen.
  // Die Livewerte werden zeilenweise aktualisiert, damit nichts scrollt/flackert.
  if (elMotorTestScreenDirty) {
    tftHeader("TEST: ELEVATION");
    tftLine(18, "+ halten = hoch", C_TEXT);
    tftLine(30, "- halten = runter", C_TEXT);
    tftLine(42, "MODE kurz = zurueck", C_CYAN);
    tftLine(104, "Softlimits im Hauptprojekt", C_YELLOW);

    elLastOkShown = -1;
    elLastElevTenths = 99999;
    elLastMotorShown = -1;
    elMotorTestScreenDirty = false;
  }

  int okState = ok ? 1 : 0;
  int elevTenths = ok ? (int)roundf(elev * 10.0f) : 99998;
  if (elLastOkShown != okState || elLastElevTenths != elevTenths) {
    tft.fillRect(0, 62, 128, 10, C_BG);
    tftLine(62, ok ? String("Winkel: ") + String(elev, 1) + " deg" : "GY-521 Fehler", ok ? C_YELLOW : C_RED);
    elLastOkShown = okState;
    elLastElevTenths = elevTenths;
  }

  int motorState = 0;
  if (btnPlus.stablePressed) {
    elUp();
    motorState = 1;
  } else if (btnMinus.stablePressed) {
    elDown();
    motorState = 2;
  } else {
    elStop();
    motorState = 0;
  }

  if (elLastMotorShown != motorState) {
    if (motorState == 1) drawMotorStateLine(86, "Motor: HOCH", C_GREEN);
    else if (motorState == 2) drawMotorStateLine(86, "Motor: RUNTER", C_GREEN);
    else drawMotorStateLine(86, "Motor: STOP", C_CYAN);
    elLastMotorShown = motorState;
  }
}

static void testWebScreen() {
  // Web-Test ist statisch. Er muss nicht alle 250 ms komplett neu gezeichnet werden.
  if (!webTestScreenDirty) return;

  tftHeader("TEST: WEB-UI");
  if (WiFi.status() == WL_CONNECTED) {
    tftLine(22, "WLAN: verbunden", C_GREEN);
    tftLine(38, String("IP: ") + WiFi.localIP().toString(), C_YELLOW);
    tftLine(58, "Browser oeffnen:", C_TEXT);
    tftLine(72, WiFi.localIP().toString(), C_CYAN);
  } else {
    tftLine(22, "WLAN: nicht verbunden", C_RED);
    tftLine(40, "Lokale Tests laufen", C_TEXT);
  }
  tftLine(104, "MODE kurz = zurueck", C_CYAN);
  webTestScreenDirty = false;
}

static void drawRunningTest() {
  switch (currentMenu) {
    case MENU_BUTTONS:     testButtonsScreen(); break;
    case MENU_DISPLAY:     runDisplayTest(); testRunning = false; drawMainMenu(); break;
    case MENU_GY521:       testGy521Screen(); break;
    case MENU_RF:          testRfScreen(); break;
    case MENU_AZ_MOTOR:    testAzMotorScreen(); break;
    case MENU_HALL_ACTIVE: testHallScreen(); break;
    case MENU_EL_MOTOR:    testElevationScreen(); break;
    case MENU_WEB:         testWebScreen(); break;
  }
}

// Setzt vor jedem Teststart nur die Flags zurück, die für den jeweiligen
// Testbildschirm gebraucht werden. Dadurch bleibt startSelectedTest()
// übersichtlich und alte Bildschirmreste werden sicher gelöscht.
static void prepareScreenForSelectedTest() {
  switch (currentMenu) {
    case MENU_BUTTONS:
      buttonTestScreenDirty = true;
      break;

    case MENU_GY521:
      gy521TestScreenDirty = true;
      break;

    case MENU_RF:
      rfTestScreenDirty = true;
      break;

    case MENU_AZ_MOTOR:
      azMotorTestScreenDirty = true;
      break;

    case MENU_HALL_ACTIVE:
      hallTestScreenDirty = true;
      hallWestOk = false;
      hallCenterOk = false;
      hallEastOk = false;
      hallError = "";
      hallState = HALL_IDLE;
      hallCenterEnterMs = 0;
      hallCenterExitMs = 0;
      hallCenterActiveMs = 0;
      hallReturnDurationMs = 0;
      hallReturnStartMs = 0;
      hallSearchReverseTried = false;
      break;

    case MENU_EL_MOTOR:
      elMotorTestScreenDirty = true;
      break;

    case MENU_WEB:
      webTestScreenDirty = true;
      break;

    case MENU_DISPLAY:
    default:
      break;
  }
}

static void startSelectedTest() {
  stopAllMotors();
  testRunning = true;
  lastStatus = String("Test gestartet: ") + menuName(currentMenu);
  Serial.println(lastStatus);

  prepareScreenForSelectedTest();
}

static void stopSelectedTest() {
  azPulseUntilMs = 0;
  azPulseDirection = 0;
  stopAllMotors();
  testRunning = false;
  hallState = HALL_IDLE;
  lastStatus = "Zurueck im Installationsmenue";
  Serial.println(lastStatus);
  drawMainMenu();
}

// =====================================================
// Web UI
// =====================================================
static String buildWebPage() {
  uint16_t raw = readRfAverage(8);
  float q = rfQualityPercent(raw);
  float elev = 0.0f;
  bool mpuOk = mpuReadElevation(elev);

  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='3'>";
  html += "<title>SatAlign ESP32 Install Test</title>";
  html += "<style>body{font-family:Arial;background:#111;color:#eee;margin:16px}button,a.btn{display:inline-block;margin:4px;padding:10px 12px;background:#2d6cdf;color:white;text-decoration:none;border-radius:8px;border:0}.card{background:#222;padding:12px;border-radius:10px;margin:10px 0}.ok{color:#4ade80}.warn{color:#facc15}.err{color:#f87171}</style>";
  html += "</head><body>";
  html += "<h2>SatAlign ESP32 Erstinstallation</h2>";
  html += "<div class='card'><b>Status:</b> " + lastStatus + "<br>";
  html += "<b>Menue:</b> " + String(menuName(currentMenu)) + "<br>";
  html += "<b>Test aktiv:</b> " + String(testRunning ? "ja" : "nein") + "</div>";

  html += "<div class='card'><b>Livewerte</b><br>";
  float drop = rfDropAdc(raw);
  bool rfOk = rfSignalPathOk(raw);
  html += "RF RAW: " + String(raw) + " | DROP_ADC: " + String(drop, 0) + " | Qualitaet: " + String(q, 0) + "%<br>";
  html += "<b>RF-Test:</b> Bitte Sat-Receiver einschalten.<br>";
  html += String("<span class='") + (rfOk ? "ok" : "err") + "'>" + (rfOk ? "RF-Signalweg OK" : "DROP_ADC < 30: Signalweg / Receiver pruefen") + "</span><br>";
  html += "GY-521: " + String(mpuOk ? "OK" : "Fehler") + " | Elevation: " + (mpuOk ? String(elev, 1) : String("--")) + " deg<br>";
  html += "Hall WEST/CENTER/EAST: ";
  html += String(hallWest() ? "1" : "0") + "/" + String(hallCenter() ? "1" : "0") + "/" + String(hallEast() ? "1" : "0") + "</div>";

  html += "<div class='card'><b>Menue</b><br>";
  html += "<a class='btn' href='/prev'>- vorheriger</a>";
  html += "<a class='btn' href='/next'>+ naechster</a>";
  html += "<a class='btn' href='/start'>MODE Start</a>";
  html += "<a class='btn' href='/stop'>STOP</a></div>";

  html += "<div class='card'><b>Direkttests</b><br>";
  html += "<a class='btn' href='/run?test=0'>Tasten</a>";
  html += "<a class='btn' href='/run?test=1'>Display</a>";
  html += "<a class='btn' href='/run?test=2'>GY-521</a>";
  html += "<a class='btn' href='/run?test=3'>RF</a>";
  html += "<a class='btn' href='/run?test=4'>Azimut</a>";
  html += "<a class='btn' href='/run?test=5'>Hall+Mitte</a>";
  html += "<a class='btn' href='/run?test=6'>Elevation</a>";
  html += "<a class='btn' href='/run?test=7'>Web</a></div>";

  html += "<div class='card'><b>Motor-Sicherheit</b><br>";
  html += "<a class='btn' href='/az/east'>AZ EAST Impuls</a>";
  html += "<a class='btn' href='/az/west'>AZ WEST Impuls</a>";
  html += "<a class='btn' href='/stop'>Alle Motoren STOP</a>";
  html += "<p>Hall+Mitte: Center-Hall wird gemessen; Rueckfahrt mit halber Hallzeit.</p>";
  html += "<p>RF-Test: Sat-Receiver einschalten. DROP_ADC muss deutlich ueber 30 liegen.</p></div>";
  html += "</body></html>";
  return html;
}

static void setupWeb() {
  server.on("/", []() { server.send(200, "text/html", buildWebPage()); });
  server.on("/next", []() { currentMenu = (currentMenu + 1) % MENU_COUNT; testRunning = false; stopAllMotors(); drawMainMenu(); server.sendHeader("Location", "/"); server.send(303); });
  server.on("/prev", []() { currentMenu = (currentMenu + MENU_COUNT - 1) % MENU_COUNT; testRunning = false; stopAllMotors(); drawMainMenu(); server.sendHeader("Location", "/"); server.send(303); });
  server.on("/start", []() { startSelectedTest(); server.sendHeader("Location", "/"); server.send(303); });
  server.on("/stop", []() { stopSelectedTest(); server.sendHeader("Location", "/"); server.send(303); });
  server.on("/run", []() {
    if (server.hasArg("test")) {
      int t = server.arg("test").toInt();
      if (t >= 0 && t < MENU_COUNT) currentMenu = t;
    }
    startSelectedTest();
    server.sendHeader("Location", "/");
    server.send(303);
  });
  server.on("/az/east", []() {
    if (!hallEast()) { azEastOn(); delay(AZ_TEST_PULSE_MS); }
    azStop();
    lastStatus = "AZ EAST Impuls ausgefuehrt";
    server.sendHeader("Location", "/"); server.send(303);
  });
  server.on("/az/west", []() {
    if (!hallWest()) { azWestOn(); delay(AZ_TEST_PULSE_MS); }
    azStop();
    lastStatus = "AZ WEST Impuls ausgefuehrt";
    server.sendHeader("Location", "/"); server.send(303);
  });
  server.begin();
}

// =====================================================
// Setup / Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println();
  Serial.println("==========================================");
  Serial.println("SatAlign ESP32 V3_01 InstallTest");
  Serial.println("==========================================");
  Serial.println("RF-Test-Hinweis: Sat-Receiver einschalten.");
  Serial.println("Wenn DROP_ADC unter 30 bleibt: Signalweg / Receiver pruefen.");

  pinMode(PIN_BTN_MODE,  USE_INTERNAL_BUTTON_PULLUPS ? INPUT_PULLUP : INPUT);
  pinMode(PIN_BTN_PLUS,  USE_INTERNAL_BUTTON_PULLUPS ? INPUT_PULLUP : INPUT);
  pinMode(PIN_BTN_MINUS, USE_INTERNAL_BUTTON_PULLUPS ? INPUT_PULLUP : INPUT);

  pinMode(PIN_AZ_EAST, OUTPUT);
  pinMode(PIN_AZ_WEST, OUTPUT);
  azStop();

  pinMode(PIN_AZ_HALL_CENTER,     USE_INTERNAL_HALL_PULLUPS ? INPUT_PULLUP : INPUT);
  pinMode(PIN_AZ_HALL_EAST_LIMIT, USE_INTERNAL_HALL_PULLUPS ? INPUT_PULLUP : INPUT);
  pinMode(PIN_AZ_HALL_WEST_LIMIT, USE_INTERNAL_HALL_PULLUPS ? INPUT_PULLUP : INPUT);

  pinMode(PIN_EL_IN1, OUTPUT);
  pinMode(PIN_EL_IN2, OUTPUT);
  pinMode(PIN_EL_ENA, OUTPUT);
  elStop();

  pinMode(PIN_AD8318_ADC, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_AD8318_ADC, ADC_11db);

  tft.initR(INITR_144GREENTAB);
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(C_BG);
  tft.setTextColor(C_TEXT, C_BG);
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.println("SatAlign InstallTest");
  tft.println("Starte Hardware...");

  bool mpuOk = mpuInitBasic();
  Serial.print("GY-521 Starttest: ");
  Serial.println(mpuOk ? "OK" : "FEHLER");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 5000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WLAN OK, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WLAN nicht verbunden. Lokale Tests bleiben aktiv.");
  }

  setupWeb();
  lastStatus = mpuOk ? "Bereit, GY-521 OK" : "Bereit, GY-521 FEHLER";
  drawMainMenu();

  Serial.println("Bedienung: +/- Menue, MODE Start/Stop. IP im seriellen Monitor und in Test 8.");
}

void loop() {
  server.handleClient();
  updateButtons();

  // MODE lang beendet jeden laufenden Test sofort.
  // Wichtig: Die Rueckkehr wird bereits beim Halten erkannt, nicht erst
  // nach dem Loslassen. Dadurch kommt man auch aus Tests heraus, deren
  // Anzeige laufend aktualisiert wird, z. B. GY-521, RF oder Elevation.
  if (testRunning && btnMode.stablePressed &&
      (millis() - btnMode.pressedAtMs >= LONG_PRESS_MS)) {
    stopSelectedTest();
    return;
  }

  if (testRunning && btnMode.longEvent) {
    stopSelectedTest();
    return;
  }

  // MODE kurz im Hall-Test:
  // - wenn der Hall-Test nur geoeffnet ist und noch nicht faehrt: aktive Fahrt starten
  // - wenn der Hall-Test bereits faehrt oder fertig/fehlerhaft ist: zurueck ins Menue
  // Damit startet Test 6 nach Auswahl nicht mehr sofort.
  if (testRunning && currentMenu == MENU_HALL_ACTIVE && btnMode.shortEvent) {
    if (hallState == HALL_IDLE) {
      startHallTest();
      hallTestScreenDirty = true;
    } else {
      stopSelectedTest();
    }
    return;
  }

  if (!testRunning) {
    if (btnPlus.shortEvent) {
      currentMenu = (currentMenu + 1) % MENU_COUNT;
      Serial.print("MENUE AUSWAHL: ");
      Serial.println(menuName(currentMenu));
      drawMainMenu();
    }
    if (btnMinus.shortEvent) {
      currentMenu = (currentMenu + MENU_COUNT - 1) % MENU_COUNT;
      Serial.print("MENUE AUSWAHL: ");
      Serial.println(menuName(currentMenu));
      drawMainMenu();
    }
    if (btnMode.shortEvent) {
      Serial.print("MENUE START: ");
      Serial.println(menuName(currentMenu));
      startSelectedTest();
      drawRunningTest();
    }
  } else {
    // Bei allen Tests außer Tasten und Hall beendet MODE kurz den Test.
    // Im Tastentest muss MODE normal geprüft werden können, deshalb dort MODE lang.
    if (currentMenu != MENU_BUTTONS && currentMenu != MENU_HALL_ACTIVE && btnMode.shortEvent) {
      stopSelectedTest();
      return;
    }

    // Tests mit Motorbedienung muessen die Tasten in JEDEM loop()-Durchlauf
    // auswerten, nicht nur im Display-Refresh-Intervall. Sonst kann ein kurzer
    // PLUS/MINUS-Druck zwischen zwei 250-ms-Aktualisierungen verloren gehen.
    // Die Displayfunktionen zeichnen nur geaenderte Zeilen neu, daher bleibt
    // die Anzeige trotzdem ruhig und laeuft nicht durch.
    if (currentMenu == MENU_AZ_MOTOR ||
        currentMenu == MENU_HALL_ACTIVE ||
        currentMenu == MENU_EL_MOTOR) {
      drawRunningTest();
    } else if (millis() - lastScreenMs >= SCREEN_REFRESH_MS) {
      lastScreenMs = millis();
      drawRunningTest();
    }
  }

  if (millis() - lastSerialMs >= SERIAL_REFRESH_MS) {
    lastSerialMs = millis();
    uint16_t raw = readRfAverage(8);
    Serial.print("STATUS | Menue=");
    Serial.print(menuName(currentMenu));
    Serial.print(" | Test=");
    Serial.print(testRunning ? "AN" : "AUS");
    float drop = rfDropAdc(raw);
    Serial.print(" | RF_RAW=");
    Serial.print(raw);
    Serial.print(" | DROP_ADC=");
    Serial.print(drop, 0);
    Serial.print(" | RF=");
    Serial.print(rfQualityPercent(raw), 0);
    Serial.print("% | RF_STATUS=");
    Serial.print(rfSignalPathOk(raw) ? "OK" : "RECEIVER/SIGNALWEG PRUEFEN");
    Serial.print(" | Hall W/C/E=");
    Serial.print(hallWest() ? "1" : "0");
    Serial.print("/");
    Serial.print(hallCenter() ? "1" : "0");
    Serial.print("/");
    Serial.println(hallEast() ? "1" : "0");
  }
}
