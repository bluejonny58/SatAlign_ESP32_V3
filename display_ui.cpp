/*
  SatAlign V3 - TFT-Anzeige
  ------------------------------------------------------------
  Zeichnet die lokale Anzeige fuer Startfenster, Hauptmenue, Ausrichten,
  Suchen, Manuell und Statusmeldungen. Die Anzeige ist bewusst kontrastreich
  gehalten, damit sie auch draussen besser lesbar bleibt.
*/

/*
  SatAlign ESP32 - display_ui.cpp
  Version: V3
  ---------------------------------------------------------------------------
  TFT-Anzeige fuer das 1,44-Zoll-Display.

  Ziel der Anzeige in dieser V3-Polish-Version:
  - Outdoor-/High-Contrast-Darstellung fuer helles Umgebungslicht
  - schwarze Flaechen statt grauer Flaechen, damit der Kontrast steigt
  - groessere Statuswoerter fuer MITTE, SUCHE, MANUELL und WARNUNG
  - hellere Signalfarben, damit Gruen/Orange/Rot auf dem kleinen TFT
    auch bei unguenstigem Blickwinkel erkennbar bleiben
  - weiterhin keine langen Diagnosesaetze auf dem TFT; Details bleiben
    im Serial Monitor und in der Web-UI

  WICHTIG:
  Diese Datei aendert nur die Darstellung. Motorlogik, Suchlogik,
  RF-Auswertung, OTA und Tastenlogik bleiben unveraendert.
*/

#include "display_ui.h"

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include "pins.h"

// =====================================================
// TFT-Objekt
// =====================================================
// WICHTIG:
// Die aktuelle Pinbelegung laut pins.h ist:
// - SDA / MOSI = GPIO19
// - SCK        = GPIO18
// - CS         = GPIO14
// - A0 / DC    = GPIO23
// - RST        = direkt an ESP32-RST
//
// Hinweis zur Bezeichnung:
// Am TFT-Modul steht häufig SDA und A0.
// In der Bibliothek heißen dieselben Signale technisch MOSI und DC.
//
// Deshalb wird hier der Konstruktor mit expliziten SPI-Pins verwendet.
// So ist sichergestellt, dass die Anzeige wirklich mit der in pins.h
// festgelegten Verdrahtung arbeitet und nicht stillschweigend die
// Standard-Hardware-SPI-Belegung des ESP32 annimmt.
//
// Der letzte Parameter ist der Reset-Pin des TFT.
// Da TFT-RST direkt am Reset des ESP32 liegt, wird hier -1 verwendet.
static Adafruit_ST7735 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_MOSI, PIN_TFT_SCK, -1);

// =====================================================
// Display-Geometrie (1.44" ST7735, 128x128)
// =====================================================
// Das Layout ist bewusst einfach und robust in fünf feste Zonen geteilt:
//
// 1. Mode-Leiste       -> globaler Betriebszustand (MANUELL / SUCHE / WARNUNG)
// 2. Signalblock       -> RF-Spannung, Balken und Peakmarke
// 3. Elevationszeile   -> Winkel + Bewegungs-/Statusinfo
// 4. Azimutzeile       -> Richtung + Bewegungs-/Statusinfo
// 5. Infozeile         -> allgemeiner Klartext aus der Runtime
//
// Diese feste Struktur ist für das Projekt sehr nützlich, weil sich der
// Bediener schnell orientieren kann und jede Information immer an derselben
// Stelle erscheint.

// Gesamtabmessungen des TFT
static const int SCREEN_W = 128;
static const int SCREEN_H = 128;

// Layout-Zonen
static const int Y_MODE_TOP     = 0;
static const int H_MODE         = 18;

static const int Y_SIGNAL_TOP   = 20;
static const int H_SIGNAL       = 38;

static const int Y_EL_TOP       = 62;
static const int H_EL           = 18;

static const int Y_AZ_TOP       = 84;
static const int H_AZ           = 18;

static const int Y_INFO_TOP     = 108;
static const int H_INFO         = 20;

// =====================================================
// Farben (RGB565)
// =====================================================
// Das ST7735 arbeitet intern mit 16-Bit-Farben (RGB565).
// Mit rgb565() lassen sich Farben trotzdem bequem als normale RGB-Werte
// definieren. Dadurch bleibt der Code lesbarer.
static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Grundfarben / Designsystem
// V3 Outdoor-Kontrast:
// Auf dem 1,44" ST7735 sind mittelgraue Flaechen im Sonnenlicht schlecht
// lesbar. Deshalb werden die Inhaltsflaechen weitgehend schwarz gehalten und
// nur Rahmen/Text sehr hell gezeichnet. Das wirkt weniger "huebsch", ist im
// Live-Test draussen aber deutlich besser ablesbar.
static const uint16_t C_BG         = ST77XX_BLACK;
static const uint16_t C_TEXT       = ST77XX_WHITE;
static const uint16_t C_PANEL      = ST77XX_BLACK;
static const uint16_t C_PANEL_2    = ST77XX_BLACK;
static const uint16_t C_BORDER     = rgb565(210, 210, 210);
static const uint16_t C_IDLE       = rgb565(215, 215, 215);

// Modusfarben
// Alle Statusfarben sind bewusst heller als in der Web-UI.
// Der kleine TFT verliert bei Tageslicht viel Saettigung; dunkles Blau/Gruen
// ist dann schlecht unterscheidbar.
static const uint16_t C_MENU       = rgb565(190, 220, 255);
static const uint16_t C_MANUAL     = rgb565(80, 255, 130);
static const uint16_t C_AUTO       = rgb565(70, 170, 255);
static const uint16_t C_SEARCH     = rgb565(255, 185, 25);
static const uint16_t C_WARN       = rgb565(255, 70, 70);
// Ruhige Menuefarben: farbig, aber einheitlich.
// Hauptmenue: bewusst ruhige Farbfamilie statt vieler Einzel-Farben.
// Farben bestehen. Ein gemeinsames Blau/Grau-Design bleibt gut lesbar und
// wirkt weniger kitschig. Warnungen bleiben bewusst deutlich rot/gelb.
static const uint16_t C_MENU_BLUE  = rgb565(20, 120, 220);
static const uint16_t C_MENU_ORANGE= rgb565(20, 120, 220);
static const uint16_t C_MENU_GREEN = rgb565(20, 120, 220);
static const uint16_t C_MENU_PURPLE= rgb565(20, 120, 220);
static const uint16_t C_MENU_DIM   = rgb565(150, 170, 190);
static const uint16_t C_MENU_SEL   = rgb565(0, 150, 255);
static const uint16_t C_WARN_BG    = rgb565(70, 0, 0);
static const uint16_t C_WARN_YEL   = rgb565(255, 235, 20);

// Signalfarben
static const uint16_t C_SIGNAL_LOW = rgb565(255, 70, 70);
static const uint16_t C_SIGNAL_MID = rgb565(255, 230, 20);
static const uint16_t C_SIGNAL_OK  = rgb565(70, 255, 120);

// Peak-/Markerfarbe
static const uint16_t C_PEAK       = ST77XX_CYAN;

// =====================================================
// Letzter Stand fuer minimale Redraws
// =====================================================
// Hier wird der zuletzt gezeichnete Zustand gemerkt.
// Dadurch zeichnet displayRender() später nur die Bereiche neu,
// die sich wirklich geändert haben.
//
// Vorteile:
// - weniger Flackern
// - schnelleres UI
// - geringere Last auf SPI und Controller
static DisplayData lastData = {
  UI_MODE_MENU,
  -999.0f, -1.0f, -1.0f, "",
  -999.0f, "",
  "", "",
  ""
};

// true direkt nach der Initialisierung.
// Beim ersten Rendern wird dann sicher alles komplett gezeichnet.
static bool firstRender = true;

// Hauptmenue-Displayfix:
// DisplayData speichert infoText nur als const char* Zeiger.
// liveGetInfoText() liefert fuer das Hauptmenue jedoch einen wiederverwendeten
// statischen Textpuffer mit MENUE_SEL=1..5. Wenn lastData nur diesen Zeiger
// merkt, zeigen data.infoText und lastData.infoText spaeter auf denselben
// aktuellen Pufferinhalt. Ein Vergleich ueber String(data.infoText) gegen
// String(lastData.infoText) erkennt dann die geaenderte Menueauswahl nicht.
// Deshalb speichern wir fuer das Hauptmenue den zuletzt wirklich gezeichneten
// Infotext als eigene String-Kopie. So wird jede PLUS/MINUS-Auswahl sofort
// auf dem TFT neu angezeigt.
static String lastMenuInfoText = "";

// Merker fuer Vollbild-Sonderseiten wie SUCHE SETUP oder AZ-WARNUNG.
// Wenn so eine Seite verlassen wird, muss das normale Layout komplett neu
// aufgebaut werden, auch wenn der numerische UiMode gleich geblieben ist.
static bool lastWasSpecialScreen = false;

// =====================================================
// Hilfsfunktionen
// =====================================================

// Begrenzung eines Werts auf den Bereich 0.0 ... 1.0.
// Wird für normierte Signalwerte verwendet.
static float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

// Liefert die Grundfarbe für den globalen UI-Modus.
// Diese Farbe wird vor allem in der oberen Mode-Leiste genutzt.
static uint16_t modeColor(UiMode mode) {
  switch (mode) {
    case UI_MODE_MENU:   return C_MENU;
    case UI_MODE_CENTER_ALIGN: return C_SEARCH;
    case UI_MODE_MANUAL: return C_MANUAL;
    case UI_MODE_AUTO:   return C_AUTO;
    case UI_MODE_SEARCH: return C_SEARCH;
    case UI_MODE_WARN:   return C_WARN;
    default:             return C_IDLE;
  }
}

// V3: UI_MODE_AUTO bleibt intern erhalten, weil die Runtime-Logik und
// viele Zustandsnamen weiterhin "AUTO" verwenden. Auf dem TFT wird dem
// Benutzer aber bewusst "SUCHE" angezeigt, damit Web-UI und Display die
// gleiche Bedienlogik sprechen.
// Liefert den kurzen Klartext für den globalen UI-Modus.
static const char* modeText(UiMode mode) {
  switch (mode) {
    case UI_MODE_MENU:   return "MENUE";
    case UI_MODE_CENTER_ALIGN: return "MITTE";
    case UI_MODE_MANUAL: return "MANUELL";
    case UI_MODE_AUTO:   return "SUCHE";
    case UI_MODE_SEARCH: return "SUCHE";
    case UI_MODE_WARN:   return "WARNUNG";
    default:             return "UNBEKANNT";
  }
}

// Farbwahl für den Signalbalken anhand des normierten Werts.
static uint16_t signalColorFromNorm(float norm) {
  norm = clamp01(norm);

  if (norm < 0.33f) return C_SIGNAL_LOW;
  if (norm < 0.66f) return C_SIGNAL_MID;
  return C_SIGNAL_OK;
}

// Zeichnet einen einfachen Rahmen für einen Displaybereich.
static void drawPanelFrame(int x, int y, int w, int h) {
  tft.fillRect(x, y, w, h, C_PANEL);

  // V3 Outdoor-Kontrast:
  // Der Rahmen wird doppelt gezeichnet. Auf dem kleinen TFT verschwinden
  // einpixelige Linien bei hellem Licht schnell; zwei Linien trennen die
  // Bereiche deutlich besser, ohne Displayflaeche zu verschwenden.
  tft.drawRect(x, y, w, h, C_BORDER);
  if (w > 4 && h > 4) {
    tft.drawRect(x + 1, y + 1, w - 2, h - 2, C_BORDER);
  }
}

// Einheitliche Textausgabe-Hilfsfunktion.
// Dadurch ist die eigentliche Zeichenlogik unten deutlich lesbarer.
static void writeText(int x, int y, uint16_t fg, uint16_t bg, uint8_t size, const String& text) {
  tft.setTextWrap(false);
  tft.setTextSize(size);
  tft.setTextColor(fg, bg);
  tft.setCursor(x, y);
  tft.print(text);
}

// Füllt eine Zone mit einer Hintergrundfarbe.
// Praktisch für Teil-Refreshes einzelner Zeilen.
static void clearZone(int x, int y, int w, int h, uint16_t color = C_PANEL) {
  tft.fillRect(x, y, w, h, color);
}

// =====================================================
// Init
// =====================================================

// Initialisiert das TFT und loescht die Anzeige konsequent.
void displayInit() {
  tft.initR(INITR_144GREENTAB);

  // Vom Nutzer praktisch bestätigt:
  // Rotation 3 ist die richtige physische Orientierung für das Display.
  tft.setRotation(3);

  // V3: Beim Einschalten wurden im Live-Test kurze Grafikrudimente
  // sichtbar. Deshalb zeichnet die Initialisierung keine Rahmen mehr vor.
  // Jeder folgende Vollbildschirm baut sich selbst aus einem komplett
  // schwarzen Zustand auf. Das ist ruhiger und verhindert alte Fragmente.
  tft.fillScreen(C_BG);
  firstRender = true;
  lastWasSpecialScreen = true;
}

// Splashscreen bewusst deaktiviert.
// Kommentarstand: V3
//
// Der fruehere Start-/Splashtext war nur ein Verdrahtungshinweis. Beim echten
// Aussentest stoerte er, weil zwischen Splash, MPU-Test und EZ-Start kurz
// unruhige Zwischenbilder sichtbar wurden. Die Funktion bleibt aus
// Kompatibilitaetsgruenden erhalten, zeigt aber nur einen sauberen schwarzen
// Bildschirm und gibt sofort weiter.
void displayShowSplash() {
  tft.fillScreen(C_BG);
  firstRender = true;
  lastWasSpecialScreen = true;
}



// Zeigt das Ergebnis des GY-521-/MPU6050-Boottests.
//
// Diese Anzeige ist bewusst einfach gehalten:
// - OK: Sensor gefunden und relative Elevation sichtbar
// - FEHLER: AUTO darf ohne MPU nicht als sicher gelten
void displayShowMpuBootTestResult(bool ok, float relativeAngleDeg) {
  tft.fillScreen(C_BG);

  if (ok) {
    writeText(10, 18, C_SIGNAL_OK, C_BG, 2, "MPU OK");
    String line = "EZ: ";
    line += String(relativeAngleDeg, 1);
    line += " deg";
    writeText(10, 52, C_TEXT, C_BG, 1, line);
    writeText(10, 76, C_IDLE, C_BG, 1, "GY-521 bereit");
  } else {
    writeText(10, 18, C_WARN, C_BG, 2, "MPU FEHLT");
    writeText(10, 52, C_TEXT, C_BG, 1, "Suche gesperrt");
    writeText(10, 76, C_IDLE, C_BG, 1, "GY-521 pruefen");
  }

  delay(900);

  // Auch nach dem MPU-Test keine Live-Rahmen vorzeichnen.
  // Der naechste Bildschirm (Sued-Hinweis oder Hauptmenue) soll den
  // kompletten Inhalt selbst aufbauen. Das verhindert alte Text-/Rahmenreste
  // waehrend des Boot-Ablaufs.
  tft.fillScreen(C_BG);
  firstRender = true;
}


// Zeigt den Hinweis fuer die grobe Sued-/Mittenausrichtung.
// Diese Anzeige ist bewusst ein Vollbild-Hinweis und wird vor dem
// WLAN/OTA-Start gezeigt, damit bei einer WLAN-Wartezeit keine alten
// Grafikreste sichtbar bleiben.

// Zeigt die automatische Boot-Anfahrt auf die Standard-Elevation.
// Kommentarstand: V3
//
// Diese Anzeige ist bewusst ein eigener Vollbildschirm und kein normaler
// Live-Render. Der Grund: Die Funktion wird im Boot-Ablauf ausgeführt, noch
// bevor Hauptmenue, Web-UI oder AUTO/Suche aktiv sind.
//
// Darstellungsprinzip fuer den Ausseneinsatz:
// - heller/oranger Kopfbereich solange gefahren wird
// - gruene Kopf-/Statusflaeche erst bei erreichtem Ziel
// - gelb/orange Statusflaeche bei Timeout/Hinweis
// - schwarzer Text auf hellen Flaechen, weil das bei ungünstigem Licht oft
//   besser lesbar ist als duenne farbige Schrift auf dunklem Grund
// - Countdown unten rechts, solange die Boot-Anfahrt laeuft oder prueft
//
// Live-Test-Hintergrund V3:
// Beim Einschalten waren auf dem TFT kurze Rudimente alter/halb gezeichneter
// Inhalte sichtbar. Deshalb zeichnet diese Funktion einen klaren Vollbildschirm
// mit schwarzer Basis und festen Wertfeldern. Die aufrufende Logik begrenzt die
// Aktualisierung auf ca. 1x pro Sekunde, damit die Anzeige nicht flackert und
// Soll/Ist/Restzeit trotz kleinem 128x128-Display lesbar bleiben.

// Kommentarstand: V3
void displayShowManualElevationStart(float currentDeg, float recommendedDeg, bool upPressed, bool downPressed, long remainingSeconds) {
  // V3: Manueller EZ-Start ersetzt den frueheren automatischen EZ-Start
  // mit harter Timeout-/Fehlerwirkung. Die Anzeige ist bewusst ruhig und
  // freundlich gestaltet: Helles Blau bedeutet "Einstellfenster aktiv", nicht
  // Warnung oder Fehler. Der Nutzer hat ca. 15 Sekunden Zeit, die Elevation
  // mit PLUS/MINUS grob auf den empfohlenen Startbereich zu bringen.
  //
  // Wichtig V3:
  // Diese Funktion bleibt reine Anzeige. Die Zeitlogik, Tastenabfrage und
  // Motorfreigabe liegen im Hauptsketch. Dadurch bleibt die Trennung zwischen
  // Display und Steuerlogik erhalten.
  const uint16_t headerColor = C_MENU;       // helles Blau: ruhiger Hinweiszustand
  const uint16_t valueColor  = C_MENU;
  tft.fillScreen(C_BG);

  tft.fillRect(0, 0, SCREEN_W, 26, headerColor);
  writeText(6, 6, ST77XX_BLACK, headerColor, 2, "EZ START");

  tft.fillRoundRect(6, 32, SCREEN_W - 12, 42, 6, valueColor);
  tft.drawRoundRect(6, 32, SCREEN_W - 12, 42, 6, C_TEXT);
  String istLine = "IST ";
  istLine += String(currentDeg, 1);
  writeText(14, 44, ST77XX_BLACK, valueColor, 2, istLine);

  String sollLine = "Ziel ";
  sollLine += String(recommendedDeg, 1);
  writeText(10, 80, C_TEXT, C_BG, 1, sollLine);

  if (remainingSeconds >= 0) {
    String timeLine = "Zeit ";
    timeLine += String(remainingSeconds);
    timeLine += "s";
    writeText(72, 80, C_TEXT, C_BG, 1, timeLine);
  }

  const char* motorText = "WARTET";
  uint16_t motorColor = C_IDLE;
  if (upPressed && !downPressed) {
    motorText = "PLUS: HOCH";
    motorColor = C_SIGNAL_OK;
  } else if (downPressed && !upPressed) {
    motorText = "MINUS: RUNTER";
    motorColor = C_WARN_YEL;
  } else if (upPressed && downPressed) {
    motorText = "STOP";
    motorColor = C_WARN;
  }

  tft.fillRoundRect(6, 94, 116, 16, 4, motorColor);
  tft.drawRoundRect(6, 94, 116, 16, 4, C_TEXT);
  writeText(12, 99, ST77XX_BLACK, motorColor, 1, motorText);

  writeText(8, 116, C_TEXT, C_BG, 1, "MODE = weiter");

  firstRender = true;
  lastWasSpecialScreen = true;
}

// Kommentarstand: V3
void displayShowBootElevationTarget(float currentDeg, float targetDeg, bool moving, bool done, bool timeout, long remainingSeconds) {
  uint16_t headerColor = C_SEARCH;
  const char* title = "EZ START";

  if (done) {
    headerColor = C_SIGNAL_OK;
    title = "EZ OK";
  } else if (timeout) {
    // V3: Boot-EZ-Timeout ist hier bewusst kein roter Fehlerzustand.
    // Der Live-Test hat gezeigt, dass der Motor/Sensor manchmal noch nicht
    // stabil innerhalb der Toleranz bestaetigt, obwohl die Grundfunktion laeuft.
    // Deshalb wird dieser Zustand als gelb/oranger Hinweis angezeigt.
    headerColor = C_WARN_YEL;
    title = "EZ ZEIT";
  }

  // V3: Ruhiger Boot-EZ-Bildschirm.
  //
  // Im Live-Test war der EZ-Start schwer lesbar, weil der Bildschirm sehr
  // oft komplett neu aufgebaut wurde. Deshalb wird hier weiterhin ein
  // Vollbild gezeichnet, aber die aufrufende Boot-Logik aktualisiert nur noch
  // in groesseren Abstaenden. Die Anzeige ist absichtlich klar gegliedert:
  // grosser Titel, grosser Soll-/Ist-Bereich und ein Countdown.
  //
  // Schwarzer Grund entfernt Start-Rudimente, helle Statusflaechen liefern bei
  // unguenstigem Licht mehr Kontrast. Der Text auf den hellen Flaechen ist
  // schwarz, weil das auf dem kleinen ST7735 draussen besser lesbar ist.
  tft.fillScreen(C_BG);

  tft.fillRect(0, 0, SCREEN_W, 28, headerColor);
  writeText(8, 6, C_BG, headerColor, 2, title);

  // Grosser Messblock: weniger Text, dafuer feste Positionen und klare Werte.
  tft.fillRoundRect(6, 34, SCREEN_W - 12, 54, 6, headerColor);
  tft.drawRoundRect(6, 34, SCREEN_W - 12, 54, 6, C_TEXT);
  tft.drawRoundRect(7, 35, SCREEN_W - 14, 52, 6, C_TEXT);

  String targetLine = "SOLL ";
  targetLine += String(targetDeg, 1);
  writeText(14, 42, C_BG, headerColor, 2, targetLine);

  String currentLine = "IST  ";
  currentLine += String(currentDeg, 1);
  writeText(14, 66, C_BG, headerColor, 2, currentLine);

  if (done) {
    tft.fillRoundRect(6, 94, 116, 26, 5, C_SIGNAL_OK);
    writeText(12, 102, C_BG, C_SIGNAL_OK, 1, "Standard-EZ erreicht");
    firstRender = true;
    lastWasSpecialScreen = true;
  } else if (timeout) {
    // V3: Timeout als Hinweis statt Fehler.
    // Rot bleibt echten Fehlern vorbehalten. Beim Boot-EZ-Timeout soll der
    // Anwender nur sehen: Die definierte Zeit ist erreicht, EZ bitte pruefen.
    tft.fillRoundRect(6, 94, 116, 26, 5, C_WARN_YEL);
    tft.drawRoundRect(6, 94, 116, 26, 5, C_TEXT);
    writeText(12, 102, C_BG, C_WARN_YEL, 1, "Zeit erreicht / pruefen");
    firstRender = true;
    lastWasSpecialScreen = true;
  } else {
    const char* stateText = moving ? "fahre zur Standard-EZ" : "pruefe / warte";
    writeText(8, 94, moving ? C_SEARCH : C_TEXT, C_BG, 1, stateText);

    // V3: Sichtbarer Countdown fuer den EZ-Start.
    // Er zeigt dem User, dass die Startpositionierung noch arbeitet und nicht
    // eingefroren ist. 30 Sekunden geben dem Aktuator mehr Zeit zum
    // Einpendeln als die vorherigen 15 Sekunden.
    if (remainingSeconds >= 0) {
      String timeoutLine = "TIME ";
      timeoutLine += String(remainingSeconds);
      timeoutLine += "s";

      tft.fillRoundRect(70, 106, 52, 18, 4, C_WARN_YEL);
      tft.drawRoundRect(70, 106, 52, 18, 4, C_TEXT);
      writeText(76, 111, C_BG, C_WARN_YEL, 1, timeoutLine);
    }
  }
}


void displayShowSouthAlignPrompt() {
  tft.fillScreen(C_BG);
  writeText(8, 8, C_SEARCH, C_BG, 2, "AUSRICHTEN");
  writeText(8, 36, C_TEXT, C_BG, 1, "Nach Sueden");
  writeText(8, 52, C_TEXT, C_BG, 1, "ausrichten");
  writeText(8, 76, C_IDLE, C_BG, 1, "+/- = EL korr.");
  writeText(8, 92, C_IDLE, C_BG, 1, "danach Hauptmenue");
  delay(1800);

  firstRender = true;
}

// =====================================================
// Zeichenfunktionen
// =====================================================

// Zeichnet die obere farbige Leiste mit dem Hauptmodus.
void drawModeBar(UiMode mode) {
  const uint16_t bg = modeColor(mode);
  tft.fillRect(1, Y_MODE_TOP + 1, SCREEN_W - 2, H_MODE - 2, bg);

  // V3 Outdoor-Kontrast:
  // Die Moduszeile ist jetzt bewusst gross. Der Bediener soll schon aus
  // etwas Abstand erkennen, ob er in MENUE, MANUELL, SUCHE oder WARNUNG ist.
  writeText(6, Y_MODE_TOP + 2, ST77XX_WHITE, bg, 2, modeText(mode));
}

// Zeichnet den kompletten Signalblock.
//
// Inhalt:
// - gemessene RF-Spannung
// - normierter Signalbalken
// - Peak-Markierung des besten Werts
// - kurze Textbeschreibung
void drawSignalBlock(float volts, float norm, float bestNorm, const char* signalText) {
  clearZone(1, Y_SIGNAL_TOP + 1, SCREEN_W - 2, H_SIGNAL - 2, C_PANEL);

  // Spannungsanzeige oben im Block
  // V3 Outdoor-Kontrast: RF-Wert groesser statt viele kleine Details.
  String v = "RF ";
  v += String(volts, 2);
  v += "V";
  writeText(6, Y_SIGNAL_TOP + 3, C_TEXT, C_PANEL, 2, v);

  // Balkengeometrie
  const int barX = 6;
  const int barY = Y_SIGNAL_TOP + 22;
  const int barW = SCREEN_W - 12;
  const int barH = 12;

  tft.drawRect(barX, barY, barW, barH, C_BORDER);

  // Aktuelle Füllung
  norm = clamp01(norm);
  const int fillW = (int)((barW - 2) * norm);
  const uint16_t fillColor = signalColorFromNorm(norm);
  tft.fillRect(barX + 1, barY + 1, fillW, barH - 2, fillColor);

  // Peak-Markierung des besten bisher beobachteten Signals.
  // Das hilft beim visuellen Verfolgen der Suchbewegung.
  bestNorm = clamp01(bestNorm);
  int peakX = barX + 1 + (int)((barW - 2) * bestNorm);
  if (peakX < barX + 1) peakX = barX + 1;
  if (peakX > barX + barW - 2) peakX = barX + barW - 2;
  tft.drawFastVLine(peakX, barY + 1, barH - 2, C_PEAK);

  // Signal-Klartext unter dem Balken.
  // Kurz halten, weil das TFT bei Outdoor-Kontrast nur die wichtigsten
  // Werte zeigen soll.
  writeText(6, Y_SIGNAL_TOP + 35, signalColorFromNorm(norm), C_PANEL, 1, signalText ? signalText : "");
}

// Zeichnet die Elevationszeile.
// Die Textfarbe wechselt je nach Status.
void drawElevationRow(float elevationDeg, const char* elevationText) {
  clearZone(1, Y_EL_TOP + 1, SCREEN_W - 2, H_EL - 2, C_PANEL);

  uint16_t stateColor = C_TEXT;
  if (elevationText) {
    String s = elevationText;
    s.toUpperCase();
    if (s.indexOf("LIMIT") >= 0) stateColor = C_WARN;
    else if (s.indexOf("FAEHRT") >= 0 || s.indexOf("HOCH") >= 0 || s.indexOf("RUNTER") >= 0) stateColor = C_SEARCH;
    else if (s.indexOf("BEREICH") >= 0) stateColor = C_SIGNAL_OK;
    else if (s.indexOf("FEIN") >= 0) stateColor = C_AUTO;
  }

  // V3 Outdoor-Kontrast:
  // Die numerische Elevation bleibt links stabil sichtbar; der kurze Status
  // steht rechts farbig. Textgroesse 1 ist hier noetig, damit Statuswoerter
  // wie LIMIT/FAEHRT noch auf 128 px Breite passen.
  writeText(6, Y_EL_TOP + 5, C_TEXT, C_PANEL, 1, "EZ");
  writeText(28, Y_EL_TOP + 5, C_TEXT, C_PANEL, 1, String(elevationDeg, 1) + " deg");
  writeText(78, Y_EL_TOP + 5, stateColor, C_PANEL, 1, elevationText ? elevationText : "");
}

// Zeichnet die Azimutzeile.
// Die Farbe des Richtungsfelds orientiert sich grob an der Bewegungsrichtung.
void drawAzimuthRow(const char* azimuthText, const char* azimuthState) {
  clearZone(1, Y_AZ_TOP + 1, SCREEN_W - 2, H_AZ - 2, C_PANEL);

  uint16_t dirColor = C_IDLE;
  String dir = azimuthText ? azimuthText : "STOP";

  if (dir.indexOf("EAST") >= 0 || dir.indexOf("WEST") >= 0 || dir.indexOf(">>") >= 0 || dir.indexOf("<<") >= 0) {
    dirColor = C_SIGNAL_OK;
  }
  if (dir.indexOf("STOP") >= 0) {
    dirColor = C_IDLE;
  }

  uint16_t stateColor = C_TEXT;
  if (azimuthState) {
    String st = azimuthState;
    st.toUpperCase();
    // Suchphasen optisch unterscheiden.
    // Grobsuche = gruen, Feinsuche = orange.
    if (st.indexOf("GROB") >= 0) stateColor = C_SIGNAL_OK;
    else if (st.indexOf("FEIN") >= 0) stateColor = C_SEARCH;
  }

  writeText(6, Y_AZ_TOP + 5, C_TEXT, C_PANEL, 1, "AZ");
  writeText(28, Y_AZ_TOP + 5, dirColor, C_PANEL, 1, dir);
  writeText(78, Y_AZ_TOP + 5, stateColor, C_PANEL, 1, azimuthState ? azimuthState : "");
}

// Zeichnet die untere Infozeile.
//
// WICHTIG:
// Diese Zeile ist im aktuellen Projekt die wichtigste freie Textfläche,
// um Zustände wie z. B.:
// - Kandidat erkannt
// - falscher Satellit gespeichert
// - Weitersuche
// - AZ fein
// - EL fein
// - Satellit gefunden
// später deutlich sichtbar anzuzeigen.
void drawInfoBar(UiMode mode, const char* infoText) {
  tft.fillRect(1, Y_INFO_TOP + 1, SCREEN_W - 2, H_INFO - 2, C_PANEL_2);

  uint16_t fg = C_TEXT;
  if (mode == UI_MODE_MENU) fg = C_MENU;
  if (mode == UI_MODE_MANUAL) fg = C_MANUAL;
  else if (mode == UI_MODE_AUTO) fg = C_AUTO;
  else if (mode == UI_MODE_SEARCH) fg = C_SEARCH;
  else if (mode == UI_MODE_WARN) fg = C_WARN;

  String out = infoText ? String(infoText) : String("");
  if (infoText) {
    String info = out;
    info.toUpperCase();
    // Auch die untere Such-Infozeile zeigt Grob/Fein eindeutig.
    // Grobsuche = gruen, Feinsuche = orange.
    if (info.indexOf("GROBSUCHE") >= 0) fg = C_SIGNAL_OK;
    else if (info.indexOf("FEINSUCHE") >= 0) fg = C_SEARCH;

    // V3: Fehler-/Limitmeldungen auf dem kleinen TFT kurz und deutlich.
    if (info.indexOf("FEHLER") >= 0 || info.indexOf("BLOCK") >= 0 ||
        info.indexOf("LIMIT") >= 0 || info.indexOf("ENDE") >= 0) {
      fg = C_WARN;
    }
  }

  // 128x128 TFT: Bei Textgroesse 1 passen ca. 20 Zeichen.
  // Laengere technische Details bleiben im Serial Monitor.
  if (out.length() > 20) out = out.substring(0, 20);

  // Die Infozeile bleibt Textgroesse 1, bekommt aber durch schwarzen
  // Hintergrund und helle Farbe mehr Kontrast als vorher.
  writeText(6, Y_INFO_TOP + 6, fg, C_PANEL_2, 1, out);
}


// Zeichnet das echte Hauptmenue nach erfolgreichem Boot.
// In diesem Zustand werden bewusst keine Live-Blöcke für Signal/AZ/EL
// angezeigt, damit klar ist: Der Tracker steht im Startmenue und bewegt nichts.
static void drawMenuItem(int y, int number, const char* label, uint16_t color, bool selected) {
  const int x = 5;
  const int w = SCREEN_W - 10;
  const int h = 17;

  // V3 Outdoor-Kontrast:
  // Auch im Hauptmenue brauchen die "Button-Zeilen" deutliche Rahmen.
  // Der ausgewaehlte Punkt bekommt eine volle blaue Flaeche; nicht ausgewaehlte
  // Punkte bleiben schwarz, haben aber helle Rahmen und hellen Text.
  if (selected) {
    tft.fillRoundRect(x, y, w, h, 3, C_MENU_SEL);
    tft.drawRoundRect(x, y, w, h, 3, C_TEXT);
    tft.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 3, C_TEXT);
    String line = ">" + String(number) + " " + String(label);
    writeText(x + 4, y + 5, C_TEXT, C_MENU_SEL, 1, line);
  } else {
    tft.fillRoundRect(x, y, w, h, 3, C_BG);
    tft.drawRoundRect(x, y, w, h, 3, C_MENU_DIM);
    tft.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 3, C_MENU_DIM);
    String line = " " + String(number) + " " + String(label);
    writeText(x + 4, y + 5, C_TEXT, C_BG, 1, line);
  }
}

static void drawMainMenuScreen(const char* infoText) {
  tft.fillScreen(C_BG);

  int sel = 1;
  String info = infoText ? String(infoText) : String("");
  if (info.indexOf("MENUE_SEL=2") >= 0) sel = 2;
  if (info.indexOf("MENUE_SEL=3") >= 0) sel = 3;
  if (info.indexOf("MENUE_SEL=4") >= 0) sel = 4;
  if (info.indexOf("MENUE_SEL=5") >= 0) sel = 5;

  tft.fillRect(0, 0, SCREEN_W, 21, C_MENU_BLUE);
  writeText(8, 3, C_TEXT, C_MENU_BLUE, 2, "MENUE");

  drawMenuItem(23, 1, "Ausrichten", C_MENU_BLUE,   sel == 1);
  drawMenuItem(42, 2, "Suche",      C_MENU_ORANGE, sel == 2);
  drawMenuItem(61, 3, "Manuell AZ", C_MENU_GREEN,  sel == 3);
  drawMenuItem(80, 4, "Manuell EZ", C_MENU_PURPLE, sel == 4);
  drawMenuItem(99, 5, "Info / IP",  C_MENU_DIM,    sel == 5);

  tft.drawFastHLine(6, 118, SCREEN_W - 12, C_BORDER);
  writeText(8, 121, C_TEXT, C_BG, 1, "+/- Wahl  MODE=OK");
}


static String infoValue(const String& source, const String& key) {
  const String token = key + "=";
  int start = source.indexOf(token);
  if (start < 0) return String("-");
  start += token.length();
  int end = source.indexOf('|', start);
  if (end < 0) end = source.length();
  return source.substring(start, end);
}

static void drawInfoIpScreen(const char* infoText) {
  tft.fillScreen(C_BG);

  const String info = infoText ? String(infoText) : String("");
  const String ip = infoValue(info, "IP");
  const String ssid = infoValue(info, "SSID");
  const String rssi = infoValue(info, "RSSI");

  // V3: Der neue TFT-Menuepunkt "Info" zeigt nur lokale Netzwerkdaten.
  // Er startet keine Motoren und ist bewusst als ruhige Statusseite gedacht,
  // damit die IP-Adresse auch ohne Serial Monitor direkt am Geraet sichtbar ist.
  tft.fillRect(0, 0, SCREEN_W, 21, C_MENU_BLUE);
  writeText(8, 3, C_TEXT, C_MENU_BLUE, 2, "INFO");

  drawPanelFrame(4, 28, SCREEN_W - 8, 68);
  writeText(10, 34, C_IDLE, C_BG, 1, "IP-Adresse");
  writeText(10, 48, C_TEXT, C_BG, 1, ip);
  writeText(10, 66, C_IDLE, C_BG, 1, "WLAN");
  writeText(10, 80, C_TEXT, C_BG, 1, ssid);
  writeText(10, 94, C_IDLE, C_BG, 1, "Signal");
  writeText(62, 94, C_TEXT, C_BG, 1, rssi);

  tft.drawFastHLine(6, 108, SCREEN_W - 12, C_BORDER);
  writeText(8, 112, C_TEXT, C_BG, 1, "MODE = Menue");
  writeText(8, 123, C_TEXT, C_BG, 1, "+/- = Auswahl");
}

static void drawAutoSetupScreen(const char* infoText) {
  tft.fillScreen(C_BG);

  // V3: Das fruehere Feinsuche-Setup wurde entfernt.
  // Diese Seite dient nur noch als ruhiger Startpunkt fuer die Suche:
  // - PLUS/MINUS korrigiert die Hoehe in kurzen Pulsen
  // - MODE startet den Suchlauf Mitte -> Ost -> West -> Mitte
  // - MODE lang geht zurueck ins Hauptmenue
  tft.fillRect(0, 0, SCREEN_W, 22, C_AUTO);
  writeText(6, 3, C_TEXT, C_AUTO, 2, "SUCHE");
  writeText(80, 8, C_TEXT, C_AUTO, 1, "SETUP");

  tft.fillRoundRect(8, 34, SCREEN_W - 16, 38, 5, C_MENU_BLUE);
  tft.drawRoundRect(8, 34, SCREEN_W - 16, 38, 5, C_TEXT);
  tft.drawRoundRect(9, 35, SCREEN_W - 18, 36, 5, C_TEXT);
  writeText(16, 42, C_TEXT, C_MENU_BLUE, 1, "Hoehe mit");
  writeText(16, 58, C_TEXT, C_MENU_BLUE, 1, "+/- pruefen");

  writeText(8, 84, C_TEXT, C_BG, 1, "+/- = Hoehe");
  writeText(8, 100, C_SIGNAL_OK, C_BG, 1, "MODE = START");
  writeText(8, 116, C_WARN_YEL, C_BG, 1, "lang = Menue");
}

static void drawAzWarningScreen(const char* infoText) {
  tft.fillScreen(C_WARN_BG);

  String info = infoText ? String(infoText) : String("");
  const bool east = info.indexOf("E=1") >= 0;
  const bool west = info.indexOf("W=1") >= 0;
  const bool manual = info.indexOf("SRC=MANUAL") >= 0;

  tft.fillRect(0, 0, SCREEN_W, 24, C_WARN);
  writeText(8, 5, C_TEXT, C_WARN, 2, "WARNUNG");

  if (east && west) {
    writeText(8, 34, C_WARN_YEL, C_WARN_BG, 1, "OST+WEST LIMIT");
    writeText(8, 50, C_TEXT, C_WARN_BG, 1, "Sensoren pruefen");
  } else if (east) {
    writeText(8, 34, C_WARN_YEL, C_WARN_BG, 1, "AZ OST-LIMIT");
    writeText(8, 50, C_TEXT, C_WARN_BG, 1, "manuell nach WEST");
  } else if (west) {
    writeText(8, 34, C_WARN_YEL, C_WARN_BG, 1, "AZ WEST-LIMIT");
    writeText(8, 50, C_TEXT, C_WARN_BG, 1, "manuell nach OST");
  } else {
    writeText(8, 34, C_WARN_YEL, C_WARN_BG, 1, "AZ BEREICH");
    writeText(8, 50, C_TEXT, C_WARN_BG, 1, "unklar");
  }

  if (manual) {
    writeText(8, 72, C_TEXT, C_WARN_BG, 1, "MANUAL OVERRIDE");
    writeText(8, 88, C_IDLE, C_WARN_BG, 1, "Motor nicht gesperrt");
  } else {
    writeText(8, 72, C_TEXT, C_WARN_BG, 1, "SUCHE/MITTE STOP");
    writeText(8, 88, C_IDLE, C_WARN_BG, 1, "erst korrigieren");
  }

  writeText(8, 112, C_TEXT, C_WARN_BG, 1, "MODE = Menue");
}


static String centerDebugField(const String& info, const char* key) {
  String marker = String(key) + "=";
  int start = info.indexOf(marker);
  if (start < 0) return "";
  start += marker.length();
  int end = info.indexOf('|', start);
  if (end < 0) end = info.length();
  return info.substring(start, end);
}

static bool centerInfoIsDebug(const char* infoText) {
  // V3: CENTERDBG wird seit der UI-Bereinigung nicht mehr als normale
  // Anwender-Info ausgegeben. Die Erkennung bleibt nur als defensive Altlast-
  // Kompatibilitaet erhalten, falls ein alter Debugtext doch noch ankommt.
  return infoText && String(infoText).indexOf("CENTERDBG|") >= 0;
}

static void writeClippedLine(int x, int y, uint16_t fg, uint16_t bg, const String& text, uint8_t maxChars) {
  String out = text;
  if (out.length() > maxChars) out = out.substring(0, maxChars);
  writeText(x, y, fg, bg, 1, out);
}

// Wandelt die Debug-Zeit aus live_runtime.cpp von Millisekunden in eine
// deutlich ruhigere Sekundenanzeige um. Beispiel:
//   "12345/40000" -> "12/40 s"
// Dadurch laeuft die Mitte-Anzeige nicht mehr mit schnell wechselnden
// Millisekundenwerten durch und bleibt beim realen Test gut lesbar.
static String centerDebugTimeSeconds(const String& timeMs) {
  int slash = timeMs.indexOf('/');
  if (slash < 0) return timeMs;

  unsigned long elapsedMs = (unsigned long)timeMs.substring(0, slash).toInt();
  unsigned long timeoutMs = (unsigned long)timeMs.substring(slash + 1).toInt();

  String out = String(elapsedMs / 1000UL);
  out += "/";
  out += String(timeoutMs / 1000UL);
  out += " s";
  return out;
}

static bool centerDebugImportantChanged(const char* currentInfo, const char* previousInfo) {
  const String cur = currentInfo ? String(currentInfo) : String("");
  const String old = previousInfo ? String(previousInfo) : String("");

  if (centerDebugField(cur, "P") != centerDebugField(old, "P")) return true;
  if (centerDebugField(cur, "D") != centerDebugField(old, "D")) return true;
  if (centerDebugField(cur, "C") != centerDebugField(old, "C")) return true;
  if (centerDebugField(cur, "E") != centerDebugField(old, "E")) return true;
  if (centerDebugField(cur, "W") != centerDebugField(old, "W")) return true;
  if (centerDebugField(cur, "ERR") != centerDebugField(old, "ERR")) return true;

  // Die reine Laufzeit T=... wird absichtlich ignoriert.
  // Sie wird nur noch periodisch aktualisiert.
  return false;
}

static void drawCenterDebugRows(const DisplayData& data) {
  const String info = data.infoText ? String(data.infoText) : String("");
  const String phase = centerDebugField(info, "P");
  const String dir   = centerDebugField(info, "D");
  const String time  = centerDebugField(info, "T");
  const String c     = centerDebugField(info, "C");
  const String e     = centerDebugField(info, "E");
  const String w     = centerDebugField(info, "W");
  String err         = centerDebugField(info, "ERR");

  // Nur den Inhaltsbereich unterhalb der Ueberschrift loeschen.
  // Dadurch bleiben Kopfzeile und Grundbild stabil, waehrend Phase/Zeit/Hall
  // ohne Flackern aktualisiert werden.
  clearZone(0, 28, SCREEN_W, 100, C_BG);

  if (phase == "FAILED") {
    writeText(8, 30, C_WARN, C_BG, 1, "MITTE FEHLER");
    if (err.length() == 0) err = "Grund unbekannt";

    // Auf 128 px Breite passen bei Textgroesse 1 etwa 20 Zeichen.
    // Der Fehlergrund wird deshalb in zwei kurze Zeilen aufgeteilt, damit
    // auf dem Display mehr als nur "Mitte Fehler" sichtbar ist.
    String err1 = err;
    String err2 = "";
    if (err1.length() > 19) {
      err2 = err1.substring(19);
      err1 = err1.substring(0, 19);
      if (err2.length() > 19) err2 = err2.substring(0, 19);
    }

    writeClippedLine(8, 46, C_TEXT, C_BG, err1, 19);
    if (err2.length() > 0) writeClippedLine(8, 58, C_TEXT, C_BG, err2, 19);

    String l3 = "P:" + phase + " D:" + dir;
    String l4 = "Zeit: " + centerDebugTimeSeconds(time);
    String l5 = "Hall C/E/W:" + c + "/" + e + "/" + w;
    writeClippedLine(8, 72, C_IDLE, C_BG, l3, 20);
    writeClippedLine(8, 84, C_IDLE, C_BG, l4, 20);
    writeClippedLine(8, 96, C_IDLE, C_BG, l5, 20);
    writeText(8, 114, C_IDLE, C_BG, 1, "MODE lang = Menue");
    return;
  }

  // Normale laufende Mittefahrt: alle wichtigen Diagnosewerte auf einen Blick.
  // P = Phase, D = Richtung, T = elapsed/timeout, C/E/W = Hall-Zustaende.
  String l1 = "P:" + phase + " D:" + dir;
  String l2 = "Zeit: " + centerDebugTimeSeconds(time);
  String l3 = "Hall C/E/W:" + c + "/" + e + "/" + w;
  String l4 = "EZ:" + String(data.elevationDeg, 1) + " " + String(data.elevationText ? data.elevationText : "");

  writeClippedLine(8, 30, C_TEXT, C_BG, l1, 20);
  writeClippedLine(8, 46, C_TEXT, C_BG, l2, 20);
  writeClippedLine(8, 62, C_TEXT, C_BG, l3, 20);
  writeClippedLine(8, 78, C_IDLE, C_BG, l4, 20);
  writeText(8, 102, C_IDLE, C_BG, 1, "Mitte wird gemessen");
  writeText(8, 116, C_IDLE, C_BG, 1, "MODE lang = Stop");
}

static void drawCenterAlignDynamicRows(const DisplayData& data) {
  // Nur die dynamischen Zeilen loeschen und neu schreiben.
  // Dadurch bleibt der Rest des Bildes stabil und das TFT flackert nicht bei
  // jeder kleinen MPU-Schwankung.
  clearZone(0, 64, SCREEN_W, 28, C_BG);

  String ez = "EZ: ";
  ez += String(data.elevationDeg, 1);
  ez += " deg";
  writeText(8, 66, C_TEXT, C_BG, 1, ez);

  String ezState = "EZ: ";
  ezState += data.elevationText ? data.elevationText : "";
  writeText(8, 78, C_IDLE, C_BG, 1, ezState);
}

static void drawCenterAlignScreen(const DisplayData& data) {
  tft.fillScreen(C_BG);

  String info = data.infoText ? String(data.infoText) : String("");
  const bool homing = (info.indexOf("CENTERDBG|") >= 0 ||
                       info.indexOf("Center") >= 0 ||
                       info.indexOf("Mitte wird gesucht") >= 0);

  // V3 Outdoor-Kontrast:
  // Die Mitte-/Ausrichten-Seite bekommt oben eine farbige Statusflaeche.
  // Laufende Mittenfahrt bleibt orange; erst abgeschlossene Erfolgszustaende
  // duerfen gruen wirken.
  tft.fillRect(0, 0, SCREEN_W, 24, C_SEARCH);
  writeText(8, 4, C_TEXT, C_SEARCH, 2, "MITTE");

  if (info.indexOf("CENTERDBG|") >= 0) {
    drawCenterDebugRows(data);
    return;
  }

  if (homing) {
    writeText(8, 32, C_SEARCH, C_BG, 2, "SUCHT");
    writeText(8, 54, C_TEXT, C_BG, 1, "Mitte laeuft...");
  } else {
    writeText(8, 30, C_TEXT, C_BG, 1, "Nach Sueden");
    writeText(8, 46, C_TEXT, C_BG, 1, "ausrichten");
  }

  drawCenterAlignDynamicRows(data);

  if (homing) {
    writeText(8, 90, C_IDLE, C_BG, 1, "Bitte warten");
    writeText(8, 108, C_IDLE, C_BG, 1, "MODE lang = Stop");
  } else {
    writeText(8, 94, C_IDLE, C_BG, 1, "+/- = EZ kurz");
    writeText(8, 108, C_IDLE, C_BG, 1, "MODE = Mitte");
    writeText(8, 122, C_IDLE, C_BG, 1, "lang = Menue");
  }
}

// Zeichnet das normale Live-Layout mit Rahmen neu.
// Das ist wichtig, wenn vorher das Vollbild-Hauptmenue angezeigt wurde.
static void drawLiveLayoutFrames() {
  tft.fillScreen(C_BG);
  drawPanelFrame(0, Y_MODE_TOP,   SCREEN_W, H_MODE);
  drawPanelFrame(0, Y_SIGNAL_TOP, SCREEN_W, H_SIGNAL);
  drawPanelFrame(0, Y_EL_TOP,     SCREEN_W, H_EL);
  drawPanelFrame(0, Y_AZ_TOP,     SCREEN_W, H_AZ);
  drawPanelFrame(0, Y_INFO_TOP,   SCREEN_W, H_INFO);
}

// =====================================================
// Komplett-Render mit Change-Checks
// =====================================================
// Diese Funktion zeichnet nur dann neu, wenn sich ein Bereich
// wirklich relevant geändert hat.
void displayRender(const DisplayData& data) {
  const String currentInfoText = data.infoText ? String(data.infoText) : String("");

  // Echte Vollbildschirme fuer Suche-Setup und AZ-Limit-Warnungen.
  // Beide werden ueber eindeutige Marker aus live_runtime.cpp angesteuert.
  if (currentInfoText.indexOf("AZWARN|") == 0) {
    if (firstRender || currentInfoText != lastMenuInfoText || data.mode != lastData.mode) {
      drawAzWarningScreen(data.infoText);
      lastMenuInfoText = currentInfoText;
    }
    lastData = data;
    firstRender = false;
    lastWasSpecialScreen = true;
    return;
  }

  if (currentInfoText.indexOf("AUTO_SETUP|") == 0) {
    if (firstRender || currentInfoText != lastMenuInfoText || data.mode != lastData.mode) {
      drawAutoSetupScreen(data.infoText);
      lastMenuInfoText = currentInfoText;
    }
    lastData = data;
    firstRender = false;
    lastWasSpecialScreen = true;
    return;
  }

  if (lastWasSpecialScreen) {
    firstRender = true;
    lastWasSpecialScreen = false;
  }

  // Menuepunkt 1 ist ebenfalls ein eigener Vollbild-Zustand.
  // Hier soll der Bediener zuerst grob nach Sueden ausrichten und mit +/-
  // die Elevation korrigieren, bevor MODE die Mittenfahrt startet.
  if (data.mode == UI_MODE_CENTER_ALIGN) {
    static unsigned long lastCenterDynamicUpdateMs = 0;
    static bool lastCenterWasDebug = false;
    const unsigned long nowMs = millis();

    const bool debugActive = centerInfoIsDebug(data.infoText);

    // Wichtig gegen Display-Rudimente und Flackern:
    // Der Center-Bildschirm wird nur beim Eintritt in diesen Modus oder beim
    // Wechsel Vorbereitung <-> laufende Center-Diagnose komplett neu aufgebaut.
    // Dynamische Texte wie Zeit, Hall C/E/W oder Fehlergrund loesen KEINEN
    // Vollbild-Redraw mehr aus, sondern werden zeilenweise aktualisiert.
    const bool fullRedraw = firstRender ||
                            data.mode != lastData.mode ||
                            debugActive != lastCenterWasDebug;

    if (fullRedraw) {
      drawCenterAlignScreen(data);
      lastCenterDynamicUpdateMs = nowMs;
      lastCenterWasDebug = debugActive;
    } else {
      const bool statusChanged = String(data.elevationText) != String(lastData.elevationText);
      const bool infoChanged = String(data.infoText) != String(lastData.infoText);

      // Waehrend der laufenden Mitte-Zeitmessung enthaelt infoText einen
      // Millisekundenzaehler. Wenn darauf bei jeder Aenderung neu gezeichnet
      // wird, wirkt das TFT im Test unruhig und schwer lesbar.
      // Deshalb werden im Debug-Modus nur echte Zustandswechsel sofort
      // gezeichnet; die Zeit selbst wird nur einmal pro Sekunde aktualisiert.
      const bool centerDebugChanged = debugActive && centerDebugImportantChanged(data.infoText, lastData.infoText);
      const bool periodicUpdate = (nowMs - lastCenterDynamicUpdateMs) >= (debugActive ? 1000 : 500);

      const bool shouldUpdate = debugActive
          ? (statusChanged || centerDebugChanged || periodicUpdate)
          : (statusChanged || infoChanged || periodicUpdate);

      if (shouldUpdate) {
        if (debugActive) {
          drawCenterDebugRows(data);
        } else {
          drawCenterAlignDynamicRows(data);
        }
        lastCenterDynamicUpdateMs = nowMs;
      }
    }

    lastData = data;
    firstRender = false;
    return;
  }

  // Das Hauptmenue ist ein eigener Vollbild-Zustand.
  // Es zeigt bewusst keine RF-/AZ-/EL-Livewerte, damit der Startzustand eindeutig ist.
  if (data.mode == UI_MODE_MENU) {
    const String currentMenuInfoText = data.infoText ? String(data.infoText) : String("");

    if (currentMenuInfoText.indexOf("INFO_SCREEN") >= 0) {
      if (firstRender || data.mode != lastData.mode || currentMenuInfoText != lastMenuInfoText) {
        drawInfoIpScreen(data.infoText);
        lastMenuInfoText = currentMenuInfoText;
      }

      lastData = data;
      firstRender = false;
      return;
    }

    // Infozeile:
    // Hauptmenue-Auswahl nicht ueber lastData.infoText vergleichen.
    // lastData.infoText ist nur ein Zeiger auf den statischen Runtime-Puffer
    // und kann dadurch denselben aktuellen Inhalt wie data.infoText zeigen.
    // Mit der eigenen String-Kopie lastMenuInfoText wird MENUE_SEL=1..5
    // zuverlaessig erkannt und das Hauptmenue nach PLUS/MINUS neu gezeichnet.
    if (firstRender || data.mode != lastData.mode || currentMenuInfoText != lastMenuInfoText) {
      drawMainMenuScreen(data.infoText);
      lastMenuInfoText = currentMenuInfoText;
    }

    lastData = data;
    firstRender = false;
    return;
  }

  // Wenn vorher das Vollbild-Hauptmenue aktiv war, muss das normale
  // Live-Layout mit Rahmen einmal vollständig neu aufgebaut werden.
  if (lastData.mode == UI_MODE_MENU || lastData.mode == UI_MODE_CENTER_ALIGN) {
    drawLiveLayoutFrames();
    firstRender = true;
  }

  // Mode-Leiste neu zeichnen, wenn sich der Hauptmodus ändert
  if (firstRender || data.mode != lastData.mode) {
    drawModeBar(data.mode);
  }

  // Signalblock nur bei relevanten Änderungen neu zeichnen
  if (firstRender ||
      fabsf(data.signalVolts - lastData.signalVolts) > 0.02f ||
      fabsf(data.signalNorm - lastData.signalNorm) > 0.01f ||
      fabsf(data.signalBestNorm - lastData.signalBestNorm) > 0.01f ||
      String(data.signalText) != String(lastData.signalText)) {
    drawSignalBlock(data.signalVolts, data.signalNorm, data.signalBestNorm, data.signalText);
  }

  // Elevationszeile nur bei relevanten Änderungen neu zeichnen
  if (firstRender ||
      fabsf(data.elevationDeg - lastData.elevationDeg) > 0.1f ||
      String(data.elevationText) != String(lastData.elevationText)) {
    drawElevationRow(data.elevationDeg, data.elevationText);
  }

  // Azimutzeile nur bei Änderungen in Richtung oder Zustand neu zeichnen
  if (firstRender ||
      String(data.azimuthText) != String(lastData.azimuthText) ||
      String(data.azimuthState) != String(lastData.azimuthState)) {
    drawAzimuthRow(data.azimuthText, data.azimuthState);
  }

  // Infozeile neu zeichnen, wenn Modus oder Infotext sich ändern
  if (firstRender ||
      data.mode != lastData.mode ||
      String(data.infoText) != String(lastData.infoText)) {
    drawInfoBar(data.mode, data.infoText);
  }

  // Aktuellen Zustand für den nächsten Vergleich merken
  lastData = data;
  firstRender = false;
}
