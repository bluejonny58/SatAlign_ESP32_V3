/*
  SatAlign V3 - mobile Web-UI
  ------------------------------------------------------------
  Diese Datei erzeugt die Weboberflaeche und HTTP-Endpunkte. Die Web-UI soll
  nur Bedienbefehle ausloesen und Status anzeigen. Sie ersetzt keine
  Sicherheitslogik der Runtime.

  Aktueller Bedienfokus V3:
  - klare Menues ohne ueberfluessige Diagnose auf Bedienseiten
  - Suchen mit RF-Ampel; PLUS bestaetigt Kandidaten ohne automatische Optimierung
  - Hall-Sensoren verstaendlich als Center/Ost/West anzeigen
  - Live-Aktualisierung ohne komplettes Neuladen, wo es fuer Tests hilft
*/

/*
  SatAlign ESP32 - web_server.cpp
  Version: V3
  ---------------------------------------------------------------------------
  Web-Oberflaeche und HTTP-Kommandos fuer den ESP32.

  Kommentarstand: V3
  ---------------------------------------------------------------------------
  Diese Datei ist bewusst ausfuehrlich kommentiert, weil die Web-UI im Projekt
  mehrere Aufgaben gleichzeitig erfuellt: Bedienoberflaeche, Statusanzeige,
  Diagnosehilfe und Fernbedienung. Die Kommentare sollen spaeter helfen,
  Fehler in der Anzeige von Fehlern in der eigentlichen Runtime-Logik zu trennen.

  Ziel der Web-UI:
  - Bedienung folgt bewusst der Menuestruktur am TFT:
    1. Ausrichten
    2. Suchen (interne AUTO-Logik)
    3. Manuell
    4. Status / Diagnose
  - Auf dem Handy werden nicht alle Informationen gleichzeitig gezeigt.
    Jede Hauptfunktion hat eine eigene Seite mit nur den dort wichtigen Daten.
  - Web zeigt zusaetzliche Diagnoseinformationen gegenueber dem kleinen TFT.
  - Keine Fachlogik und keine NVS-Speicherung im Webserver.
  - Die Manuell-Seite nutzt fetch()/JSON, damit die Buttonfarben ohne
    kompletten Seitenreload schneller den Motorzustand spiegeln.
*/
#include <Arduino.h>
#include <WebServer.h>

#include "wifi_config.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "config.h"
#include "live_runtime.h"
#include "rf_detector.h"
#include "azimuth_control.h"
#include "elevation_control.h"

// =====================================================
// HTML-/HTTP-Hilfsfunktionen
// =====================================================

// Ein einziger WebServer auf dem in config.h definierten Port.
// Die Instanz bleibt absichtlich dateilokal (static), damit nur dieses Modul
// HTTP-Routen registriert und Anfragen annimmt. Andere Module sprechen das Web
// nicht direkt an, sondern nur ueber die oeffentlichen Funktionen aus
// web_server.h: webServerInit() und webServerLoop().
static WebServer server(WEB_SERVER_PORT);

// HTML-Escaping fuer alle dynamischen Texte.
// Wichtig: Viele angezeigte Werte kommen aus Runtime-Status-Texten. Falls dort
// Sonderzeichen wie <, >, & oder Anfuehrungszeichen vorkommen, duerfen sie im
// Browser nicht als HTML interpretiert werden.
static String esc(const String& in) {
  String s = in;
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  return s;
}

// Erzeugt einen normalen Aktionsbutton als Link im Button-Stil.
//
// allowed == true:
//   Es wird ein klickbarer <a>-Link erzeugt. Der Browser ruft danach eine
//   HTTP-Route auf, z. B. /auto/start oder /center/start.
//
// allowed == false:
//   Es wird bewusst KEIN Link erzeugt, sondern nur ein <span>. Dadurch ist
//   optisch sichtbar, dass die Funktion aktuell gesperrt ist, und der Nutzer
//   kann nicht versehentlich einen unzulaessigen Befehl senden.
static String actionButton(const String& href,
                           const String& label,
                           bool allowed,
                           const String& cssClass = "primary") {
  if (allowed) {
    // Auf Handy-Browsern funktionieren echte Links im Button-Stil zuverlaessiger
    // als verschachtelte Konstruktionen wie <a><button>...</button></a>.
    return "<a class='btn " + cssClass + "' href='" + href + "'>" + esc(label) + "</a>";
  }
  return "<span class='btn disabled'>" + esc(label) + "</span>";
}

// Spezialbutton fuer manuelle Motorsteuerung.
//
// Besonderheit gegenueber actionButton():
// - Ohne JavaScript funktioniert der Link weiterhin als Fallback.
// - Mit JavaScript wird der Befehl per fetch() an eine /api/... Route gesendet.
// - Danach kann die Web-UI die Buttonfarben live aktualisieren, ohne die ganze
//   Seite neu zu laden.
//
// Diese Loesung ist fuer die manuelle Fahrt wichtig, weil der Bediener sofort
// erkennen soll, ob AZ/EZ gerade laeuft oder gestoppt ist.
static String manualControlButton(const String& id,
                                  const String& fallbackHref,
                                  const String& apiHref,
                                  const String& label,
                                  const String& cssClass) {
  // Fallback bleibt ein normaler Link. Mit JavaScript wird der Befehl per fetch()
  // gesendet und die Buttonfarbe ohne kompletten Seitenreload aktualisiert.
  return "<a id='" + id + "' class='btn " + cssClass + "' href='" + fallbackHref +
         "' onclick=\"return sendManualCmd('" + apiHref + "')\">" + esc(label) + "</a>";
}

// Hauptmenue-Kachel.
// Wird nur fuer echte Navigation verwendet, nicht fuer reine Statusanzeigen.
// Dadurch bleibt die visuelle Trennung zwischen Hauptaktionen und Diagnosewerten
// auch im Code nachvollziehbar.
static String tile(const String& href, const String& number, const String& title, const String& hint, const String& cssClass) {
  String html;
  html += "<a class='tile " + cssClass + "' href='" + href + "'>";
  html += "<div class='tileNo'>" + number + "</div>";
  html += "<div><b>" + esc(title) + "</b><span>" + esc(hint) + "</span></div>";
  html += "</a>";
  return html;
}

// Kleine Statusmarke.
// Chips sind reine Informationen, keine Buttons. Sie zeigen kurze Zustaende wie
// Such-Setup, Winkelkorrektur, RF-Qualitaet oder Fehlerphasen.
static String chip(const String& text, const String& cssClass) {
  return "<span class='chip " + cssClass + "'>" + esc(text) + "</span>";
}

static String boolText(bool v) {
  return v ? "1" : "0";
}

// V3: Bedienfreundliche Anzeige der Hall-Sensoren.
// Die fruehere Kurzform C/O/W war fuer Diagnose zwar kompakt, aber fuer die
// Web-UI unklar. Hier wird bewusst ausgeschrieben, ob Center, Ost und West
// aktiv oder aus sind. Das aendert keine Sensorlogik.
static String hallActiveText(bool v) {
  return v ? "aktiv" : "aus";
}

static String hallSensorSummary() {
  // V3: Nutzerfreundliche Reihenfolge und Klartextanzeige.
  // Statt technischer Kurzform C/O/W 1/0/0 wird angezeigt, welcher
  // Hall-Sensor wirklich aktiv ist. Das hilft besonders bei Offline-Tests,
  // weil der Bediener sofort sieht, ob Center, Ost oder West erkannt wird.
  return String("Center ") + hallActiveText(liveHallCenter()) +
         String(" | Ost ") + hallActiveText(liveHallEast()) +
         String(" | West ") + hallActiveText(liveHallWest());
}

static String hallRawSummary() {
  // V3: Rohpinanzeige ebenfalls ausgeschrieben.
  // Diese Zeile bleibt Diagnose: Sie zeigt den direkten Pinzustand vor der
  // logischen Verarbeitung/Invertierung der Hall-Sensoren.
  return String("Center ") + hallActiveText(liveRawHallCenter()) +
         String(" | Ost ") + hallActiveText(liveRawHallEast()) +
         String(" | West ") + hallActiveText(liveRawHallWest());
}

// V3: CSS-Klasse fuer die Hall-Sensor-Zeile.
// Center ist ein gewuenschter Referenzzustand und wird hellgruen markiert.
// Ost/West sind Endbereiche; diese Zustaende werden hellrot markiert, damit
// der Bediener sofort sieht, dass er manuell vorsichtig in Gegenrichtung fahren
// muss. Wenn kein Sensor aktiv ist, bleibt die Zeile neutral.
static String hallSensorRowClass() {
  if (liveHallEast() || liveHallWest()) return "hallLimit";
  if (liveHallCenter()) return "hallCenter";
  return "hallNeutral";
}

// Ordnet den Text der RF-Bewertung einer CSS-Klasse zu.
// Die eigentliche RF-Bewertung kommt aus rf_detector.cpp/live_runtime.cpp.
// Der Webserver entscheidet hier nur ueber die Darstellung: gruen, gelb, rot.
static String rfQualityClass(const String& q) {
  if (q == "sehr gut" || q == "gut") return "good";

  // V3: Die RF-Ampel ist nur eine Anzeige-/Diagnosehilfe.
  // Sie sperrt PLUS bewusst nicht; der Nutzer entscheidet am TV/Receiver.
  // Trotzdem soll "schwach" optisch klar rot sein, damit ein fehlendes
  // oder sehr schwaches Signal nicht wie ein brauchbarer Hinweis wirkt.
  if (q == "brauchbar") return "warn";
  if (q == "schwach") return "bad";

  return "bad";
}

// Ordnet Modus- und AUTO-Phasentexte einer optischen Farbe zu.
// Diese Funktion veraendert keinen Zustand. Sie ist nur Darstellungsschicht.
// Fehler/Block/Limit werden rot, Hinweise orange, Suche gruen,
// Kandidaten- und Satellitenzustaende blau, Setup violett.
static String phaseClass(const String& sIn) {
  String s = sIn;
  s.toUpperCase();
  if (s.indexOf("FEHL") >= 0 || s.indexOf("BLOCK") >= 0 || s.indexOf("LIMIT") >= 0) return "bad";

  // V3: Die Web-UI zeigt jetzt den vereinfachten Suchablauf
  // Mitte -> Ost -> West -> Mitte/Winkel. Die alte Feinsuche ist entfernt.
  // Farben dienen nur der Orientierung und aendern keine Suchlogik.
  if (s.indexOf("HOEHE") >= 0) return "warn";
  if (s.indexOf("RUECK") >= 0 || s.indexOf("MITTE") >= 0) return "blue";
  if (s.indexOf("OST") >= 0 || s.indexOf("WEST") >= 0 || s.indexOf("GROB") >= 0) return "good";
  if (s.indexOf("KANDIDAT") >= 0 || s.indexOf("SAT") >= 0) return "blue";
  if (s.indexOf("SETUP") >= 0) return "violet";
  return "neutral";
}

static String row(const String& label, const String& value) {
  return "<div class='row'><span>" + esc(label) + "</span><b>" + esc(value) + "</b></div>";
}

static String navLink(const String& href, const String& label, const String& page, const String& activePage) {
  String cls = (page == activePage) ? "active" : "";
  return "<a class='" + cls + "' href='" + href + "'>" + esc(label) + "</a>";
}

// Vereinfacht den RF-ADC-Wert fuer den Balken im Web-UI.
// Kommentarstand: V3
// Die Stufen orientieren sich an den Aussentests mit realem TV-Bild.
// kleinerer ADC-Wert = staerkeres Signal. Diese Anzeige ist nur eine Ampel;
// sie blockiert PLUS bewusst nicht; der Nutzer entscheidet am TV/Receiver.
// V3: "schwach" wird eindeutig rot dargestellt. Gelb/Orange ist nur fuer
// "brauchbar" reserviert, damit kein Signalzustand falsch positiv wirkt.
//
// V3: Die Balkenlaenge ist bewusst nicht linear zum ADC-Rohwert. Beim
// AD8317/AD8318 bedeutet ein kleinerer ADC-Wert ein staerkeres Signal. Ohne
// Antenne bzw. bei eindeutig schwachem Signal soll der rote Balken deshalb
// nur sehr kurz erscheinen. Ein langer roter Balken wirkte optisch wie
// "noch recht viel Signal" und war fuer die Bedienung irrefuehrend.
static int rfPercentFromAdc(float adc) {
  if (adc <= RF_TV_STRONG_MAX_ADC) return 100;
  if (adc <= RF_TV_GOOD_MAX_ADC)   return 82;
  if (adc <= RF_TV_USABLE_MAX_ADC) return 58;
  return 8;
}

static String rfColorFromQuality(const String& q) {
  const String cls = rfQualityClass(q);
  if (cls == "good") return "#2e9d64";
  if (cls == "warn") return "#f0a03a";
  return "#d9534f";
}

// Zentrale CSS-Definition fuer alle Web-Seiten.
//
// Gestaltungsidee V3:
// - farbige Buttons = bedienbare Aktionen
// - helle Karten/Infofelder = reine Information, nicht bedienbar
// - jeder echte Button bekommt einen sichtbaren Rahmen gegen Verwechslung mit Infofeldern
// - gruen = abgeschlossener Erfolg oder gute RF-Bewertung
// - orange = laufender Vorgang, Setup, Korrektur oder manuelle Bewegungsrichtung
// - hellgruen = optionale positive Hinweise, aber nicht so dominant wie Erfolg
// - rot/hellrot = Abbruch, Zurueck oder STOP
// - violett = AUTO SETUP / Auswahlphase
//
// Das CSS wird als String ausgeliefert, damit keine zusaetzliche Datei im
// SPIFFS/LittleFS benoetigt wird. Alles bleibt im Arduino-Projekt kompilierbar.
static String commonCss() {
  String css;
  css += ":root{--bg:#edf4fb;--card:#ffffff;--ink:#1d2f42;--muted:#65788b;--line:#d7e4f0;--buttonLine:#24435f;--buttonLineSoft:rgba(36,67,95,.55);--blue:#2f80c9;--blue2:#63a7e6;--green:#2ea66a;--orange:#f1a33b;--orangeLight:#ffd89a;--red:#d9534f;--gray:#8fa1b3;--violet:#7d63c7;--greenLight:#86e39b;--redLight:#f08a84;--shadow:0 8px 24px rgba(34,70,105,.13)}";
  css += "*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:linear-gradient(180deg,#eaf3fb 0%,#f8fbfe 45%,#eef5fb 100%);color:var(--ink);font-family:Arial,sans-serif;padding:8px;}";
  css += ".wrap{max-width:940px;margin:0 auto}.top{position:sticky;top:0;z-index:10;background:rgba(237,244,251,.94);backdrop-filter:blur(10px);padding:8px 0 10px;border-bottom:1px solid var(--line)}";
  css += ".brand{display:flex;align-items:center;justify-content:center;gap:10px}.logo{width:36px;height:36px;border-radius:12px;background:linear-gradient(135deg,var(--blue),var(--violet));display:flex;align-items:center;justify-content:center;color:#fff;font-weight:900;box-shadow:0 5px 16px rgba(47,128,201,.30)}";
  css += "h1{font-size:1.34rem;margin:0;color:#17324d}.sub{text-align:center;color:var(--muted);font-size:.86rem;margin-top:3px}";
  css += ".nav{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-top:9px}.nav a{background:#fff;color:#23415d;text-decoration:none;text-align:center;border-radius:14px;padding:10px 4px;font-weight:900;font-size:.82rem;border:2px solid var(--buttonLineSoft);box-shadow:0 2px 9px rgba(30,60,90,.06),inset 0 0 0 1px rgba(255,255,255,.65)}.nav a.active{background:linear-gradient(135deg,var(--blue),var(--blue2));color:white;border-color:#174f80;box-shadow:0 0 0 3px rgba(47,128,201,.20),0 2px 9px rgba(30,60,90,.10)}";
  css += ".card{background:var(--card);border:1px solid var(--line);border-radius:22px;padding:15px;margin:12px 0;box-shadow:var(--shadow)}";
  css += ".hero{background:linear-gradient(135deg,#ffffff 0%,#edf7ff 100%);border-color:#c8def1}.glass{background:rgba(255,255,255,.74);backdrop-filter:blur(8px)}.title{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-bottom:10px}.title h2{font-size:1.16rem;margin:0;color:#17324d}.step{display:inline-flex;align-items:center;justify-content:center;min-width:32px;height:32px;border-radius:999px;background:linear-gradient(135deg,var(--blue),var(--blue2));color:#fff;font-weight:900;margin-right:8px}";
  css += ".btn{display:flex;align-items:center;justify-content:center;box-sizing:border-box;width:100%;border:3px solid var(--buttonLine);border-radius:17px;padding:15px 8px;font-weight:900;font-size:1.02rem;color:#fff;cursor:pointer;min-height:54px;text-align:center;transition:background .10s ease,box-shadow .10s ease,transform .08s ease,filter .1s ease;box-shadow:0 3px 0 rgba(23,50,77,.28),0 7px 16px rgba(30,60,90,.10)}.btn:active{transform:scale(.975);filter:brightness(.96);box-shadow:0 1px 0 rgba(23,50,77,.32),0 4px 10px rgba(30,60,90,.10)}.primary{background:linear-gradient(135deg,var(--blue),var(--blue2));border-color:#174f80}.green{background:linear-gradient(135deg,#2ea66a,#49bf80);border-color:#146d40}.greenLight{background:linear-gradient(135deg,#86e39b,#c8f7d0);color:#145c2c;border-color:#24964a}.orange{background:linear-gradient(135deg,#f1a33b,#ffc063);color:#5e3600;border-color:#a85f00}.orangeLight{background:linear-gradient(135deg,#ffd89a,#fff0cf);color:#684100;border-color:#bd7a00}.red{background:linear-gradient(135deg,#d9534f,#ef7a73);border-color:#8f211c}.redLight{background:linear-gradient(135deg,#f08a84,#ffd0cc);color:#84201a;border-color:#a9312a}.gray{background:linear-gradient(135deg,#eef3f8,#cfdbe6);color:#405366;border-color:#71879b}.violet{background:linear-gradient(135deg,#7d63c7,#9a82e0);border-color:#4c348e}.activeMove{background:linear-gradient(135deg,#25a85f,#4cc681);border-color:#0f6a37;box-shadow:0 0 0 5px rgba(46,157,100,.20),0 3px 0 rgba(15,106,55,.30)}.activeStop{background:linear-gradient(135deg,#d9534f,#ef7a73);border-color:#8f211c;box-shadow:0 0 0 5px rgba(217,83,79,.23),0 3px 0 rgba(143,33,28,.30)}.disabled{background:#eef2f6;color:#7f8c99;border:3px dashed #94a6b8;cursor:not-allowed;box-shadow:none}";
  css += "a{text-decoration:none}.grid2{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}.ctrl3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.stack{display:grid;gap:10px}.tiles{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}.tile{display:flex;align-items:center;gap:12px;border-radius:22px;padding:17px;background:#fff;border:3px solid var(--buttonLineSoft);box-shadow:0 4px 0 rgba(23,50,77,.18),var(--shadow);color:var(--ink);min-height:92px}.tile b{display:block;font-size:1.08rem}.tile span{display:block;color:var(--muted);font-size:.9rem;margin-top:3px}.tileNo{display:flex;align-items:center;justify-content:center;width:46px;height:46px;border-radius:16px;color:#fff;font-weight:900;font-size:1.2rem;border:2px solid rgba(23,50,77,.35)}.t1 .tileNo{background:linear-gradient(135deg,var(--blue),var(--blue2))}.t2 .tileNo{background:linear-gradient(135deg,var(--violet),#9a82e0)}.t3 .tileNo{background:linear-gradient(135deg,var(--green),#55c989)}.t4 .tileNo{background:linear-gradient(135deg,var(--gray),#9eb0bf)}.t5 .tileNo{background:linear-gradient(135deg,var(--orange),#ffc063)}";
  css += ".chips{display:flex;flex-wrap:wrap;gap:7px;justify-content:center;margin:8px 0}.chip{display:inline-block;border-radius:999px;padding:7px 10px;font-weight:900;font-size:.88rem}.good{background:#dff4e8;color:#17613b}.warn{background:#fff1d7;color:#8a5200}.bad{background:#ffe0de;color:#9a1c18}.blue{background:#dceeff;color:#155b98}.orange.chip{background:#ffe8c6;color:#8a4b00}.violet.chip{background:#ece5ff;color:#4d3197}.neutral{background:#edf2f7;color:#405366}";
  css += ".row{display:flex;justify-content:space-between;gap:10px;border-bottom:1px solid #edf2f7;padding:8px 0}.row span{color:var(--muted)}.row b{text-align:right;color:#1f3448}.row.hallCenter{background:#e7f8ee!important;border:3px solid #73c98c!important;border-radius:12px;padding:12px 14px!important;margin:8px 0!important;box-shadow:0 2px 0 rgba(23,97,59,.16)}.row.hallCenter span,.row.hallCenter b{color:#17613b!important}.row.hallLimit{background:#ffe3e0!important;border:3px solid #ef8f88!important;border-radius:12px;padding:12px 14px!important;margin:8px 0!important;box-shadow:0 2px 0 rgba(141,34,27,.16)}.row.hallLimit span,.row.hallLimit b{color:#8d221b!important}.row.hallNeutral{background:#f8fafc!important;border:2px solid #dbe6ef!important;border-radius:12px;padding:12px 14px!important;margin:8px 0!important}.bigRf{text-align:center;font-size:1.28rem;font-weight:900;color:#17324d}.bar{height:22px;background:#e6edf5;border-radius:999px;overflow:hidden;margin-top:10px}.fill{height:100%;border-radius:999px;transition:width .18s ease}.note{background:#f0f6fc;border-radius:14px;padding:11px;color:#4a6177;line-height:1.38;margin-top:9px}.warnbox{background:#fff0ed;color:#8c2b22}.orangebox{background:#fff2dc;color:#7a4700;border-left:6px solid #f1a33b}.okbox{background:#edf9f1;color:#17613b}.successbox{background:linear-gradient(135deg,#bff2cc,#e7faec);color:#0f572b;border:3px solid #28a85b;box-shadow:0 0 0 6px rgba(40,168,91,.18)}.infoPanel{background:#f7fafc;border:1px solid #d9e5ef;border-left:6px solid #9eb0bf;border-radius:16px;padding:12px;margin-top:10px;color:#4a6177}.infoPanel b{color:#1f3448}.statusGrid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;margin-top:10px}.statusBox{background:#f8fafc;border:1px solid #dfe9f2;border-left:6px solid #aebdcc;border-radius:16px;padding:12px;color:#4a6177}.statusBox span{display:block;font-size:.78rem;text-transform:uppercase;letter-spacing:.04em;color:#6b7d8f;font-weight:900}.statusBox b{display:block;margin-top:4px;font-size:1.1rem;color:#1f3448}.infoPill{display:flex;align-items:center;justify-content:center;border-radius:17px;padding:15px 8px;min-height:54px;font-weight:900;text-align:center}.infoGreen{background:#e3f8e9;color:#17613b;border:1px solid #8bd8a3}.infoOrange{background:#fff0d7;color:#7a4700;border:1px solid #efb95e}.infoRed{background:#ffe7e4;color:#8d221b;border:1px solid #f19a94}.infoBlue{background:#e8f3ff;color:#155b98;border:1px solid #aad0f2}.mini{font-size:.86rem;color:var(--muted)}.sectionHint{color:var(--muted);font-size:.92rem;margin:-4px 0 10px}.pageLead{font-size:.98rem;color:#4a6177;line-height:1.42}";
  css += ".timeline{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:12px 0}.tl{border-radius:13px;padding:9px 4px;text-align:center;font-size:.78rem;font-weight:900;background:#edf2f7;color:#53687b}.tl.on{background:#dff4e8;color:#17613b}.tl.hot{background:#ffe8c6;color:#8a4b00}.tl.bad{background:#ffe0de;color:#9a1c18}.diag{border-left:6px solid var(--blue);padding-left:12px}.diag.good{border-color:var(--green)}.diag.warn{border-color:var(--orange)}.diag.bad{border-color:var(--red)}";
  css += "details summary{cursor:pointer;font-weight:900;padding:8px 0;color:#23415d}details{background:#f7fafc;border-radius:13px;padding:4px 10px;margin-top:10px;border:1px solid #e4ecf4}";
  css += "@media(max-width:620px){body{padding:6px}.nav{grid-template-columns:repeat(5,1fr);gap:4px}.nav a{font-size:.68rem;padding:9px 1px;border-radius:11px}.grid2,.tiles,.statusGrid{grid-template-columns:1fr}.ctrl3{grid-template-columns:repeat(3,1fr)}.btn{font-size:.9rem;padding:13px 5px;min-height:50px}.card{padding:12px;border-radius:18px}.row{font-size:.92rem}h1{font-size:1.12rem}.logo{width:32px;height:32px}.tile{padding:14px;min-height:78px}.tileNo{width:40px;height:40px}.timeline{grid-template-columns:repeat(2,1fr)}}";
  return css;
}
// Einheitlicher HTML-Kopf fuer alle Seiten.
// activePage markiert den aktuell offenen Navigationspunkt. Dadurch erkennt man
// auf dem Handy sofort, ob man im Menue, Ausrichten, Suchen, Manuell oder Hilfe ist.
static String htmlHeader(const String& title, const String& activePage) {
  String html;
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>" + esc(title) + "</title><style>" + commonCss() + "</style></head><body><div class='wrap'>";
  html += "<div class='top'><div class='brand'><div class='logo'>SAT</div><div><h1>SatAlign ESP32</h1><div class='sub'>V3 mobile Web-UI</div></div></div>";
  html += "<div class='nav'>";
  html += navLink("/", "Menue", "home", activePage);
  html += navLink("/ausrichten", "Ausr.", "ausrichten", activePage);
  html += navLink("/auto", "Suchen", "auto", activePage);
  html += navLink("/manuell", "Manuell", "manuell", activePage);
  html += navLink("/trouble", "Hilfe", "trouble", activePage);
  html += "</div></div>";
  return html;
}
static String htmlFooter() {
  return "</div></body></html>";
}

// V3: Kleine Web-UI-Robustheitsfunktion.
// Auf dem ESP32 koennen viele String-Verkettungen Heap-Fragmentierung erzeugen.
// Die Seiten reservieren deshalb vor groesseren HTML-Aufbauten grob genug
// Speicher. Das aendert keine Bedienlogik, reduziert aber das Risiko, dass
// der Browser nur eine leere Antwort bekommt.
static void reserveHtml(String& html, size_t bytes) {
  html.reserve(bytes);
}

// V3: Der fruehere globale Statuskopf wurde bewusst entfernt.
//
// Frueher zeigte jede Web-UI-Seite oben einen zusaetzlichen Block mit
// internem Modus, Suchzustand, manueller Achse und RF-Wert. In den aktuellen
// Bedienseiten stehen die jeweils relevanten Informationen direkt im passenden
// Menuepunkt. Dadurch war der globale Block redundant und fuer Nutzer eher
// verwirrend. Diagnosewerte bleiben weiterhin auf der separaten Statusseite.

// RF-Diagnosekarte.
// Aktualisiert vor dem Anzeigen einmal den RF-Wert und stellt Spannung, RAW-Wert
// und Qualitaet dar. Die Ampel-/Balkenfarbe ist nur Visualisierung; die
// Bewertung selbst kommt aus dem RF-Modul.
static String rfCard() {
  rfUpdate();
  const String rfQuality = liveGetRfQualityText();
  const float rfV = liveGetRfVoltage();
  const float rfAdc = liveGetRfFilteredAdc();
  const int rfPercent = rfPercentFromAdc(rfAdc);
  const String rfColor = rfColorFromQuality(rfQuality);

  String html;
  html += "<div class='card'>";
  html += "<div class='bigRf'>RF " + String(rfV, 3) + " V / " + String(rfAdc, 0) + " ADC - " + esc(rfQuality) + "</div>";
  html += "<div class='bar'><div class='fill' style='width:" + String(rfPercent) + "%;background:" + rfColor + "'></div></div>";
  html += "</div>";
  return html;
}

// Startseite / Hauptmenue.
// Die Reihenfolge entspricht der Bedienlogik am Geraet:
// 1. Ausrichten, 2. AUTO, 3. Manuell, 4. Status, 5. Hilfe.
// Die Web-UI soll sich damit wie eine groessere, besser lesbare Variante des
// TFT-Menues anfuehlen.
static String buildHomePage() {
  String html;
  reserveHtml(html, 9000);
  html = htmlHeader("SatAlign V3", "home");
  // V3: Der fruehere Dashboard-Hinweis unter dem Hauptmenue wurde bewusst
  // entfernt. Die Startseite soll nur noch die direkt bedienbaren Menuekacheln
  // anzeigen; Detail- und Hilfetexte stehen in den jeweiligen Unterseiten.
  html += "<div class='tiles'>";
  html += tile("/ausrichten", "1", "Ausrichten", "Mitte / Center setzen", "t1");
  html += tile("/auto", "2", "Suchen", "Winkel pruefen und Suche starten", "t2");
  html += tile("/manuell", "3", "Manuell", "Azimut und Winkel direkt steuern", "t3");
  html += tile("/status", "4", "Status / Diagnose", "RF, Hall, Winkel, Reset", "t4");
  html += tile("/trouble", "5", "Troubleshooting", "Ursache und Empfehlung", "t5");
  html += "</div>";
  html += rfCard();
  html += htmlFooter();
  return html;
}
// Seite "Ausrichten".
//
// Aufgabe:
// - Center-/Mittenfahrt starten
// - laufenden Vorgang anzeigen
// - Erfolg oder Fehler klar darstellen
//
// Besonderheit:
// Diese Seite nutzt eine kleine JSON-Statusabfrage. Dadurch erscheint die
// gruen hervorgehobene Erfolgsmeldung automatisch, ohne dass der Nutzer die
// Seite manuell aktualisieren muss.
static String buildAusrichtenPage() {
  const bool mpuReady = liveMpuReady();
  const bool inAuto = (String(liveGetModeText()) == "AUTO");
  const bool centerActive = liveCenteringActive();
  const bool centerSuccess = liveCenteringSuccessNoticeActive();

  String html;
  reserveHtml(html, 12000);
  html = htmlHeader("Ausrichten", "ausrichten");
  html += "<div class='card'><div class='title'><h2><span class='step'>1</span>Ausrichten</h2></div>";
  html += "<div class='pageLead'>Setzt die mechanische Mitte als Referenz. Diese Funktion sollte vor der Suche sauber laufen.</div>";

  // Live-Bereich: Diese Elemente werden per /api/center/status aktualisiert.
  // Dadurch erscheint die Erfolgsmeldung nach der Mittenfahrt automatisch,
  // ohne dass der Nutzer die Webseite manuell neu laden muss.
  html += "<div id='centerLiveBox'>";
  if (centerActive) {
    html += "<div id='centerMsg' class='note orangebox'><b>Ausrichten laeuft.</b><br>Die Center-/Mittenfahrt ist aktiv. Mit ABBRUCH wird der Motor gestoppt und das Hauptmenue wieder geoeffnet.</div>";
    html += "<div id='centerButtons' class='grid2' style='margin-top:12px'>";
    html += "<span class='infoPill infoOrange'>Ausrichten laeuft</span>";
    html += actionButton("/center/abort", "ABBRUCH", true, "redLight");
    html += "</div>";
  } else {
    if (centerSuccess) {
      html += "<div id='centerMsg' class='note successbox'><b>✅ Ausrichtung erfolgreich beendet.</b><br>Die Mitte wurde gesetzt. Die Antenne kann jetzt grob nach Sueden ausgerichtet werden; danach die Suche starten.</div>";
    } else {
      html += "<div id='centerMsg' class='note'>Bereit zum Ausrichten. Start setzt die Center-/Mittenreferenz.</div>";
    }
    html += "<div id='centerButtons' class='grid2' style='margin-top:12px'>";
    html += actionButton("/center/start", centerSuccess ? "Ausrichten erneut starten" : "Ausrichten starten", mpuReady && !inAuto && !liveAutoSetupActive() && !liveHallEast() && !liveHallWest(), "orange");
    html += actionButton("/", "Zurueck", true, "redLight");
    html += "</div>";
  }

  // V3: Livewerte fuer Ausrichten/Mitte.
  // Dieser Block wird per /api/center/status aktualisiert, ohne die komplette
  // Seite neu zu laden. Dadurch bleiben Abbruchbutton und Web-UI reaktionsfaehig,
  // waehrend Phase, Info, RF und Winkel trotzdem live mitlaufen.
  html += "<div class='statusGrid' style='margin-top:12px'>";
  html += "<div class='statusBox'><span>Phase</span><b id='centerPhase'>" + String(liveGetCenteringPhaseText()) + "</b></div>";
  html += "<div class='statusBox'><span>RF</span><b id='centerRf'>" + String(liveGetRfVoltage(), 3) + " V</b></div>";
  html += "<div class='statusBox'><span>Winkel</span><b id='centerAngle'>" + String(liveGetRelativeAngleDeg(), 2) + " deg</b></div>";
  html += "<div class='statusBox'><span>Info</span><b id='centerInfo'>" + String(liveGetCenteringInfoText()) + "</b></div>";
  html += "</div>";
  html += "</div>";

  // V3: Die Ausrichten-Seite ist eine reine Bedienseite.
  // Die frueher hier angezeigten Hall-/Rohpin-Diagnosewerte und der
  // separate "Live Status" wurden entfernt, weil sie dem normalen
  // Bediener beim Start der Mittenfahrt nicht helfen und die Seite
  // unnoetig technisch wirken lassen. Diagnosewerte bleiben weiterhin
  // auf der separaten Status-/Diagnose-Seite verfuegbar.
  // V3: Rote Endbereich-Warnung direkt auf der Ausrichten-Seite.
  // Wenn die Anlage manuell oder durch einen vorherigen Lauf am Ost-/West-Endsensor
  // steht, ist die Mittenfahrt bewusst gesperrt. Der Nutzer soll das sofort sehen
  // und ueber Manuell vorsichtig aus dem Endbereich herausfahren.
  if (liveHallEast()) {
    html += "<div id='centerEndNote' class='note warnbox'><b>Azimut-Ost-Endbereich aktiv.</b><br>Ausrichten kann erst wieder starten, wenn die Anlage manuell vorsichtig Richtung West aus dem Endbereich gefahren wurde.</div>";
  } else if (liveHallWest()) {
    html += "<div id='centerEndNote' class='note warnbox'><b>Azimut-West-Endbereich aktiv.</b><br>Ausrichten kann erst wieder starten, wenn die Anlage manuell vorsichtig Richtung Ost aus dem Endbereich gefahren wurde.</div>";
  } else {
    html += "<div id='centerEndNote' class='note'>Wenn die Anlage einen Endbereich erreicht hat, bitte bei Bedarf ueber die manuelle Steuerung vorsichtig in die Gegenrichtung fahren.</div>";
  }
  html += "</div>";

  html += "<script>";
  html += "function setTxt(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}";
  html += "function setCenterHtml(msgClass,msgHtml,buttonsHtml){var m=document.getElementById('centerMsg');var b=document.getElementById('centerButtons');if(m){m.className='note '+msgClass;m.innerHTML=msgHtml;}if(b)b.innerHTML=buttonsHtml;}";
  html += "function centerLimitHtml(d){if(d.hallE&&d.hallW)return '<b>Azimut-Endbereich unklar.</b><br>Ost und West melden gleichzeitig. Bitte Sensoren/Mechanik pruefen.';if(d.hallE)return '<b>Azimut-Ost-Endbereich aktiv.</b><br>Ausrichten kann erst wieder starten, wenn die Anlage manuell vorsichtig Richtung West aus dem Endbereich gefahren wurde.';if(d.hallW)return '<b>Azimut-West-Endbereich aktiv.</b><br>Ausrichten kann erst wieder starten, wenn die Anlage manuell vorsichtig Richtung Ost aus dem Endbereich gefahren wurde.';return 'Wenn die Anlage einen Endbereich erreicht hat, bitte bei Bedarf ueber die manuelle Steuerung vorsichtig in die Gegenrichtung fahren.';}";
  html += "function updateCenterEndNote(d){var n=document.getElementById('centerEndNote');if(!n)return;var lim=d.hallE||d.hallW;n.className=lim?'note warnbox':'note';n.innerHTML=centerLimitHtml(d);}";
  html += "function centerStartButtons(d,label){var lim=d.hallE||d.hallW;var start=lim?'<span class=\"btn disabled\">'+label+'</span>':'<a class=\"btn orange\" href=\"/center/start\">'+label+'</a>';return start+'<a class=\"btn redLight\" href=\"/\">Zurueck</a>'; }";
  html += "function updateCenterStatus(){fetch('/api/center/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{setTxt('centerPhase',d.phase);setTxt('centerInfo',d.centerInfo);setTxt('centerRf',d.rfText);setTxt('centerAngle',d.angleText);updateCenterEndNote(d);if(d.active){setCenterHtml('orangebox','<b>'+d.phase+'</b><br>'+d.centerInfo,'<span class=\"infoPill infoOrange\">Ausrichten laeuft</span><a class=\"btn redLight\" href=\"/center/abort\">ABBRUCH</a>');}else if(d.success){setCenterHtml('successbox','<b>✅ Ausrichtung erfolgreich beendet.</b><br>Die Mitte wurde gesetzt. Die Antenne kann jetzt grob nach Sueden ausgerichtet werden; danach die Suche starten.',centerStartButtons(d,'Ausrichten erneut starten'));}else if(d.failed){setCenterHtml('warnbox','<b>Ausrichten nicht erfolgreich.</b><br>'+d.centerInfo,centerStartButtons(d,'Ausrichten erneut starten'));}else if(d.hallE||d.hallW){setCenterHtml('warnbox','<b>Ausrichten gesperrt: Endbereich aktiv.</b><br>'+centerLimitHtml(d),centerStartButtons(d,'Ausrichten starten'));}else{setCenterHtml('','Bereit zum Ausrichten. Start setzt die Center-/Mittenreferenz.',centerStartButtons(d,'Ausrichten starten'));}}).catch(()=>{});}setInterval(updateCenterStatus,500);updateCenterStatus();";
  html += "</script>";

  html += htmlFooter();
  return html;
}

// Seite "Suchen". Intern bleiben die Routennamen /auto und die Runtime-AUTO-Logik erhalten,
// damit bestehende Links und Handler nicht unnoetig umbenannt werden.
//
// Diese Seite hat mehrere Betriebsbilder:
// - SUCH-SETUP: Winkel korrigieren und Suche starten
// - laufende Suche: Mitte suchen, Ostfahrt, Westfahrt, Rueckfahrt zur Mitte
// - Mitte erreicht: Winkel manuell veraendern und Suchlauf wiederholen
// - Kandidat gefunden: Nutzer entscheidet OK oder FALSCH
// - Fehler / SAT OK / Bereitschaft
//
// Wichtige Bedienphilosophie V3:
// Reine Informationen bleiben als helle Text-/Statusbereiche ohne Button-Rahmen.
// Echte Bedienaktionen sind farbige Buttons mit sichtbarem Rahmen.
// Die Winkel-Ist-/Zielwerte stehen im SUCH-SETUP direkt in den Winkel-Buttons,
// damit keine zusaetzlichen Info-Karten mit Bedienbuttons verwechselt werden.
static String buildAutoPage() {
  const bool mpuReady = liveMpuReady();

  // V3: Web-UI-Vereinfachung fuer die Seite "Suchen".
  // Frueher zeigte /auto zuerst eine Zwischenkarte "SUCHE BEREIT" mit
  // dem Button "SUCH-SETUP OEFFNEN". Das war fuer die Tastenbedienung
  // nachvollziehbar, in der Web-UI aber ein unnoetiges Menue im Menue.
  // Deshalb oeffnet /auto das Such-Setup automatisch, solange keine Suche
  // laeuft, kein Kandidat wartet und kein Abschluss-/Fehlerzustand aktiv ist.
  // Die Runtime-AUTO-Logik bleibt unveraendert; nur der Web-Einstieg wird
  // direkter und damit eindeutiger.
  if (mpuReady &&
      !liveAutoSetupActive() &&
      String(liveGetModeText()) != "AUTO" &&
      !liveIsCandidateHold() &&
      !liveAutoFailed() &&
      !liveIsSatelliteConfirmed()) {
    liveCommandEnterAuto();
  }

  const bool autoSetup = liveAutoSetupActive();
  const bool inAuto = (String(liveGetModeText()) == "AUTO");
  const bool candidate = liveIsCandidateHold();
  const bool autoFailed = liveAutoFailed();
  const bool satOk = liveIsSatelliteConfirmed();
  const String phase = liveGetAutoStateText();
  const String info = liveGetInfoText();
  const String rfQuality = liveGetRfQualityText();

  String html;
  reserveHtml(html, 14000);
  html = htmlHeader("Suchen", "auto");
  html += "<div class='card hero'><div class='title'><h2><span class='step'>2</span>Suchen</h2></div>";
  // V3: Die fruehere Timeline direkt unter der Ueberschrift
  // (Setup / Mitte / Ost-West / Winkel) wurde entfernt. Die noetigen
  // Informationen stehen jetzt kompakt im eigentlichen Statusbereich der
  // jeweiligen Suchseite. Dadurch wirkt die Web-UI ruhiger und zeigt keine
  // doppelte, fuer den Nutzer wenig hilfreiche Statusleiste mehr.

  // Die Such-Seite ist bewusst als Zustandsseite aufgebaut. Es werden nur die
  // Buttons angezeigt, die im aktuellen Zustand wirklich sinnvoll sind.
  // Jeder sichtbare Aktionsbutton loest eine echte Runtime-Funktion aus.
  if (candidate) {
    // V3: Die RF-Ampel bewertet nur die Signalstaerke. Ob es der richtige
    // Satellit ist, entscheidet weiterhin der Nutzer am Receiver/TV.
    // Die fruehere Funktion "Signal optimieren" ist aus der Web-UI entfernt.
    html += "<div class='chips'>" + chip("SAT-KANDIDAT", "blue") + chip("Signal " + rfQuality, rfQualityClass(rfQuality)) + "</div>";
    html += "<div class='pageLead'>Bitte am Receiver/TV pruefen, ob es der richtige Satellit ist. Die RF-Ampel ist nur eine Hilfe; die Entscheidung trifft der Nutzer.</div>";
    html += row("RF-Bewertung", rfQuality + " / " + String(liveGetRfFilteredAdc(), 0) + " ADC");
    html += "<div class='ctrl3' style='margin-top:12px'>";
    html += actionButton("/candidate/ok", "+ OK", true, "green");
    html += actionButton("/candidate/false", "- FALSCH", true, "orange");
    html += actionButton("/auto/abort", "ABBRUCH", true, "red");
    html += "</div>";
    html += "<div class='note'>+ OK bestaetigt den Satelliten und bleibt an der aktuellen Position. - FALSCH sperrt diesen Bereich und die Suche laeuft weiter.</div>";

  } else if (autoSetup) {
    const String ezNow = String(liveGetRelativeAngleDeg(), 1);
    const String ezTarget = String(DEFAULT_TARGET_ELEVATION, 1);
    html += "<div class='chips'>" + chip("SUCH-SETUP", "violet") + chip("Winkel pruefen", "blue") + "</div>";
    html += "<div class='pageLead'>Winkel bei Bedarf kurz korrigieren. Danach startet die Suche ueber Mitte, Osten, Westen und zurueck zur Mitte.</div>";
    // V3: Feinsuche AN/AUS wurde entfernt. Die Suchseite bleibt dadurch
    // einfacher: erst Winkel pruefen, dann Suche starten. Die Winkel-Ist-/Zielwerte
    // stehen direkt in den beiden Korrekturbuttons.
    html += "<div class='infoPanel'><b>Ablauf:</b> Mitte suchen -> Richtung Osten suchen -> Richtung Westen suchen -> zurueck zur Mitte. Danach Winkel leicht veraendern und Suche bei Bedarf wiederholen.</div>";
    html += "<div class='grid2' style='margin-top:12px'>";
    html += actionButton("/auto/start", "SUCHE STARTEN", mpuReady, "green");
    html += actionButton("/auto/abort", "ZURUECK", true, "redLight");
    html += "</div>";
    html += "<div class='grid2' style='margin-top:10px'>";
    html += actionButton("/auto/ez/down", "Winkel -  Ist " + ezNow + " / Ziel " + ezTarget, mpuReady, "orange");
    html += actionButton("/auto/ez/up", "Winkel +  Ist " + ezNow + " / Ziel " + ezTarget, mpuReady, "orange");
    html += "</div>";
    html += "<div class='grid2' style='margin-top:10px'>";
    html += actionButton("/ausrichten", "AUSRICHTEN", true, "primary");
    html += actionButton("/status", "STATUS", true, "gray");
    html += "</div>";
    if (!mpuReady) {
      html += "<div class='note warnbox'>Winkelsensor ist nicht bereit. SUCHE START und Winkelkorrektur sind deshalb gesperrt.</div>";
    }

  } else if (inAuto && !autoFailed && !satOk && phase == "HOEHE PRUEFEN") {
    const String ezNow = String(liveGetRelativeAngleDeg(), 1);
    const String ezTarget = String(DEFAULT_TARGET_ELEVATION, 1);
    html += "<div class='chips'>" + chip("MITTE ERREICHT", "blue") + chip("Winkel pruefen", "warn") + "</div>";
    html += "<div class='pageLead'>Die Anlage steht wieder in der Mitte. Bitte Winkel leicht veraendern, RF-Signal beobachten und die Suche bei Bedarf erneut starten.</div>";
    html += row("RF", String(liveGetRfVoltage(), 3) + " V - " + rfQuality);
    html += row("Winkel", String(liveGetRelativeAngleDeg(), 2) + " deg");
    html += "<div class='grid2' style='margin-top:12px'>";
    html += actionButton("/auto/ez/down", "Winkel -  Ist " + ezNow + " / Ziel " + ezTarget, mpuReady, "orange");
    html += actionButton("/auto/ez/up", "Winkel +  Ist " + ezNow + " / Ziel " + ezTarget, mpuReady, "orange");
    html += "</div>";
    html += "<div class='grid2' style='margin-top:10px'>";
    html += actionButton("/auto/start", "SUCHE WIEDERHOLEN", mpuReady, "green");
    html += actionButton("/auto/abort", "ZURUECK", true, "redLight");
    html += "</div>";

  } else if (inAuto && !autoFailed && !satOk) {
    // V3: Laufende Suche ohne doppelte Kopf-/Timeline-Anzeige.
    // Die Werte in diesem Block werden per /api/auto/status live aktualisiert,
    // damit Phase, Info, RF und Winkel waehrend Mitte, Ostfahrt, Westfahrt
    // und Rueckfahrt zur Mitte sichtbar mitlaufen, ohne die Seite neu zu laden.
    html += "<div class='pageLead'>Die Suche ist aktiv. Die Anlage prueft das Signal waehrend Mitte, Ostfahrt, Westfahrt und Rueckfahrt zur Mitte.</div>";
    html += "<div class='row'><span>Phase</span><b id='autoPhase'>" + phase + "</b></div>";
    html += "<div class='row'><span>Info</span><b id='autoInfo'>" + info + "</b></div>";
    html += "<div class='row'><span>RF</span><b id='autoRf'>" + String(liveGetRfVoltage(), 3) + " V - " + rfQuality + "</b></div>";
    html += "<div class='row'><span>Winkel</span><b id='autoAngle'>" + String(liveGetRelativeAngleDeg(), 2) + " deg</b></div>";
    html += actionButton("/auto/abort", "SUCHE ABBRUCH", true, "red");
    html += "<script>";
    html += "function setTxt(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}";
    html += "function updateAutoStatus(){fetch('/api/auto/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{setTxt('autoPhase',d.phase);setTxt('autoInfo',d.info);setTxt('autoRf',d.rfText);setTxt('autoAngle',d.angleText);if(!d.running){window.location='/auto';}}).catch(()=>{});}";
    html += "setInterval(updateAutoStatus,500);updateAutoStatus();";
    html += "</script>";

  } else if (autoFailed) {
    html += "<div class='chips'>" + chip("SUCHE FEHLER", "bad") + chip(phase, phaseClass(phase)) + "</div>";
    html += "<div class='pageLead'>Die Suche wurde mit Fehler beendet. Details stehen im Status/Diagnose-Bereich und im Serial Monitor.</div>";
    html += row("Fehler", info);
    html += "<div class='grid2' style='margin-top:12px'>";
    html += actionButton("/auto/setup", "SUCH-SETUP NEU", mpuReady, "primary");
    html += actionButton("/status", "DIAGNOSE", true, "gray");
    html += "</div>";

  } else if (satOk) {
    html += "<div class='chips'>" + chip("SAT OK", "good") + chip("Suche beendet", "neutral") + "</div>";
    html += "<div class='pageLead'>Der Satellit wurde bestaetigt. Fuer einen neuen Suchlauf kann das Such-Setup erneut geoeffnet werden.</div>";
    html += "<div class='grid2' style='margin-top:12px'>";
    html += actionButton("/auto/setup", "SUCH-SETUP NEU", mpuReady, "primary");
    html += actionButton("/status", "STATUS", true, "gray");
    html += "</div>";

  } else {
    // Dieser Fallback wird normalerweise nur erreicht, wenn der MPU/Winkelsensor
    // nicht bereit ist und das Such-Setup deshalb nicht automatisch betreten
    // werden darf. Es gibt hier bewusst kein zweites Untermenue mehr.
    html += "<div class='chips'>" + chip("SUCHE NICHT BEREIT", "warn") + chip("Winkel manuell", "blue") + "</div>";
    html += "<div class='pageLead'>Vor der Suche bitte Ausrichten durchfuehren, den Sat-Receiver einschalten und den Winkelsensor pruefen.</div>";
    html += actionButton("/ausrichten", "AUSRICHTEN", true, "primary");
    if (!mpuReady) {
      html += "<div class='note warnbox'>Winkelsensor ist nicht bereit. Die Suche kann deshalb noch nicht gestartet werden.</div>";
    }
  }

  html += "</div>";
  html += rfCard();
  html += htmlFooter();
  return html;
}

// Seite "Manuell".
//
// Die manuelle Steuerung ist bewusst ein Override-Modus. Der Bediener hat hier
// direkte Kontrolle ueber AZ und EZ. Die Web-Buttons senden Start-/Stop-Befehle
// an die Runtime. Die Live-Farben werden ueber /api/manual/status aktualisiert.
// V3: Nicht aktive Fahrtrichtungen sind hellorange, STOP ist hellrot.
// So bleibt auch im Stand klar: Das sind echte Bedienbuttons und keine Infofelder.
static String buildManualPage() {
  const bool azActive = liveWebManualAzActive();
  const bool elActive = liveWebManualElActive();
  const String azDir = liveWebManualAzDirectionText();
  const String elDir = liveWebManualElDirectionText();

  String html;
  reserveHtml(html, 15000);
  html = htmlHeader("Manuell", "manuell");

  html += "<div class='card'><div class='title'><h2><span class='step'>3</span>Manuell Azimut</h2></div>";
  // V3: Der erklaerende Override-Hinweis wurde aus der Bedienansicht entfernt.
  // Die folgenden Status- und Steuerfelder erklaeren die manuelle Azimutbedienung ausreichend.
  html += "<div id='azNote' class='note " + String(azActive ? "warnbox" : "") + "'>";
  html += azActive ? ("<b>Azimut laeuft</b><br>Motor bleibt aktiv, bis STOP gedrueckt wird.") : "Azimut steht. Azimut + oder Azimut - startet die Bewegung.";
  html += "</div>";
  html += "<div class='ctrl3' style='margin-top:12px'>";
  html += manualControlButton("btnAzMinus", "/az/minus", "/api/az/minus", "Azimut -", (azActive && azDir == "AZ-") ? "activeMove" : "orangeLight");
  html += manualControlButton("btnAzStop", "/az/stop", "/api/az/stop", "STOP", azActive ? "activeStop" : "redLight");
  html += manualControlButton("btnAzPlus", "/az/plus", "/api/az/plus", "Azimut +", (azActive && azDir == "AZ+") ? "activeMove" : "orangeLight");
  html += "</div>";
  html += row("Azimut Web-Zustand", azActive ? "LAEUFT" : "STOP");
  html += "<div class='row'><span>Azimut Live</span><b id='azLive'>" + String(azActive ? "LAEUFT" : "STOP") + "</b></div>";
  html += "<div id='hallManualRow' class='row " + hallSensorRowClass() + "'><span>Hall-Sensoren</span><b id='hallManual'>" + hallSensorSummary() + "</b></div>";
  html += "</div>";

  html += "<div class='card'><div class='title'><h2>Manuell Winkel</h2></div>";
  // V3: Der erklaerende Override-Hinweis wurde aus der Bedienansicht entfernt.
  // Die Buttonbeschriftungen mit aktuellem Winkel reichen fuer die Bedienung aus.
  html += "<div id='elNote' class='note " + String(elActive ? "warnbox" : "") + "'>";
  html += elActive ? ("<b>Winkel laeuft</b><br>Motor bleibt aktiv, bis STOP gedrueckt wird.") : "Winkel steht. Winkel + oder Winkel - startet die Bewegung.";
  html += "</div>";
  html += "<div class='ctrl3' style='margin-top:12px'>";
  const String currentAngleButtonText = String(liveGetRelativeAngleDeg(), 1) + " deg";
  // V3: In der manuellen Winkelsteuerung steht der aktuelle Winkel direkt
  // in den beiden Fahrbuttons. Dadurch muss der Nutzer nicht zwischen Button
  // und Statuszeile wechseln, wenn er den Winkel fein nachregelt.
  html += manualControlButton("btnElMinus", "/el/minus", "/api/el/minus", "Winkel -  " + currentAngleButtonText, (elActive && elDir == "EZ-") ? "activeMove" : "orangeLight");
  html += manualControlButton("btnElStop", "/el/stop", "/api/el/stop", "STOP", elActive ? "activeStop" : "redLight");
  html += manualControlButton("btnElPlus", "/el/plus", "/api/el/plus", "Winkel +  " + currentAngleButtonText, (elActive && elDir == "EZ+") ? "activeMove" : "orangeLight");
  html += "</div>";
  html += row("Winkel Web-Zustand", elActive ? "LAEUFT" : "STOP");
  html += "<div class='row'><span>Winkel Live</span><b id='elLive'>" + String(elActive ? "LAEUFT" : "STOP") + "</b></div>";
  html += "<div class='row'><span>Winkel</span><b id='ezLive'>" + String(liveGetRelativeAngleDeg(), 2) + " deg</b></div>";
  html += row("Softlimits", String(ELEVATION_MIN_SOFT, 1) + " bis " + String(ELEVATION_MAX_SOFT, 1) + " deg");
  html += "</div>";

  html += "<script>";
  html += "function setCls(id,cls){var e=document.getElementById(id);if(e)e.className='btn '+cls;}";
  html += "function setText(id,txt){var e=document.getElementById(id);if(e)e.textContent=txt;}";
  html += "function setNote(id,active,dir,axis){var e=document.getElementById(id);if(!e)return;var isH=axis=='Winkel';e.className='note '+(active?'warnbox':'');e.innerHTML=active?('<b>'+axis+' laeuft</b><br>Motor bleibt aktiv, bis STOP gedrueckt wird.'):(isH?'Winkel steht. Winkel + oder Winkel - startet die Bewegung.':axis+' steht. '+axis+'+ oder '+axis+'- startet die Bewegung.');}";
  html += "function applyManualState(d){setCls('btnAzMinus',(d.azActive&&d.azDir=='AZ-')?'activeMove':'orangeLight');setCls('btnAzPlus',(d.azActive&&d.azDir=='AZ+')?'activeMove':'orangeLight');setCls('btnAzStop',d.azActive?'activeStop':'redLight');setCls('btnElMinus',(d.elActive&&d.elDir=='EZ-')?'activeMove':'orangeLight');setCls('btnElPlus',(d.elActive&&d.elDir=='EZ+')?'activeMove':'orangeLight');setCls('btnElStop',d.elActive?'activeStop':'redLight');setText('azLive',d.azActive?'LAEUFT':'STOP');setText('elLive',d.elActive?'LAEUFT':'STOP');var w=Number(d.ez).toFixed(2)+' deg';setText('ezLive',w);setText('btnElMinus','Winkel -  '+Number(d.ez).toFixed(1)+' deg');setText('btnElPlus','Winkel +  '+Number(d.ez).toFixed(1)+' deg');if(d.hallText)setText('hallManual',d.hallText);var hm=document.getElementById('hallManualRow');if(hm&&d.hallClass)hm.className='row '+d.hallClass;setNote('azNote',d.azActive,d.azDir,'Azimut');setNote('elNote',d.elActive,d.elDir,'Winkel');}";
  html += "function optimistic(url){var d={azActive:false,azDir:'STOP',elActive:false,elDir:'STOP',ez:parseFloat((document.getElementById('ezLive')||{}).textContent)||0};if(url.indexOf('/api/az/plus')>=0){d.azActive=true;d.azDir='AZ+';}else if(url.indexOf('/api/az/minus')>=0){d.azActive=true;d.azDir='AZ-';}else if(url.indexOf('/api/el/plus')>=0){d.elActive=true;d.elDir='EZ+';}else if(url.indexOf('/api/el/minus')>=0){d.elActive=true;d.elDir='EZ-';}applyManualState(d);}";
  html += "function updateManualState(){fetch('/api/manual/status',{cache:'no-store'}).then(r=>r.json()).then(applyManualState).catch(()=>{});}";
  html += "function sendManualCmd(url){optimistic(url);fetch(url,{cache:'no-store'}).then(r=>r.json()).then(applyManualState).catch(updateManualState);return false;}";
  html += "document.addEventListener('DOMContentLoaded',function(){updateManualState();setInterval(updateManualState,250);});";
  html += "</script>";
  html += htmlFooter();
  return html;
}

static String diagnoseClass() {
  const float rfV = liveGetRfVoltage();
  if (liveAutoFailed()) return "bad";
  if (liveHallEast() || liveHallWest()) return "warn";
  if (rfV > 0.82f) return "warn";
  if (liveIsCandidateHold()) return "warn";
  return "good";
}

static String diagnoseTitle() {
  const float rfV = liveGetRfVoltage();
  if (liveAutoFailed()) return "Such-Fehler erkannt";
  if (liveHallEast()) return "Azimut Ost-Limit aktiv";
  if (liveHallWest()) return "Azimut West-Limit aktiv";
  if (liveIsCandidateHold()) return "Satellitenkandidat gefunden";
  if (rfV > 0.82f) return "RF-Signal schwach";
  return "System wirkt plausibel";
}

static String diagnoseRecommendation() {
  const float rfV = liveGetRfVoltage();
  if (liveAutoFailed()) return "Fehlertext und Hallwerte pruefen. Danach ggf. manuell korrigieren, Ausrichten erneut starten oder ESP Reset verwenden.";
  if (liveHallEast()) return "Manuell Azimut nach West fahren. Danach Ausrichten erneut starten.";
  if (liveHallWest()) return "Manuell Azimut nach Ost fahren. Danach Ausrichten erneut starten.";
  if (liveIsCandidateHold()) return "Am Receiver/TV pruefen: + OK bei richtigem Satelliten, - FALSCH wenn es nicht Astra ist.";
  if (rfV > 0.82f) return "Sat-Receiver eingeschaltet? RF-Signalweg und Koax-Verbindung pruefen. Bei Hardwareverdacht InstallTest verwenden.";
  return "Keine offensichtliche Stoerung. Fuer die Suche zuerst Ausrichten ausfuehren und Receiver eingeschaltet lassen.";
}

// Hilfe-/Troubleshooting-Seite.
// Diese Seite bleibt bewusst textlastiger als die Bedienseiten, weil sie dem
// Nutzer vor Ort schnelle Pruefpunkte liefern soll: Receiver an, RF plausibel,
// MPU bereit, Hall-Sensoren plausibel, Suche korrekt gestartet.
// V3: Die fruehere Signaloptimierungsseite wurde aus der Bedienung entfernt.
// Troubleshooting bleibt auf Diagnose, manuelle Korrektur und Statuswerte fokussiert.
static String buildTroubleshootingPage() {
  String cls = diagnoseClass();
  String html;
  reserveHtml(html, 11000);
  html = htmlHeader("Troubleshooting", "trouble");
  html += "<div class='card diag " + cls + "'><div class='title'><h2><span class='step'>?</span>Troubleshooting</h2></div>";
  html += "<div class='pageLead'>Diese Seite bewertet die wichtigsten Live-Werte und gibt eine direkte Empfehlung.</div>";
  html += "<div class='chips'>" + chip(diagnoseTitle(), cls) + chip(liveGetAutoStateText(), phaseClass(liveGetAutoStateText())) + "</div>";
  html += row("Moegliche Ursache", diagnoseTitle());
  html += row("Empfehlung", diagnoseRecommendation());
  html += "<div class='note'><b>Hardwaretest:</b><br>Wenn ein einzelnes Bauteil unklar ist, den separaten Sketch <b>SatAlign_ESP32_V3_InstallTest.ino</b> verwenden.</div>";
  html += "<div class='grid2' style='margin-top:12px'>";
  html += actionButton("/status", "Statuswerte", true, "primary");
  html += actionButton("/manuell", "Manuell korrigieren", true, "orange");
  html += "</div>";
  html += "</div>";

  html += "<div class='card'><div class='title'><h2>Schnellpruefung</h2></div>";
  html += row("Receiver", "muss eingeschaltet sein");
  html += row("RF", String(liveGetRfVoltage(), 3) + " V - " + String(liveGetRfQualityText()));
  html += row("Winkel", String(liveGetRelativeAngleDeg(), 2) + " deg");
  // V3: Die Hilfe-Seite zeigt die aktuellen Netzwerkdaten direkt mit an.
  // Dadurch kann der Nutzer die Web-UI-Adresse auch ohne Serial Monitor
  // ablesen bzw. pruefen, ob das ausgewaehlte WLAN und RSSI plausibel sind.
  html += row("WLAN", wifiGetConnectedSsid());
  html += row("IP-Adresse", wifiGetIpString());
  html += row("WLAN-Signal", wifiGetRssiString());
  html += "<div id='hallTroubleRow' class='row " + hallSensorRowClass() + "'><span>Hall-Sensoren</span><b id='hallTrouble'>" + hallSensorSummary() + "</b></div>";
  html += row("Info", liveGetInfoText());
  // V3: Auch im Troubleshooting wird die Hall-Zeile live farblich aktualisiert.
  // Center = hellgruen, Ost/West-Endbereich = hellrot, sonst neutral.
  html += "<script>function updHallTrouble(){fetch('/api/hall/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{var h=document.getElementById('hallTrouble');if(h)h.textContent=d.hallText;var row=document.getElementById('hallTroubleRow');if(row&&d.hallClass)row.className='row '+d.hallClass;}).catch(()=>{});}setInterval(updHallTrouble,500);updHallTrouble();</script>";
  html += "</div>";

  html += "<div class='card'><div class='title'><h2>System / ESP Reset</h2></div>";
  html += "<div class='pageLead'>ESP Reset startet nur den Controller neu. Die Antennenposition wird mechanisch nicht veraendert.</div>";
  html += "<div class='note warnbox'>Vor dem Reset werden Azimut- und Winkelnmotor gestoppt. Danach startet das System wie nach Druecken der Reset-Taste. Bei unklarem Zustand zuerst STOP verwenden, danach bei Bedarf ESP RESET.</div>";
  html += actionButton("/system/reset", "ESP RESET", true, "red");
  html += "</div>";

  html += htmlFooter();
  return html;
}


// V3: Die Web-Seite fuer die fruehere Signaloptimierung wurde entfernt.
// PLUS bestaetigt wieder nur den richtigen Satelliten; eine automatische
// Optimierung wird in dieser Projektlinie bewusst nicht mehr angeboten.

// Status-/Diagnoseseite.
// Hier werden moeglichst viele Roh- und Laufzeitwerte zusammengefuehrt.
// Diese Seite ist fuer Fehlersuche gedacht und soll nicht die normale Bedienung
// ersetzen.
static String buildStatusPage() {
  String html;
  reserveHtml(html, 11000);
  html = htmlHeader("Status", "status");
  html += rfCard();
  html += "<div class='card'><div class='title'><h2><span class='step'>4</span>Status / Diagnose</h2></div>";
  html += row("Modus", liveGetModeText());
  html += row("Achse", liveGetAxisText());
  html += row("Suchen", liveGetAutoStateText());
  html += row("Azimut", liveGetAzimuthStateText());
  html += row("Winkel", liveGetElevationStateText());
  html += row("Info", liveGetInfoText());
  html += row("WLAN", wifiGetConnectedSsid());
  html += row("IP-Adresse", wifiGetIpString());
  html += row("WLAN-Signal", wifiGetRssiString());
  html += row("Winkel", String(liveGetRelativeAngleDeg(), 2) + " deg");
  html += row("MPU gefiltert", String(liveGetFilteredAngleDeg(), 2) + " deg");
  html += row("RAW ADC", String(liveGetRfRawAdc()));
  html += row("Filtered ADC", String(liveGetRfFilteredAdc(), 1));
  html += "<div id='hallStatusRow' class='row " + hallSensorRowClass() + "'><span>Hall-Sensoren</span><b id='hallStatus'>" + hallSensorSummary() + "</b></div>";
  html += "<div class='row'><span>Hall-Rohpins</span><b id='hallRawStatus'>" + hallRawSummary() + "</b></div>";
  html += "<script>function updHall(){fetch('/api/hall/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{var h=document.getElementById('hallStatus');if(h)h.textContent=d.hallText;var row=document.getElementById('hallStatusRow');if(row&&d.hallClass)row.className='row '+d.hallClass;var hr=document.getElementById('hallRawStatus');if(hr)hr.textContent=d.hallRawText;}).catch(()=>{});}setInterval(updHall,500);updHall();</script>";
  html += row("Blocked Sat Bereiche", String(liveGetBlockedRangeCount()));
  html += "<div class='note'>Diese Seite zeigt bewusst mehr Diagnosewerte als das TFT.</div>";
  html += actionButton("/trouble", "Troubleshooting", true, "orange");
  html += "</div>";

  html += "<div class='card'><div class='title'><h2>System</h2></div>";
  html += "<div class='pageLead'>ESP Reset befindet sich jetzt auf der Hilfe-/Troubleshooting-Seite, damit Reset und Fehlerhilfe zusammengehoeren.</div>";
  html += actionButton("/trouble", "Hilfe / ESP RESET", true, "primary");
  html += "</div>";
  html += htmlFooter();
  return html;
}

// Sicherheitsseite fuer ESP-Reset.
// Der Reset wird nicht direkt aus der Navigation ausgefuehrt, sondern erst nach
// einer Bestaetigungsseite. So wird ein versehentlicher Neustart waehrend einer
// Ausrichtung vermieden.
static String buildResetConfirmPage() {
  String html;
  reserveHtml(html, 8000);
  html = htmlHeader("ESP Reset", "trouble");
  html += "<div class='card'><div class='title'><h2>ESP Reset bestaetigen</h2></div>";
  html += "<div class='note warnbox'><b>Der ESP32 wird neu gestartet.</b><br>Die Antennenposition wird mechanisch nicht zurueckgesetzt. Motoren werden vor dem Reset gestoppt. Danach startet das System wie nach Druecken der Reset-Taste.</div>";
  html += "<div class='grid2' style='margin-top:12px'>";
  html += actionButton("/system/reset/confirm", "RESET JETZT", true, "red");
  html += actionButton("/trouble", "ABBRECHEN", true, "gray");
  html += "</div></div>";
  html += htmlFooter();
  return html;
}

// JSON-Hilfsfunktionen fuer die kleinen Live-Status-APIs.
// Es wird bewusst manuell JSON gebaut, um keine weitere Bibliothek zu benoetigen.
static String jsonBool(bool v) {
  return v ? "true" : "false";
}

static String jsonEsc(const String& in) {
  String s = in;
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  s.replace("\n", " ");
  s.replace("\r", " ");
  return s;
}

// JSON-API fuer die Ausrichten-Seite.
// Wird zyklisch vom Browser abgefragt, damit der Nutzer Fortschritt, Erfolg oder
// Fehler der Center-/Mittenfahrt ohne kompletten Reload sieht.
static void sendCenterStatusJson() {
  const bool active = liveCenteringActive();
  const bool success = liveCenteringSuccessNoticeActive();
  const String info = liveGetInfoText();
  const String mode = liveGetModeText();
  const bool failed = (!active && !success && (info.indexOf("FEHL") >= 0 || info.indexOf("BLOCK") >= 0));

  String json = "{";
  json += "\"active\":" + jsonBool(active) + ",";
  json += "\"success\":" + jsonBool(success) + ",";
  json += "\"failed\":" + jsonBool(failed) + ",";
  json += "\"mode\":\"" + jsonEsc(mode) + "\",";
  json += "\"info\":\"" + jsonEsc(info) + "\",";
  json += "\"phase\":\"" + jsonEsc(liveGetCenteringPhaseText()) + "\",";
  json += "\"centerInfo\":\"" + jsonEsc(liveGetCenteringInfoText()) + "\",";
  json += "\"rfText\":\"" + jsonEsc(String(liveGetRfVoltage(), 3) + " V / RAW " + String(liveGetRfRawAdc()) + " / FILT " + String(liveGetRfFilteredAdc(), 1)) + "\",";
  json += "\"angleText\":\"" + jsonEsc(String(liveGetRelativeAngleDeg(), 2) + " deg") + "\",";
  json += "\"hallC\":" + jsonBool(liveHallCenter()) + ",";
  json += "\"hallE\":" + jsonBool(liveHallEast()) + ",";
  json += "\"hallW\":" + jsonBool(liveHallWest()) + ",";
  json += "\"hallText\":\"" + jsonEsc(hallSensorSummary()) + "\",";
  json += "\"hallRawText\":\"" + jsonEsc(hallRawSummary()) + "\",";
  json += "\"hallClass\":\"" + hallSensorRowClass() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// JSON-API fuer die Suchseite.
// V3: Liefert nur die kleinen Livewerte der laufenden Suche. Dadurch kann
// die Web-UI Phase, Info, RF und Winkel aktualisieren, ohne die komplette Seite
// neu zu laden. Die Motor-/Suchlogik bleibt weiterhin ausschliesslich in
// live_runtime.cpp.
static void sendAutoStatusJson() {
  const bool inAuto = (String(liveGetModeText()) == "AUTO");
  const bool running = inAuto && !liveAutoSetupActive() && !liveAutoFailed() && !liveIsSatelliteConfirmed() && !liveIsCandidateHold();
  const String phase = liveGetAutoStateText();
  const String info = liveGetInfoText();
  const String rfText = String(liveGetRfVoltage(), 3) + " V - " + String(liveGetRfQualityText());
  const String angleText = String(liveGetRelativeAngleDeg(), 2) + " deg";

  String json = "{";
  json += "\"running\":" + jsonBool(running) + ",";
  json += "\"phase\":\"" + jsonEsc(phase) + "\",";
  json += "\"info\":\"" + jsonEsc(info) + "\",";
  json += "\"rfText\":\"" + jsonEsc(rfText) + "\",";
  json += "\"angleText\":\"" + jsonEsc(angleText) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// V3: Die JSON-API der frueheren Signaloptimierungsseite wurde entfernt.
// Die normalen Livewerte fuer Suche, Manuell und Hall-Sensoren bleiben erhalten.

// JSON-API fuer die manuelle Seite.
// Liefert nur kompakte Statuswerte, damit JavaScript die Buttonfarben und die
// angezeigte EZ-Position aktualisieren kann. Motorlogik bleibt in der Runtime.
static void sendManualStatusJson() {
  String json = "{";
  json += "\"azActive\":" + jsonBool(liveWebManualAzActive()) + ",";
  json += "\"azDir\":\"" + String(liveWebManualAzDirectionText()) + "\",";
  json += "\"elActive\":" + jsonBool(liveWebManualElActive()) + ",";
  json += "\"elDir\":\"" + String(liveWebManualElDirectionText()) + "\",";
  json += "\"ez\":" + String(liveGetRelativeAngleDeg(), 2) + ",";
  json += "\"hallC\":" + jsonBool(liveHallCenter()) + ",";
  json += "\"hallE\":" + jsonBool(liveHallEast()) + ",";
  json += "\"hallW\":" + jsonBool(liveHallWest()) + ",";
  json += "\"hallText\":\"" + jsonEsc(hallSensorSummary()) + "\",";
  json += "\"hallRawText\":\"" + jsonEsc(hallRawSummary()) + "\",";
  json += "\"hallClass\":\"" + hallSensorRowClass() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}


// JSON-API fuer Hall-Sensoren.
// V3: Diese API liefert die logischen Hall-Sensoren Center/Ost/West
// und zusaetzlich die Rohpins. Sie ist bewusst getrennt von Motorlogik,
// damit Status-/Troubleshooting-Seiten die Anzeige live aktualisieren
// koennen, ohne eine Bewegung auszulösen.
static void sendHallStatusJson() {
  String json = "{";
  json += "\"hallText\":\"" + jsonEsc(hallSensorSummary()) + "\",";
  json += "\"hallRawText\":\"" + jsonEsc(hallRawSummary()) + "\",";
  json += "\"hallClass\":\"" + hallSensorRowClass() + "\",";
  json += "\"hallC\":" + jsonBool(liveHallCenter()) + ",";
  json += "\"hallE\":" + jsonBool(liveHallEast()) + ",";
  json += "\"hallW\":" + jsonBool(liveHallWest()) + ",";
  json += "\"rawC\":" + jsonBool(liveRawHallCenter()) + ",";
  json += "\"rawE\":" + jsonBool(liveRawHallEast()) + ",";
  json += "\"rawW\":" + jsonBool(liveRawHallWest());
  json += "}";
  server.send(200, "application/json", json);
}

static void sendHtml(const String& html) {
  // V3: Schutz gegen leere Browserantworten.
  // Falls eine Seite wegen Speicherproblemen unerwartet leer waere, senden wir
  // eine minimale Notfallseite statt die Verbindung leer zu beenden.
  if (html.length() == 0) {
    Serial.println("WEB V3 WARNUNG: HTML-Seite leer - sende Notfallseite.");
    server.send(200, "text/html", "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>SatAlign</title></head><body style='font-family:Arial;padding:20px'><h2>SatAlign Web-UI</h2><p>Die Seite konnte nicht vollstaendig aufgebaut werden. Bitte Seite neu laden oder Serial Monitor pruefen.</p></body></html>");
    return;
  }
  Serial.print("WEB V3: sende HTML, Bytes=");
  Serial.println(html.length());
  server.send(200, "text/html", html);
}

static void redirectTo(const String& path) {
  server.sendHeader("Location", path);
  server.send(303);
}

// HTTP-Handler: jeder Handler fuehrt genau einen Runtime-Befehl aus
// und leitet danach wieder auf die passende Web-Seite um.
//
// Wichtig fuer Wartung:
// In diesen Handlern soll keine eigene Motor-, RF- oder AUTO-Logik entstehen.
// Der Handler ist nur Adapter: HTTP-Klick -> liveCommand...() -> Redirect.
// Dadurch bleibt live_runtime.cpp die einzige fachliche Wahrheit.
static void handleRoot() {
  Serial.println("WEB V3: GET /");

  // V3: Web-UI und TFT sollen denselben Bedienzustand zeigen.
  // Wenn der Nutzer im Browser auf "Menue" bzw. die Startseite klickt,
  // wird deshalb auch die Runtime in das echte Hauptmenue gesetzt.
  // Vorher laufende Web-/AUTO-/Manuell-Aktionen werden dabei sauber gestoppt.
  liveCommandOpenMainMenu();

  sendHtml(buildHomePage());
}

static void handleAusrichtenPage() {
  Serial.println("WEB V3: GET /ausrichten");

  // V3: Web-UI und TFT-Menue werden hier bewusst synchronisiert.
  // Wenn der Nutzer im Browser den Menuepunkt "Ausrichten" oeffnet, soll
  // das TFT nicht im vorherigen Menue stehen bleiben, sondern dieselbe
  // Ausrichten-/Mitte-Seite zeigen wie bei der lokalen Tastenbedienung.
  //
  // Wichtig: liveCommandStartCentering() bereitet nur Menuepunkt 1 vor.
  // Die echte Center-/Mittenfahrt startet dadurch NICHT automatisch.
  // Erst der Web-Button "Ausrichten starten" ruft /center/start auf und
  // loest liveCommandStartCenterRun() aus. Dadurch bleibt das reine Oeffnen
  // der Seite sicher und bewegt keinen Motor.
  // V3: Nur das Oeffnen der Web-Seite soll das TFT in den
  // Ausrichten-/Mitte-Menuepunkt bringen. Es darf aber keine laufende
  // oder fehlgeschlagene Mittenfahrt durch einen Seiten-Reload zuruecksetzen.
  // Deshalb wird das Center-Menue nur vorbereitet, wenn wir nicht bereits
  // im Center-Menue sind. Der eigentliche Start bleibt exklusiv bei
  // /center/start bzw. beim lokalen MODE-Start am Geraet.
  const String modeText = String(liveGetModeText());

  // V3: Wenn der Nutzer aus der Web-UI heraus den Menuepunkt
  // "Ausrichten" oeffnet, soll wirklich die Ausrichten-Seite aktiv sein.
  // Falls zuvor eine Suche/AUTO-Mitttenfahrt lief, darf deren interner
  // Center-Zustand nicht als "Ausrichten laeuft" auf dieser Seite erscheinen.
  // Deshalb wird bei einem Wechsel aus anderen Modi zuerst die Suche sauber
  // abgebrochen und danach nur das Ausrichten-Menue vorbereitet.
  // Ein Reload innerhalb der bereits offenen MITTE-Seite bleibt dagegen
  // unangetastet, damit eine echte Mittenfahrt nicht versehentlich gestoppt wird.
  if (modeText != "MITTE") {
    if (modeText == "AUTO") {
      liveCommandAbortAuto();
    }
    liveCommandStartCentering();
  }

  sendHtml(buildAusrichtenPage());
}

static void handleAutoPage() {
  Serial.println("WEB V3: GET /auto");

  // V3: Die automatische Signaloptimierung ist aus dieser Bedienlinie entfernt.
  // Die Suche bleibt bewusst bei Kandidat, OK/Falsch und manueller
  // Winkelpruefung. PLUS bestaetigt den Kandidaten und behaelt die Position.

  // V3: Beim Oeffnen der Web-Seite "Suchen" soll auch das TFT das
  // Such-Setup anzeigen. Gleichzeitig darf eine laufende Suche NICHT durch
  // blosses Neuladen der Seite zurueckgesetzt werden. Deshalb wird das Setup
  // nur aktiviert, wenn weder AUTO laeuft noch das Setup bereits aktiv ist.
  const String modeText = String(liveGetModeText());
  if (modeText != "AUTO" && !liveAutoSetupActive()) {
    liveCommandEnterAuto();
  }

  sendHtml(buildAutoPage());
}

static void handleManualPage() {
  Serial.println("WEB V3: GET /manuell");

  // V3: Web-UI und TFT synchronisieren.
  // Bisher hat /manuell teilweise nur die Browserseite geliefert. Das TFT blieb
  // dann im vorherigen Menue stehen, obwohl der Nutzer im Web bereits "Manuell"
  // geoeffnet hatte. Ab jetzt setzt das reine Oeffnen der Seite den Runtime-
  // Modus ebenfalls auf MANUELL. Die zuletzt gewaehlte Achse bleibt erhalten;
  // die eigentlichen Fahrbefehle setzen AZ/Winkel bei Bedarf explizit.
  liveCommandEnterManual();

  sendHtml(buildManualPage());
}
static void handleStatusPage() {
  Serial.println("WEB V3: GET /status");

  // V3: Fuer Status/Diagnose gibt es bewusst keinen eigenen TFT-Modus.
  // Diese Seite ist eine reine Web-Diagnoseseite. Sie soll deshalb die
  // aktuelle TFT-Bedienseite nicht ueberschreiben. So bleibt z. B. bei einer
  // laufenden Suche oder manuellen Bedienung das passende TFT-Menue sichtbar.
  sendHtml(buildStatusPage());
}

static void handleTroublePage() {
  Serial.println("WEB V3: GET /trouble");

  // V3: Auch die Hilfe-/Troubleshooting-Seite ist nur Web-Information.
  // Kein TFT-Umschalten, damit der zuletzt gewaählte Bedienmodus sichtbar
  // bleibt.
  sendHtml(buildTroubleshootingPage());
}
static void handleResetConfirmPage() { Serial.println("WEB V3: GET /system/reset"); sendHtml(buildResetConfirmPage()); }

static void handleModeManual() { liveCommandEnterManual(); redirectTo("/manuell"); }
static void handleCenterStart() { liveCommandStartCenterRun(); redirectTo("/ausrichten"); }
static void handleCenterAbort() { liveCommandAbortCentering(); redirectTo("/"); }

static void handleAutoSetup() { liveCommandEnterAuto(); redirectTo("/auto"); }
static void handleAutoToggleFine() {
  // V3: Altlink bleibt absichtlich kompatibel, schaltet aber nichts mehr um.
  // Die fruehere Feinsuche wurde aus der Bedienlogik entfernt; die Winkel wird
  // vor oder nach dem Suchlauf manuell ueber die Winkel-Buttons korrigiert.
  redirectTo("/auto");
}
static void handleAutoStart() { liveCommandStartAutoFromSetup(); redirectTo("/auto"); }
static void handleAutoAbort() {
  // V3: Nach ZURUECK/ABBRUCH nicht wieder nach /auto umleiten.
  // Da /auto jetzt direkt das Such-Setup oeffnet, wuerde ein Redirect nach
  // /auto sofort wieder in die Suche-Seite springen. Zurueck/Abbruch soll
  // den Nutzer eindeutig ins Hauptmenue fuehren.
  liveCommandAbortAuto();
  redirectTo("/");
}
static void handleAutoEzUp() { liveCommandSetupElevationUpPulse(); redirectTo("/auto"); }
static void handleAutoEzDown() { liveCommandSetupElevationDownPulse(); redirectTo("/auto"); }

static void handleAzPlus() { liveCommandAzButtonPlus(); redirectTo("/manuell"); }
static void handleAzMinus() { liveCommandAzButtonMinus(); redirectTo("/manuell"); }
static void handleAzEast() { liveCommandAzEast(); redirectTo("/manuell"); }
static void handleAzWest() { liveCommandAzWest(); redirectTo("/manuell"); }
static void handleAzStop() { liveCommandAzStop(); redirectTo("/manuell"); }

static void handleElPlus() { liveCommandElButtonPlus(); redirectTo("/manuell"); }
static void handleElMinus() { liveCommandElButtonMinus(); redirectTo("/manuell"); }
static void handleElUp() { liveCommandElUp(); redirectTo("/manuell"); }
static void handleElDown() { liveCommandElDown(); redirectTo("/manuell"); }
static void handleElStop() { liveCommandElStop(); redirectTo("/manuell"); }

static void handleApiCenterStatus() { sendCenterStatusJson(); }
static void handleApiAutoStatus() { sendAutoStatusJson(); }
static void handleApiManualStatus() { sendManualStatusJson(); }
static void handleApiHallStatus() { sendHallStatusJson(); }
static void handleApiAzPlus() { liveCommandAzButtonPlus(); sendManualStatusJson(); }
static void handleApiAzMinus() { liveCommandAzButtonMinus(); sendManualStatusJson(); }
static void handleApiAzStop() { liveCommandAzStop(); sendManualStatusJson(); }
static void handleApiElPlus() { liveCommandElButtonPlus(); sendManualStatusJson(); }
static void handleApiElMinus() { liveCommandElButtonMinus(); sendManualStatusJson(); }
static void handleApiElStop() { liveCommandElStop(); sendManualStatusJson(); }

static void handleSystemResetExecute() {
  Serial.println("WEB V3: ESP Reset angefordert. Motoren stoppen, Rueckkehrseite senden und Controller neu starten.");
  azimuthStop();
  elevationStop();

  // V3: Diese Seite hat nur einen Zweck: Der Browser soll nach einem
  // Web-Reset nicht auf einer leeren/alten Seite stehen bleiben.
  // Ablauf:
  // 1) Motoren stoppen.
  // 2) Diese Warteseite an den Browser senden.
  // 3) ESP32 neu starten.
  // 4) Die Warteseite wartet laenger als das Boot-/Winkel-Startfenster und
  //    prueft danach wiederholt, ob die Standardseite / wieder erreichbar ist.
  // Dadurch ist die Rueckkehr deutlich robuster als ein einmaliger Meta-Refresh,
  // weil der ESP32 waehrend Neustart, Winkel-Startfenster und WLAN-Verbindung
  // fuer einige Sekunden keine HTTP-Anfragen beantworten kann.
  String html;
  html.reserve(2600);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  html += F("<title>ESP Reset</title>");
  html += F("<style>body{font-family:Arial,sans-serif;margin:0;padding:24px;background:#eef7ff;color:#18324a}");
  html += F(".card{max-width:560px;margin:40px auto;background:white;border:3px solid #5aa7df;border-radius:18px;padding:22px;box-shadow:0 8px 24px rgba(0,0,0,.12)}");
  html += F("h2{margin:0 0 12px;color:#0f5f99}.note{background:#fff3cd;border:2px solid #f0b429;border-radius:12px;padding:12px;margin-top:14px}");
  html += F(".ok{background:#e8f6ff;border:2px solid #5aa7df;border-radius:12px;padding:12px;margin-top:14px}.btn{display:inline-block;margin-top:16px;padding:12px 16px;background:#1f7ab8;color:white;text-decoration:none;border-radius:12px;font-weight:bold}</style>");
  html += F("<script>");
  html += F("let s=22;function tick(){const e=document.getElementById('count');if(e)e.textContent=s;if(s>0){s--;setTimeout(tick,1000);}else{tryHome();}}");
  html += F("function tryHome(){const st=document.getElementById('state');if(st)st.textContent='Standardseite wird gesucht...';fetch('/?check='+Date.now(),{cache:'no-store'}).then(r=>{if(r.ok)location.replace('/?r='+Date.now());else setTimeout(tryHome,2000);}).catch(()=>setTimeout(tryHome,2000));}");
  html += F("window.addEventListener('load',tick);");
  html += F("</script>");
  html += F("</head><body><div class='card'>");
  html += F("<h2>Anlage startet neu...</h2>");
  html += F("<p>Motoren wurden gestoppt. Die Anlage fuehrt jetzt einen Reset aus.</p>");
  html += F("<div class='note'>Bitte warten. Die Standardseite wird automatisch geoeffnet, sobald die Anlage nach dem Neustart wieder erreichbar ist.</div>");
  html += F("<div class='ok'>Noch ca. <b><span id='count'>22</span> s</b>. <span id='state'>Anlage startet...</span></div>");
  html += F("<a class='btn' href='/'>Standardseite jetzt oeffnen</a>");
  html += F("</div></body></html>");

  server.send(200, "text/html", html);
  // V3: Kurze Wartezeit, damit der Browser die HTML-Antwort sicher
  // empfangen kann, bevor der Controller neu startet.
  delay(1500);
  ESP.restart();
}


static void handleCandidateOk() { liveCommandConfirmSatellite(); redirectTo("/auto"); }
static void handleCandidateFalse() { liveCommandCandidateFalse(); redirectTo("/auto"); }

static void handleNotFound() {
  // V3: Auch unbekannte Pfade liefern eine kleine HTML-Antwort.
  // Damit fuehren z. B. Browseranfragen nach /favicon.ico nicht zu einer
  // irritierenden leeren Seite.
  Serial.print("WEB V3: 404 fuer ");
  Serial.println(server.uri());
  server.send(404, "text/html", "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>SatAlign 404</title></head><body style='font-family:Arial;padding:20px'><h2>SatAlign</h2><p>Diese Web-UI-Seite gibt es nicht.</p><p><a href='/'>Zum Hauptmenue</a></p></body></html>");
}

// Registriert alle HTTP-Routen und startet den Webserver.
//
// Routenstruktur:
// - normale Seiten liefern HTML
// - Aktionsrouten senden genau einen Runtime-Befehl und leiten zurueck
// - /api/... Routen liefern JSON fuer Live-Aktualisierung
// - Kandidatenrouten bilden PLUS/MINUS aus der AUTO-Kandidatenlogik ab
void webServerInit() {
  server.on("/", handleRoot);
  server.on("/ausrichten", handleAusrichtenPage);
  server.on("/auto", handleAutoPage);
  server.on("/manuell", handleManualPage);
  server.on("/status", handleStatusPage);
  server.on("/trouble", handleTroublePage);
  server.on("/system/reset", handleResetConfirmPage);
  server.on("/system/reset/confirm", handleSystemResetExecute);

  server.on("/mode/manual", handleModeManual);
  server.on("/center/start", handleCenterStart);
  server.on("/center/abort", handleCenterAbort);

  server.on("/auto/setup", handleAutoSetup);
  server.on("/mode/auto", handleAutoSetup);       // Altlink bleibt kompatibel.
  server.on("/auto/toggleFine", handleAutoToggleFine); // V3: nur noch Altlink/No-op, keine Feinsuche mehr.
  server.on("/auto/start", handleAutoStart);
  server.on("/auto/abort", handleAutoAbort);
  server.on("/auto/ez/up", handleAutoEzUp);
  server.on("/auto/ez/down", handleAutoEzDown);

  server.on("/az/plus", handleAzPlus);
  server.on("/az/minus", handleAzMinus);
  server.on("/az/east", handleAzEast);
  server.on("/az/west", handleAzWest);
  server.on("/az/stop", handleAzStop);

  server.on("/el/plus", handleElPlus);
  server.on("/el/minus", handleElMinus);
  server.on("/el/up", handleElUp);
  server.on("/el/down", handleElDown);
  server.on("/el/stop", handleElStop);

  server.on("/api/center/status", handleApiCenterStatus);
  server.on("/api/auto/status", handleApiAutoStatus);
  server.on("/api/manual/status", handleApiManualStatus);
  server.on("/api/hall/status", handleApiHallStatus);
  server.on("/api/az/plus", handleApiAzPlus);
  server.on("/api/az/minus", handleApiAzMinus);
  server.on("/api/az/stop", handleApiAzStop);
  server.on("/api/el/plus", handleApiElPlus);
  server.on("/api/el/minus", handleApiElMinus);
  server.on("/api/el/stop", handleApiElStop);

  server.on("/candidate/ok", handleCandidateOk);
  server.on("/candidate/false", handleCandidateFalse);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.print("Webserver V3 gestartet auf Port ");
  Serial.println(WEB_SERVER_PORT);
}

// Muss in loop() regelmaessig laufen.
// Der Arduino WebServer arbeitet nicht im Hintergrund-Thread. Ohne diesen Aufruf
// werden Browseranfragen, Buttonklicks und API-Abfragen nicht verarbeitet.
void webServerLoop() {
  server.handleClient();
}
