/*
  SatAlign V3 - WLAN-Verbindung
  ------------------------------------------------------------
  Baut die WLAN-Verbindung auf und sorgt dafuer, dass Web-UI und OTA im
  lokalen Netz erreichbar werden.

  V3: Die Zugangsdaten liegen nicht im GitHub-Code, sondern lokal in
  secrets.h. Dort koennen mehrere bekannte Netzwerke hinterlegt werden.

  Aktueller Stand V3:
  - Die Anlage scannt die sichtbaren WLANs.
  - Die Reihenfolge in WIFI_NETWORKS[] ist eine feste Prioritaet.
    GeniusBulli steht lokal an erster Stelle und wird deshalb bevorzugt.
  - GeniusBulli verwendet die feste IP 192.168.4.15.
  - Andere bekannte Netze (z. B. GeniusHome) verwenden DHCP.
  - Im laufenden Betrieb erfolgt ein Reconnect nicht-blockierend, damit TFT,
    Tasten, Motoren, Suche und lokale Bedienung weiterlaufen.
*/

#include <WiFi.h>
#include <Arduino.h>

#include "wifi_config.h"
#include "wifi_manager.h"

namespace {

// V3: Pro Netzwerk nur begrenzt warten, damit die Anlage beim Boot nicht
// minutenlang an einem nicht erreichbaren WLAN haengen bleibt.
static const int WIFI_ATTEMPTS_PER_NETWORK = 24;  // 24 * 500 ms = ca. 12 s

// V3: Reconnect im laufenden Betrieb darf Motor-, Such- und Tastenlogik nicht
// blockieren. Deshalb wird nach einem WLAN-Abbruch nicht mehr mit delay()
// durch alle Netzwerke gewartet, sondern stueckweise im loop() gearbeitet.
static const unsigned long WIFI_RECONNECT_INTERVAL_MS = 5000UL;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 12000UL;
static const unsigned long WIFI_SCAN_RETRY_INTERVAL_MS = 8000UL;
static const unsigned long WIFI_SCAN_TIMEOUT_MS = 10000UL;

// V3: WiFi.RSSI() liefert negative dBm-Werte. Je naeher an 0, desto besser.
// -45 dBm ist also staerker als -75 dBm. Dieser Startwert bedeutet: unbekannt.
static const int WIFI_RSSI_UNKNOWN = -999;

// V3: Merkt sich das zuletzt erfolgreiche Netzwerk. Bei einem Reconnect wird
// erneut nach bekannten Netzen in Prioritaetsreihenfolge gescannt; falls kein
// Scan-Ergebnis vorliegt,
// dient dieser Index als sinnvolle Rueckfallloesung.
static int currentNetworkIndex = -1;
static int currentNetworkRssi = WIFI_RSSI_UNKNOWN;

// V3: Zustandswerte fuer den nicht-blockierenden Reconnect.
static bool reconnectConnecting = false;
static bool reconnectScanRunning = false;
static bool disconnectNoticePrinted = false;
static int reconnectIndex = 0;
static int reconnectConnectingIndex = -1;
static unsigned long reconnectStartMs = 0;
static unsigned long reconnectScanStartMs = 0;
static unsigned long nextReconnectAttemptMs = 0;

static bool validNetworkIndex(int index) {
  return index >= 0 && index < WIFI_NETWORK_COUNT;
}

static String rssiText(int rssi) {
  if (rssi <= WIFI_RSSI_UNKNOWN / 2) {
    return String("nicht gefunden");
  }
  return String(rssi) + " dBm";
}

// V3: Auswertung eines WLAN-Scans. Es werden nur Netzwerke beruecksichtigt,
// die in secrets.h in WIFI_NETWORKS[] eingetragen sind. Die Reihenfolge der
// Eintraege ist die feste Prioritaet: Der erste sichtbare bekannte Eintrag wird
// verwendet. So bleibt GeniusBulli auch dann das bevorzugte Netz, wenn am
// Testort gleichzeitig GeniusHome mit hoeherem RSSI sichtbar ist.
static int bestKnownNetworkFromScanResult(int scanCount, bool printResult) {
  int bestIndex = -1;
  int bestRssi = WIFI_RSSI_UNKNOWN;

  if (printResult) {
    Serial.println("WLAN V3: Scan bekannter Netzwerke:");
  }

  for (int known = 0; known < WIFI_NETWORK_COUNT; known++) {
    int foundRssi = WIFI_RSSI_UNKNOWN;

    for (int i = 0; i < scanCount; i++) {
      if (WiFi.SSID(i) == String(WIFI_NETWORKS[known].ssid)) {
        const int rssi = WiFi.RSSI(i);
        if (rssi > foundRssi) {
          foundRssi = rssi;
        }
      }
    }

    if (printResult) {
      Serial.print("  ");
      Serial.print(known + 1);
      Serial.print("/");
      Serial.print(WIFI_NETWORK_COUNT);
      Serial.print(" ");
      Serial.print(WIFI_NETWORKS[known].ssid);
      Serial.print(" -> ");
      Serial.println(rssiText(foundRssi));
    }

    // Feste Prioritaet: Sobald der erste sichtbare bekannte Eintrag gefunden
    // wurde, bleibt er ausgewaehlt. RSSI wird weiterhin fuer Diagnose angezeigt.
    if (bestIndex < 0 && foundRssi > WIFI_RSSI_UNKNOWN / 2) {
      bestRssi = foundRssi;
      bestIndex = known;
    }
  }

  if (printResult) {
    if (validNetworkIndex(bestIndex)) {
      Serial.print("WLAN V3: bevorzugtes sichtbares Netzwerk: ");
      Serial.print(WIFI_NETWORKS[bestIndex].ssid);
      Serial.print(" (");
      Serial.print(bestRssi);
      Serial.println(" dBm)");
    } else {
      Serial.println("WLAN V3: kein bekanntes Netzwerk im Scan gefunden.");
    }
  }

  return bestIndex;
}

// V3: Blockierender Startscan nur in wifiInit(). Zu diesem Zeitpunkt laeuft
// noch keine Motorfahrt; deshalb ist ein kurzer Scan vertretbar und verbessert
// die Auswahl des ersten sichtbaren bekannten Netzes gemaess Prioritaetsliste.
static int scanBestKnownNetworkBlocking() {
  Serial.println("WLAN V3: scanne sichtbare Netzwerke...");
  const int scanCount = WiFi.scanNetworks(false, true);

  if (scanCount <= 0) {
    Serial.println("WLAN V3: Scan ohne Ergebnis.");
    WiFi.scanDelete();
    return -1;
  }

  const int bestIndex = bestKnownNetworkFromScanResult(scanCount, true);
  WiFi.scanDelete();
  return bestIndex;
}

// Setzt vor jedem Verbindungsversuch die passende IP-Konfiguration.
// GeniusBulli: feste IP 192.168.4.15 / Gateway 192.168.4.1 /24.
// Alle anderen Netze: DHCP. Das explizite Zurueckschalten auf DHCP ist wichtig,
// wenn der ESP32 zuvor im Bulli-Netz mit statischer IP verbunden war.
static bool configureIpForNetwork(int index) {
  if (!validNetworkIndex(index)) {
    return false;
  }

  const bool isBulli = String(WIFI_NETWORKS[index].ssid) == String(WIFI_BULLI_SSID);

  if (isBulli) {
    const IPAddress localIp(WIFI_BULLI_IP[0], WIFI_BULLI_IP[1], WIFI_BULLI_IP[2], WIFI_BULLI_IP[3]);
    const IPAddress gateway(WIFI_BULLI_GATEWAY[0], WIFI_BULLI_GATEWAY[1], WIFI_BULLI_GATEWAY[2], WIFI_BULLI_GATEWAY[3]);
    const IPAddress subnet(WIFI_BULLI_SUBNET[0], WIFI_BULLI_SUBNET[1], WIFI_BULLI_SUBNET[2], WIFI_BULLI_SUBNET[3]);
    const IPAddress dns(WIFI_BULLI_DNS[0], WIFI_BULLI_DNS[1], WIFI_BULLI_DNS[2], WIFI_BULLI_DNS[3]);

    Serial.println("WLAN V3: IP-Modus = statisch (GeniusBulli 192.168.4.15)");
    if (!WiFi.config(localIp, gateway, subnet, dns)) {
      Serial.println("WLAN V3: FEHLER - statische IP-Konfiguration fehlgeschlagen.");
      return false;
    }
    return true;
  }

  Serial.println("WLAN V3: IP-Modus = DHCP");
  if (!WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE)) {
    Serial.println("WLAN V3: FEHLER - DHCP konnte nicht aktiviert werden.");
    return false;
  }
  return true;
}

// V3: Hilfsfunktion zum Starten eines konkreten Netzwerks aus secrets.h.
// Diese Funktion setzt zuerst die netzabhaengige IP-Konfiguration und startet
// danach den Verbindungsaufbau. Ob die Verbindung klappt, wird anschliessend
// blockierend in wifiInit() oder nicht-blockierend in wifiLoop() ausgewertet.
static bool beginNetwork(int index) {
  if (!validNetworkIndex(index)) {
    return false;
  }

  if (!configureIpForNetwork(index)) {
    return false;
  }

  Serial.print("WLAN V3: verbinde mit Netzwerk ");
  Serial.print(index + 1);
  Serial.print("/");
  Serial.print(WIFI_NETWORK_COUNT);
  Serial.print(": ");
  Serial.println(WIFI_NETWORKS[index].ssid);

  WiFi.begin(WIFI_NETWORKS[index].ssid, WIFI_NETWORKS[index].password);
  return true;
}

// V3: Versucht ein bestimmtes Netzwerk und wartet kurz auf Verbindung.
// Wird bewusst nur in wifiInit() verwendet. Der spaetere Reconnect laeuft
// nicht-blockierend ueber wifiLoop(), damit keine Motorbewegung haengen bleibt.
static bool tryNetworkBlocking(int index) {
  if (!beginNetwork(index)) {
    return false;
  }

  for (int attempt = 0; attempt < WIFI_ATTEMPTS_PER_NETWORK; attempt++) {
    if (WiFi.status() == WL_CONNECTED) {
      currentNetworkIndex = index;
      currentNetworkRssi = WiFi.RSSI();
      Serial.println();
      Serial.print("WLAN V3: verbunden mit Netzwerk ");
      Serial.print(index + 1);
      Serial.print("/");
      Serial.print(WIFI_NETWORK_COUNT);
      Serial.print(": ");
      Serial.print(WIFI_NETWORKS[index].ssid);
      Serial.print(" | RSSI=");
      Serial.print(currentNetworkRssi);
      Serial.println(" dBm");
      return true;
    }

    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WLAN V3: keine Verbindung zu ");
  Serial.println(WIFI_NETWORKS[index].ssid);

  WiFi.disconnect();
  delay(200);
  return false;
}

// V3: Durchsucht beim Boot alle in secrets.h hinterlegten Netzwerke. Primaer
// wird das erste sichtbare Netzwerk gemaess fester Prioritaetsreihenfolge
// versucht. Wenn dieses nicht verbindet, folgen die restlichen Eintraege.
static bool connectToKnownNetworkBlocking() {
  if (WIFI_NETWORK_COUNT <= 0) {
    Serial.println("WLAN V3: keine Netzwerke in secrets.h eingetragen.");
    return false;
  }

  const int bestIndex = scanBestKnownNetworkBlocking();

  if (validNetworkIndex(bestIndex)) {
    if (tryNetworkBlocking(bestIndex)) {
      return true;
    }
  }

  for (int i = 0; i < WIFI_NETWORK_COUNT; i++) {
    if (i == bestIndex) {
      continue;
    }
    if (tryNetworkBlocking(i)) {
      return true;
    }
  }

  return false;
}

// V3: Waehlt das naechste Netzwerk als Fallback, falls beim Reconnect kein
// Scan-Ergebnis verfuegbar ist. Zuerst wird das zuletzt erfolgreiche Netzwerk
// versucht, danach die komplette Liste nach Prioritaet.
static int nextReconnectFallbackIndex() {
  if (WIFI_NETWORK_COUNT <= 0) {
    return -1;
  }

  if (validNetworkIndex(currentNetworkIndex)) {
    const int idx = currentNetworkIndex;
    currentNetworkIndex = -1;  // nur einmal als Sofort-Fallback nutzen
    return idx;
  }

  const int index = reconnectIndex;
  reconnectIndex = (reconnectIndex + 1) % WIFI_NETWORK_COUNT;
  return index;
}

// V3: Startet einen einzelnen Reconnect-Versuch ohne Warte-Schleife.
static void startNonBlockingReconnectAttempt(int index) {
  if (!validNetworkIndex(index)) {
    return;
  }

  // Kein WiFi.disconnect(true): true kann gespeicherte Verbindungsdaten loeschen
  // bzw. den WLAN-Stack unnoetig hart zuruecksetzen. Fuer den laufenden Betrieb
  // reicht ein normales disconnect(), bevor das naechste Netzwerk versucht wird.
  WiFi.disconnect();
  if (!beginNetwork(index)) {
    reconnectConnecting = false;
    reconnectConnectingIndex = -1;
    nextReconnectAttemptMs = millis() + WIFI_RECONNECT_INTERVAL_MS;
    return;
  }

  reconnectConnecting = true;
  reconnectConnectingIndex = index;
  reconnectStartMs = millis();
}

// V3: Erfolgreichen Reconnect abschliessen und Status sauber ausgeben.
static void finishReconnectSuccess() {
  currentNetworkIndex = reconnectConnectingIndex;
  currentNetworkRssi = WiFi.RSSI();
  reconnectConnecting = false;
  reconnectConnectingIndex = -1;
  reconnectScanRunning = false;
  disconnectNoticePrinted = false;
  nextReconnectAttemptMs = millis() + WIFI_RECONNECT_INTERVAL_MS;

  Serial.println();
  Serial.print("WLAN V3: wieder verbunden mit Netzwerk ");
  Serial.print(currentNetworkIndex + 1);
  Serial.print("/");
  Serial.print(WIFI_NETWORK_COUNT);
  Serial.print(": ");
  Serial.print(WIFI_NETWORKS[currentNetworkIndex].ssid);
  Serial.print(" | RSSI=");
  Serial.print(currentNetworkRssi);
  Serial.println(" dBm");
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.localIP());
}

// V3: Fehlgeschlagenen Reconnect-Versuch beenden. Der naechste Versuch wird
// spaeter im loop() gestartet, ohne die restliche Steuerung zu blockieren.
static void finishReconnectFailure() {
  if (validNetworkIndex(reconnectConnectingIndex)) {
    Serial.println();
    Serial.print("WLAN V3: Reconnect fehlgeschlagen: ");
    Serial.println(WIFI_NETWORKS[reconnectConnectingIndex].ssid);
  }

  WiFi.disconnect();
  reconnectConnecting = false;
  reconnectConnectingIndex = -1;
  nextReconnectAttemptMs = millis() + WIFI_RECONNECT_INTERVAL_MS;
}

static void startReconnectScan() {
  WiFi.scanDelete();
  WiFi.scanNetworks(true, true);  // async Scan; Ergebnis spaeter in wifiLoop()
  reconnectScanRunning = true;
  reconnectScanStartMs = millis();
  Serial.println("WLAN V3: scanne bekannte Netzwerke fuer Reconnect...");
}

static bool handleReconnectScan() {
  if (!reconnectScanRunning) {
    return false;
  }

  const int result = WiFi.scanComplete();
  if (result == -1) {  // Scan laeuft noch
    if (millis() - reconnectScanStartMs > WIFI_SCAN_TIMEOUT_MS) {
      Serial.println("WLAN V3: Reconnect-Scan Timeout.");
      WiFi.scanDelete();
      reconnectScanRunning = false;
      nextReconnectAttemptMs = millis() + WIFI_SCAN_RETRY_INTERVAL_MS;
    }
    return true;
  }

  reconnectScanRunning = false;

  if (result <= 0) {
    Serial.println("WLAN V3: Reconnect-Scan ohne bekannte Treffer.");
    WiFi.scanDelete();
    const int fallback = nextReconnectFallbackIndex();
    if (validNetworkIndex(fallback)) {
      startNonBlockingReconnectAttempt(fallback);
    } else {
      nextReconnectAttemptMs = millis() + WIFI_RECONNECT_INTERVAL_MS;
    }
    return true;
  }

  const int bestIndex = bestKnownNetworkFromScanResult(result, true);
  WiFi.scanDelete();

  if (validNetworkIndex(bestIndex)) {
    startNonBlockingReconnectAttempt(bestIndex);
  } else {
    const int fallback = nextReconnectFallbackIndex();
    if (validNetworkIndex(fallback)) {
      startNonBlockingReconnectAttempt(fallback);
    } else {
      nextReconnectAttemptMs = millis() + WIFI_RECONNECT_INTERVAL_MS;
    }
  }

  return true;
}

} // namespace

// Initialisiert die WLAN-Verbindung.
//
// Ablauf V3:
// 1. Station-Mode aktivieren
// 2. Hostname setzen
// 3. sichtbare WLANs scannen
// 4. staerkstes bekanntes Netzwerk verbinden
// 5. bei Erfolg IP-Adresse und RSSI ausgeben
//
// WICHTIG:
// Wenn keine Verbindung moeglich ist, laeuft das System trotzdem lokal weiter.
// Nur Web-UI und OTA stehen dann nicht zur Verfuegung.
void wifiInit() {
  Serial.println();
  Serial.println("Verbinde WLAN...");

  WiFi.mode(WIFI_STA);

  // V3: WLAN-Schlafmodus deaktivieren.
  // Hintergrund: Bei einigen ESP32/Router-Kombinationen werden OTA-/WLAN-
  // Pakete unzuverlaessig sichtbar, wenn der WLAN-Sleep aktiv ist.
  WiFi.setSleep(false);

  // Sichtbarer Geraetename im Netzwerk.
  WiFi.setHostname(WIFI_HOSTNAME);
  Serial.print("WLAN Hostname V3: ");
  Serial.println(WIFI_HOSTNAME);

  if (!connectToKnownNetworkBlocking()) {
    currentNetworkIndex = -1;
    currentNetworkRssi = WIFI_RSSI_UNKNOWN;
    Serial.println("WLAN V3: Verbindung fehlgeschlagen - kein bekanntes Netzwerk erreichbar.");
    return;
  }

  Serial.println("WLAN verbunden.");
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.localIP());
  Serial.print("WLAN RSSI V3: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

// Muss zyklisch aufgerufen werden, um Verbindungsverlust zu erkennen
// und bei Bedarf erneut zu verbinden.
//
// V3: Der Reconnect ist bewusst nicht-blockierend umgesetzt. Fruehere Versionen
// haben im Fehlerfall mit delay() jedes WLAN nacheinander probiert. Das war fuer
// die Web-UI akzeptabel, konnte aber im laufenden Betrieb Tasten-, Motor- und
// Suchlogik stoeren. Jetzt wird pro loop()-Durchlauf nur kurz geprueft bzw. ein
// einzelner Verbindungsversuch betreut.
void wifiLoop() {
  const unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    currentNetworkRssi = WiFi.RSSI();
    if (reconnectConnecting) {
      finishReconnectSuccess();
    }
    reconnectScanRunning = false;
    disconnectNoticePrinted = false;
    return;
  }

  // Verbindung ist nicht vorhanden. Einmalig melden, dann im Hintergrund
  // reconnecten. Die lokale Bedienung ueber TFT/Tasten bleibt dabei aktiv.
  if (!disconnectNoticePrinted) {
    Serial.println("WLAN V3: Verbindung verloren oder nicht verfuegbar.");
    Serial.println("WLAN V3: Lokale Bedienung bleibt aktiv; Web-UI/OTA warten auf Reconnect.");
    disconnectNoticePrinted = true;
    nextReconnectAttemptMs = now;
  }

  // Wenn gerade ein Netzwerk versucht wird, nur Timeout/Erfolg pruefen.
  // Keine Warte-Schleife, keine langen delays.
  if (reconnectConnecting) {
    if (now - reconnectStartMs >= WIFI_CONNECT_TIMEOUT_MS) {
      finishReconnectFailure();
    }
    return;
  }

  if (handleReconnectScan()) {
    return;
  }

  // Reconnect-Versuche zeitlich begrenzen, damit bei fehlendem WLAN nicht
  // permanent neue WiFi.begin()-Aufrufe den Betrieb stoeren.
  if ((long)(now - nextReconnectAttemptMs) < 0) {
    return;
  }

  startReconnectScan();
}

// Liefert den aktuellen Verbindungsstatus als bool.
bool wifiIsConnected() {
  return WiFi.status() == WL_CONNECTED;
}

// Liefert die aktuelle IP-Adresse fuer TFT/Web-Diagnose.
// Kommentarstand V3: Diese Funktion ist reine Anzeigehilfe.
String wifiGetIpString() {
  if (WiFi.status() != WL_CONNECTED) {
    return String("keine Verbindung");
  }
  return WiFi.localIP().toString();
}

// Liefert das aktuell verbundene WLAN fuer die Info-Anzeige.
// Die SSID ist kein Passwort; sie wird nur lokal am Geraet angezeigt.
String wifiGetConnectedSsid() {
  if (WiFi.status() != WL_CONNECTED) {
    return String("nicht verbunden");
  }
  return WiFi.SSID();
}

// Liefert die aktuelle WLAN-Signalstaerke als Text.
// V3: RSSI wird in dBm angezeigt. Je naeher der Wert an 0 liegt, desto besser
// ist die Verbindung, z. B. -45 dBm sehr gut, -70 dBm eher schwach.
String wifiGetRssiString() {
  if (WiFi.status() != WL_CONNECTED) {
    return String("-");
  }
  return String(WiFi.RSSI()) + " dBm";
}
