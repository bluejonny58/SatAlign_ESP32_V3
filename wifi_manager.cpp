/*
  SatAlign V3 - WLAN-Verbindung
  ------------------------------------------------------------
  Baut die WLAN-Verbindung auf und sorgt dafuer, dass Web-UI und OTA im
  lokalen Netz erreichbar werden.

  V3: Die Zugangsdaten liegen nicht mehr als einzelnes WLAN im Code, sondern
  in einer Liste in secrets.h. Dadurch kann die Anlage mehrere bekannte
  Netzwerke nacheinander pruefen, z. B. Heimnetz, Handy-Hotspot und Camping-WLAN.
  Die Reihenfolge der Eintraege in WIFI_NETWORKS[] ist gleichzeitig die
  Prioritaet.
*/

#include <WiFi.h>
#include <Arduino.h>

#include "wifi_config.h"
#include "wifi_manager.h"

namespace {

// V3: Pro Netzwerk nur begrenzt warten, damit die Anlage beim Boot nicht
// minutenlang an einem nicht erreichbaren WLAN haengen bleibt.
// Wird nur in wifiInit() verwendet, also in der Startphase vor dem normalen
// Betrieb.
static const int WIFI_ATTEMPTS_PER_NETWORK = 24;  // 24 * 500 ms = ca. 12 s

// V3: Reconnect im laufenden Betrieb darf Motor-, Such- und Tastenlogik nicht
// blockieren. Deshalb wird nach einem WLAN-Abbruch nicht mehr mit delay()
// durch alle Netzwerke gewartet, sondern stueckweise im loop() gearbeitet.
static const unsigned long WIFI_RECONNECT_INTERVAL_MS = 5000UL;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 12000UL;

// V3: Merkt sich das zuletzt erfolgreiche Netzwerk. Bei einem Reconnect wird
// zuerst dieses Netzwerk versucht, danach die restliche Liste nach Prioritaet.
static int currentNetworkIndex = -1;

// V3: Zustandswerte fuer den nicht-blockierenden Reconnect.
static bool reconnectConnecting = false;
static bool reconnectPreferLastSuccessful = true;
static bool disconnectNoticePrinted = false;
static int reconnectIndex = 0;
static int reconnectConnectingIndex = -1;
static unsigned long reconnectStartMs = 0;
static unsigned long nextReconnectAttemptMs = 0;

// V3: Hilfsfunktion zum Starten eines konkreten Netzwerks aus secrets.h.
// Diese Funktion startet nur den Verbindungsaufbau. Ob die Verbindung klappt,
// wird danach entweder blockierend in wifiInit() oder nicht-blockierend in
// wifiLoop() ausgewertet.
static void beginNetwork(int index) {
  if (index < 0 || index >= WIFI_NETWORK_COUNT) {
    return;
  }

  Serial.print("WLAN V3: pruefe Netzwerk ");
  Serial.print(index + 1);
  Serial.print("/");
  Serial.print(WIFI_NETWORK_COUNT);
  Serial.print(": ");
  Serial.println(WIFI_NETWORKS[index].ssid);

  WiFi.begin(WIFI_NETWORKS[index].ssid, WIFI_NETWORKS[index].password);
}

// V3: Versucht ein bestimmtes Netzwerk und wartet kurz auf Verbindung.
// Wird bewusst nur in wifiInit() verwendet. Der spaetere Reconnect laeuft
// nicht-blockierend ueber wifiLoop(), damit keine Motorbewegung haengen bleibt.
static bool tryNetworkBlocking(int index) {
  beginNetwork(index);

  for (int attempt = 0; attempt < WIFI_ATTEMPTS_PER_NETWORK; attempt++) {
    if (WiFi.status() == WL_CONNECTED) {
      currentNetworkIndex = index;
      Serial.println();
      Serial.print("WLAN V3: verbunden mit Netzwerk ");
      Serial.print(index + 1);
      Serial.print("/");
      Serial.print(WIFI_NETWORK_COUNT);
      Serial.print(": ");
      Serial.println(WIFI_NETWORKS[index].ssid);
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

// V3: Durchsucht beim Boot alle in secrets.h hinterlegten Netzwerke nach
// Prioritaet. Diese Startsuche darf kurz blockieren, weil zu diesem Zeitpunkt
// noch keine Suche/Motorfahrt laeuft.
static bool connectToKnownNetworkBlocking() {
  if (WIFI_NETWORK_COUNT <= 0) {
    Serial.println("WLAN V3: keine Netzwerke in secrets.h eingetragen.");
    return false;
  }

  for (int i = 0; i < WIFI_NETWORK_COUNT; i++) {
    if (tryNetworkBlocking(i)) {
      return true;
    }
  }

  return false;
}

// V3: Waehlt das naechste Netzwerk fuer den nicht-blockierenden Reconnect.
// Zuerst wird das zuletzt erfolgreiche Netzwerk versucht. Danach wird die
// komplette Liste nach Prioritaet durchlaufen.
static int nextReconnectIndex() {
  if (WIFI_NETWORK_COUNT <= 0) {
    return -1;
  }

  if (reconnectPreferLastSuccessful &&
      currentNetworkIndex >= 0 &&
      currentNetworkIndex < WIFI_NETWORK_COUNT) {
    reconnectPreferLastSuccessful = false;
    return currentNetworkIndex;
  }

  const int index = reconnectIndex;
  reconnectIndex = (reconnectIndex + 1) % WIFI_NETWORK_COUNT;
  return index;
}

// V3: Startet einen einzelnen Reconnect-Versuch ohne Warte-Schleife.
static void startNonBlockingReconnectAttempt(int index) {
  if (index < 0 || index >= WIFI_NETWORK_COUNT) {
    return;
  }

  reconnectConnecting = true;
  reconnectConnectingIndex = index;
  reconnectStartMs = millis();

  // Kein WiFi.disconnect(true): true kann gespeicherte Verbindungsdaten loeschen
  // bzw. den WLAN-Stack unnoetig hart zuruecksetzen. Fuer den laufenden Betrieb
  // reicht ein normales disconnect(), bevor das naechste Netzwerk versucht wird.
  WiFi.disconnect();
  beginNetwork(index);
}

// V3: Erfolgreichen Reconnect abschliessen und Status sauber ausgeben.
static void finishReconnectSuccess() {
  currentNetworkIndex = reconnectConnectingIndex;
  reconnectConnecting = false;
  reconnectConnectingIndex = -1;
  reconnectPreferLastSuccessful = true;
  disconnectNoticePrinted = false;
  nextReconnectAttemptMs = millis() + WIFI_RECONNECT_INTERVAL_MS;

  Serial.println();
  Serial.print("WLAN V3: wieder verbunden mit Netzwerk ");
  Serial.print(currentNetworkIndex + 1);
  Serial.print("/");
  Serial.print(WIFI_NETWORK_COUNT);
  Serial.print(": ");
  Serial.println(WIFI_NETWORKS[currentNetworkIndex].ssid);
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.localIP());
}

// V3: Fehlgeschlagenen Reconnect-Versuch beenden. Der naechste Versuch wird
// spaeter im loop() gestartet, ohne die restliche Steuerung zu blockieren.
static void finishReconnectFailure() {
  if (reconnectConnectingIndex >= 0 && reconnectConnectingIndex < WIFI_NETWORK_COUNT) {
    Serial.println();
    Serial.print("WLAN V3: Reconnect fehlgeschlagen: ");
    Serial.println(WIFI_NETWORKS[reconnectConnectingIndex].ssid);
  }

  WiFi.disconnect();
  reconnectConnecting = false;
  reconnectConnectingIndex = -1;
  nextReconnectAttemptMs = millis() + WIFI_RECONNECT_INTERVAL_MS;
}

} // namespace

// Initialisiert die WLAN-Verbindung.
//
// Ablauf V3:
// 1. Station-Mode aktivieren
// 2. Hostname setzen
// 3. alle in secrets.h hinterlegten WLANs nacheinander pruefen
// 4. bei Erfolg IP-Adresse ausgeben
//
// WICHTIG:
// Wenn keine Verbindung moeglich ist, laeuft das System trotzdem lokal weiter.
// Nur Web-UI und OTA stehen dann nicht zur Verfuegung.
void wifiInit() {
  Serial.println();
  Serial.println("Verbinde WLAN...");

  WiFi.mode(WIFI_STA);

  // V3: WLAN-Schlafmodus deaktivieren.
  // Hintergrund: Bei einigen ESP32/Router-Kombinationen werden mDNS/OTA-
  // Pakete unzuverlaessig sichtbar, wenn der WLAN-Sleep aktiv ist.
  WiFi.setSleep(false);

  // Sichtbarer Geraetename im Netzwerk.
  WiFi.setHostname(WIFI_HOSTNAME);
  Serial.print("WLAN Hostname V3: ");
  Serial.println(WIFI_HOSTNAME);

  if (!connectToKnownNetworkBlocking()) {
    currentNetworkIndex = -1;
    Serial.println("WLAN V3: Verbindung fehlgeschlagen - kein bekanntes Netzwerk erreichbar.");
    return;
  }

  Serial.println("WLAN verbunden.");
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.localIP());
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
    if (reconnectConnecting) {
      finishReconnectSuccess();
    }
    disconnectNoticePrinted = false;
    return;
  }

  // Verbindung ist nicht vorhanden. Einmalig melden, dann im Hintergrund
  // reconnecten. Die lokale Bedienung ueber TFT/Tasten bleibt dabei aktiv.
  if (!disconnectNoticePrinted) {
    Serial.println("WLAN V3: Verbindung verloren oder nicht verfuegbar.");
    Serial.println("WLAN V3: Lokale Bedienung bleibt aktiv; Web-UI/OTA warten auf Reconnect.");
    disconnectNoticePrinted = true;
    reconnectPreferLastSuccessful = true;
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

  // Reconnect-Versuche zeitlich begrenzen, damit bei fehlendem WLAN nicht
  // permanent neue WiFi.begin()-Aufrufe den Betrieb stoeren.
  if ((long)(now - nextReconnectAttemptMs) < 0) {
    return;
  }

  const int index = nextReconnectIndex();
  if (index < 0) {
    nextReconnectAttemptMs = now + WIFI_RECONNECT_INTERVAL_MS;
    return;
  }

  startNonBlockingReconnectAttempt(index);
}

// Liefert den aktuellen Verbindungsstatus als bool.
bool wifiIsConnected() {
  return WiFi.status() == WL_CONNECTED;
}
