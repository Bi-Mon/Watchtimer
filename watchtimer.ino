/*
 * Wach-Timer für ESP32
 * -----------------------------------------------------------
 * Löst alle INTERVAL_MINUTES einen Alarm aus (aktiver Piezo-
 * Buzzer, im Intervall-Piepen). Der Alarm kann nur über einen
 * großen Taster (normally-open, gegen GND) gestoppt werden.
 * Danach läuft der Timer vollautomatisch wieder von vorne los.
 *
 * Verdrahtung:
 *   Buzzer  +  -> GPIO 25
 *   Buzzer  -  -> GND
 *   Taster  A  -> GPIO 27
 *   Taster  B  -> GND
 *   (interner Pullup wird per Software aktiviert, kein
 *    externer Widerstand nötig)
 *
 *   Onboard-LED (GPIO 2) blinkt während des Countdowns im
 *   Sekundentakt (Lebenszeichen) und leuchtet während des
 *   Alarms dauerhaft.
 *
 *   SSD1309-Display (I2C, 128x64):
 *   SDA -> GPIO 21
 *   SCL -> GPIO 22
 *   VCC -> 3V3
 *   GND -> GND
 *   Zeigt die verbleibende Zeit rückwärts laufend an und
 *   blinkt "ALARM" während der Alarm aktiv ist.
 *
 *   Rotary Encoder (z.B. KY-040):
 *   CLK -> GPIO 32
 *   DT  -> GPIO 33
 *   SW  -> nicht verwendet (Stopp erfolgt ausschliesslich
 *          über den grossen Taster)
 *   +   -> 3V3
 *   GND -> GND
 *   Drehen verändert das Alarm-Intervall (1-180 Min., ein
 *   Rasten = 1 Minute) und startet den Countdown sofort mit
 *   dem neuen Wert neu. Funktioniert nur während des
 *   Countdowns, nicht während ein Alarm laeuft.
 *
 *   WLAN / Weboberfläche (per WiFiManager-Captive-Portal):
 *   Beim ersten Start (oder wenn kein bekanntes WLAN gefunden
 *   wird) öffnet der ESP32 selbst ein WLAN namens
 *   "WachTimer-Setup". Verbinde dich per Handy/Laptop damit,
 *   es öffnet sich automatisch eine Konfigurationsseite
 *   (sonst http://192.168.4.1) - dort WLAN, MQTT-Broker und
 *   optional SignalK eintragen. Alles wird im NVS-Flash
 *   gespeichert und übersteht Reboots. Kein Neuflashen mehr
 *   nötig, wenn sich das Boot-WLAN ändert. Nach 3 Minuten ohne
 *   Konfiguration läuft der Timer notfalls offline weiter.
 *   Danach ist der Timer per Browser erreichbar unter
 *   http://wachtimer.local oder der IP aus der seriellen
 *   Konsole. Zeigt Status/Countdown live an, erlaubt das
 *   Verändern des Intervalls, zeigt eine Verlaufsgrafik und
 *   enthält einen STOPP-Button mit der exakt gleichen Funktion
 *   wie der physische Taster.
 *   Bricht die WLAN-Verbindung während des Betriebs ab, wird
 *   automatisch alle 10s ein Reconnect versucht (nicht
 *   blockierend).
 *
 *   OTA-Updates:
 *   Nach dem Verbinden erscheint der Timer in der Arduino IDE
 *   unter Werkzeuge -> Port als Netzwerkport ("wachtimer" via
 *   mDNS). Firmware-Updates gehen dann per WLAN statt per
 *   Kabel. Durch das im Setup-Portal vergebene OTA-Passwort
 *   geschützt. Während eines Updates pausiert der Timer
 *   komplett (Buzzer aus), der Watchdog wird während der
 *   Übertragung aktiv weitergefüttert, damit er nicht
 *   mittendrin resettet.
 *
 *   MQTT / Home Assistant:
 *   Broker-Adresse wird im Setup-Portal eingetragen. Meldet
 *   Status, Restzeit und Intervall auf "wachtimer/state",
 *   "wachtimer/remaining_seconds", "wachtimer/interval_minutes".
 *   Nimmt Befehle entgegen auf "wachtimer/cmd" (Payload "stop")
 *   und "wachtimer/set_interval" (Payload = neue Minutenzahl).
 *   Meldet sich zusätzlich per Home-Assistant-MQTT-Discovery an.
 *
 *   SignalK:
 *   Host/Port/Token optional im Setup-Portal eintragen (Feld
 *   leer lassen = SignalK-Anbindung deaktiviert). Verbindet
 *   sich per WebSocket-Delta-Stream (der von SignalK offiziell
 *   empfohlene Weg, um eigene Werte einzuspeisen - eine PUT-
 *   Anfrage funktioniert dafür NICHT, die ist für Aktoren mit
 *   registriertem PUT-Handler gedacht). Meldet den Alarmzustand
 *   als Notification auf "notifications.wachtimer" (state
 *   "alarm"/"normal", passend zur SignalK-Alarm-Konvention) und
 *   Restzeit/Intervall als eigene Pfade unter
 *   "electrical.wachtimer.*". Falls dein Server Security aktiv
 *   hat, brauchst du ein Zugriffs-Token (Device-Access-Request
 *   oder manuell erzeugt) im Feld "SignalK Token".
 *
 *   Intervall-Speicherung (NVS):
 *   Das zuletzt gesetzte Intervall wird über die Preferences-
 *   Library im NVS-Flash gespeichert und übersteht Reboots und
 *   Stromausfall.
 *
 *   Verlaufsprotokoll:
 *   Jede Quittierung (Taster/Web/MQTT) wird mit Uhrzeit (per NTP)
 *   und Abstand zur vorherigen Quittierung in einem Ringpuffer
 *   im RAM gespeichert (die letzten 50 Einträge, geht bei Reboot
 *   verloren - für eine einzelne Nacht/Wache reicht das). Auf der
 *   Weboberfläche als kleines Balkendiagramm sichtbar; grüner
 *   Balken = Abstand ~ Intervall, oranger Balken = deutlich
 *   längerer Abstand (z.B. Reboot oder verpasste Quittierung).
 *
 *   Watchdog:
 *   Ein Hardware-Watchdog (esp_task_wdt) resettet den ESP32
 *   automatisch, falls die Hauptschleife mal 20s lang nicht mehr
 *   durchläuft (z.B. durch einen haengenden WLAN/I2C-Zugriff) -
 *   sicherheitsrelevant, da der Alarm sonst stumm ausfallen könnte.
 *
 * Benötigte Libraries (Arduino IDE Bibliotheksverwalter):
 *   "U8g2" von olikraus
 *   "PubSubClient" von Nick O'Leary
 *   "WiFiManager" von tzapu
 *   "WebSockets" von Markus Sattler (Links2004)
 *   WiFi, WebServer, ESPmDNS, Preferences, ArduinoOTA
 *   (im ESP32-Boardpaket enthalten)
 * -----------------------------------------------------------
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <WebSocketsClient.h>
#include <time.h>
#include "esp_task_wdt.h"

// ----- WLAN / Setup-Portal -------------------------------------------------
const char* MDNS_HOSTNAME = "wachtimer";   // -> http://wachtimer.local

WebServer server(80);

uint32_t lastWifiCheck   = 0;
const uint32_t WIFI_CHECK_INTERVAL_MS = 10000;   // Reconnect-Versuch alle 10s

// Per WiFiManager-Setup-Portal konfigurierbare Werte (im NVS gespeichert)
char mqttServerBuf[40] = "";
char mqttPortBuf[6]    = "1883";
char mqttUserBuf[32]   = "";
char mqttPassBuf[32]   = "";
char skHostBuf[40]     = "";
char skPortBuf[6]      = "3000";
char skTokenBuf[180]   = "";
char otaPassBuf[32]    = "wachtimer";

bool shouldSaveConfig = false;
void saveConfigCallback() { shouldSaveConfig = true; }

WiFiManager wm;   // global, damit auch der Web-"WLAN zuruecksetzen"-Button darauf zugreifen kann

// ----- MQTT -----------------------------------------------------------
const char* MQTT_CLIENT_ID = "wachtimer";
const char* TOPIC_STATE       = "wachtimer/state";
const char* TOPIC_REMAINING   = "wachtimer/remaining_seconds";
const char* TOPIC_INTERVAL    = "wachtimer/interval_minutes";
const char* TOPIC_LAST_ACK    = "wachtimer/last_ack";
const char* TOPIC_CMD         = "wachtimer/cmd";
const char* TOPIC_SET_INTERVAL = "wachtimer/set_interval";

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
uint32_t     lastMqttAttempt = 0;
const uint32_t MQTT_RETRY_MS = 5000;
uint32_t     lastPublish = 0;
const uint32_t PUBLISH_INTERVAL_MS = 3000;
bool         mqttDiscoverySent = false;

// ----- SignalK (WebSocket-Delta) ----------------------------------------
WebSocketsClient skWs;
bool skEnabled = false;

// ----- OTA ---------------------------------------------------------------
bool otaInProgress = false;

// ----- NVS (Konfigurationsspeicherung) ---------------------------------
Preferences prefs;

// ----- NTP / Uhrzeit ---------------------------------------------------
const char* NTP_SERVER = "de.pool.ntp.org";
const char* TZ_INFO    = "CET-1CEST,M3.5.0,M10.5.0/3";   // Deutschland, inkl. Sommerzeit

// ----- Verlaufsprotokoll (Ringpuffer im RAM) --------------------------
struct AckEntry {
  char     timestamp[9];   // "HH:MM:SS"
  uint32_t gapSeconds;     // Abstand zur vorherigen Quittierung
};
const uint8_t HISTORY_SIZE = 50;
AckEntry history[HISTORY_SIZE];
uint8_t  historyHead  = 0;   // naechster Schreib-Index
uint8_t  historyCount = 0;
uint32_t lastAckMillis = 0;

// ----- Watchdog ---------------------------------------------------------
const uint32_t WDT_TIMEOUT_S = 20;

// Vorwaertsdeklarationen (Definition weiter unten im Sketch)
void publishState();
void publishSignalK();
void logAcknowledgement();

// ----- Display -------------------------------------------------------------
// Falls das Display beim Start schwarz bleibt oder Grafikfehler zeigt,
// stattdessen U8G2_SSD1309_128X64_NONAME2_F_HW_I2C probieren.
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ----- Rotary Encoder --------------------------------------------------
const uint8_t ENC_CLK = 32;
const uint8_t ENC_DT  = 33;

volatile int32_t encoderDelta  = 0;   // von der ISR akkumulierte Schritte
volatile uint8_t lastEncoded   = 0;
volatile uint32_t lastEncoderStepUs = 0;
const uint32_t ENC_DEBOUNCE_US = 1200;   // Mindestabstand zwischen zwei gültigen Rasten

void IRAM_ATTR encoderISR() {
  uint8_t msb = digitalRead(ENC_CLK);
  uint8_t lsb = digitalRead(ENC_DT);
  uint8_t encoded = (msb << 1) | lsb;
  uint8_t sum = (lastEncoded << 2) | encoded;

  int8_t step = 0;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    step = 1;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    step = -1;
  }

  if (step != 0) {
    uint32_t now = micros();
    // nur zählen, wenn seit dem letzten gültigen Schritt genug Zeit
    // vergangen ist -> filtert mechanisches Prellen des Encoders
    if (now - lastEncoderStepUs > ENC_DEBOUNCE_US) {
      encoderDelta += step;
      lastEncoderStepUs = now;
    }
  }

  // lastEncoded IMMER aktualisieren, auch wenn der Schritt wegen
  // Entprellung verworfen wurde - sonst verliert die Zustandstabelle
  // die Synchronisation zum tatsächlichen Encoder-Signal
  lastEncoded = encoded;
}

// ----- Konfiguration -----------------------------------------------------
uint32_t INTERVAL_MINUTES = 30;                            // Alarm-Intervall (verstellbar)
const uint32_t INTERVAL_MIN_LIMIT = 1;
const uint32_t INTERVAL_MAX_LIMIT = 180;
uint32_t INTERVAL_MS      = INTERVAL_MINUTES * 60UL * 1000UL;

const uint8_t  BUZZER_PIN = 25;
const uint8_t  BUTTON_PIN = 27;
const uint8_t  LED_PIN    = 2;

const uint32_t BEEP_ON_MS  = 300;   // Piepdauer im Alarm
const uint32_t BEEP_OFF_MS = 300;   // Pause zwischen den Piepern
const uint32_t DEBOUNCE_MS = 40;    // Entprellzeit Taster

// ----- Zustände ------------------------------------------------------------
enum State { COUNTING, ALARMING };
State state = COUNTING;

uint32_t countdownStart   = 0;   // millis() bei Timer-Start
uint32_t beepToggleAt     = 0;   // nächster Wechsel des Buzzer-Tons
bool     buzzerOn         = false;

bool     ledOn            = false;
uint32_t ledToggleAt      = 0;

// Taster-Entprellung
int      lastRawState     = HIGH;
int      stableState      = HIGH;
uint32_t lastEdgeAt       = 0;

// Display
uint32_t lastDisplayUpdate = 0;
bool     alarmBlinkOn      = true;
uint32_t alarmBlinkAt      = 0;
uint32_t showIntervalUntil = 0;   // != 0 -> Intervall-Anzeige statt Countdown

void startCountdown() {
  state = COUNTING;
  countdownStart = millis();
  digitalWrite(BUZZER_PIN, LOW);
  buzzerOn = false;
  Serial.println("[Timer] Countdown gestartet, naechster Alarm in " +
                  String(INTERVAL_MINUTES) + " Minuten.");
  lastDisplayUpdate = 0;   // erzwingt sofortiges Neuzeichnen
}

void triggerAlarm() {
  state = ALARMING;
  beepToggleAt = millis();
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerOn = true;
  digitalWrite(LED_PIN, HIGH);
  alarmBlinkOn = true;
  alarmBlinkAt = 0;        // erzwingt sofortiges Neuzeichnen
  Serial.println("[Timer] ALARM ausgeloest -> Taster druecken zum Stoppen.");
  publishState();
  publishSignalK();
}

// Setzt ein neues Intervall (geklemmt auf 1-180 Min.), startet den
// Countdown sofort neu und blendet kurz die neue Einstellung ein.
// Wird sowohl vom Encoder als auch von der Weboberfläche genutzt.
void applyIntervalChange(int32_t newMinutes) {
  if (newMinutes < (int32_t)INTERVAL_MIN_LIMIT) newMinutes = INTERVAL_MIN_LIMIT;
  if (newMinutes > (int32_t)INTERVAL_MAX_LIMIT) newMinutes = INTERVAL_MAX_LIMIT;
  INTERVAL_MINUTES = (uint32_t)newMinutes;
  INTERVAL_MS = INTERVAL_MINUTES * 60UL * 1000UL;

  prefs.putUInt("interval", INTERVAL_MINUTES);   // ueberlebt Reboot/Stromausfall

  Serial.println("[Timer] Intervall geaendert auf " +
                  String(INTERVAL_MINUTES) + " Minuten.");

  startCountdown();
  showIntervalUntil = millis() + 1200;
  publishState();
  publishSignalK();
}

// Beendet den Alarm (Buzzer/LED aus), protokolliert die Quittierung
// und startet den Countdown neu. Wird vom physischen Taster, dem
// Web-STOPP-Button und dem MQTT-Kommando "stop" gemeinsam genutzt.
void stopAlarmAndRestart() {
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  logAcknowledgement();
  startCountdown();
  publishState();
  publishSignalK();
}

// true, wenn der Taster in diesem Aufruf frisch gedrueckt wurde
bool buttonPressedEdge() {
  int raw = digitalRead(BUTTON_PIN);

  if (raw != lastRawState) {
    lastEdgeAt = millis();
    lastRawState = raw;
  }

  if ((millis() - lastEdgeAt) > DEBOUNCE_MS && raw != stableState) {
    stableState = raw;
    if (stableState == LOW) {   // Taster gegen GND = gedrueckt
      return true;
    }
  }
  return false;
}

void handleCounting() {
  // Lebenszeichen-LED im Sekundentakt blinken lassen
  if (millis() - ledToggleAt >= 1000) {
    ledToggleAt = millis();
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  }

  // Encoder-Schritte seit dem letzten Aufruf atomar auslesen
  noInterrupts();
  int32_t delta = encoderDelta;
  encoderDelta = 0;
  interrupts();

  if (delta != 0) {
    applyIntervalChange((int32_t)INTERVAL_MINUTES + delta);
  }

  if (millis() - countdownStart >= INTERVAL_MS) {
    triggerAlarm();
  }
}

void handleAlarming() {
  // Buzzer im Intervall piepen lassen
  uint32_t interval = buzzerOn ? BEEP_ON_MS : BEEP_OFF_MS;
  if (millis() - beepToggleAt >= interval) {
    beepToggleAt = millis();
    buzzerOn = !buzzerOn;
    digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
  }

  if (buttonPressedEdge()) {
    Serial.println("[Timer] Taster gedrueckt -> Alarm gestoppt, Timer neu gestartet.");
    stopAlarmAndRestart();
  }
}

// Verbleibende Sekunden bis zum nächsten Alarm
uint32_t remainingSeconds() {
  uint32_t elapsed = millis() - countdownStart;
  if (elapsed >= INTERVAL_MS) return 0;
  return (INTERVAL_MS - elapsed) / 1000;
}

// Protokolliert eine Quittierung mit NTP-Uhrzeit und Abstand zur
// vorherigen Quittierung im Ringpuffer.
void logAcknowledgement() {
  AckEntry &e = history[historyHead];

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 100)) {
    strftime(e.timestamp, sizeof(e.timestamp), "%H:%M:%S", &timeinfo);
  } else {
    strcpy(e.timestamp, "--:--:--");   // keine Uhrzeit verfuegbar (kein WLAN/NTP)
  }

  uint32_t now = millis();
  e.gapSeconds = (lastAckMillis == 0) ? 0 : (now - lastAckMillis) / 1000;
  lastAckMillis = now;

  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

// Liefert die Historie als JSON-Array, chronologisch (aeltester zuerst)
String historyToJson() {
  String json = "[";
  uint8_t start = (historyHead + HISTORY_SIZE - historyCount) % HISTORY_SIZE;
  for (uint8_t i = 0; i < historyCount; i++) {
    uint8_t idx = (start + i) % HISTORY_SIZE;
    if (i > 0) json += ",";
    json += "{\"time\":\"" + String(history[idx].timestamp) + "\",";
    json += "\"gapSeconds\":" + String(history[idx].gapSeconds) + "}";
  }
  json += "]";
  return json;
}

void drawCountdown() {
  uint32_t rem = remainingSeconds();
  uint32_t mm = rem / 60;
  uint32_t ss = rem % 60;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)mm, (unsigned long)ss);

  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tf);
  const char* label = "NAECHSTER ALARM";
  int labelW = u8g2.getStrWidth(label);
  u8g2.drawStr((128 - labelW) / 2, 14, label);

  u8g2.setFont(u8g2_font_logisoso32_tn);   // grosse, schmale Ziffern
  int numW = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - numW) / 2, 58, buf);

  u8g2.sendBuffer();
}

void drawAlarmScreen() {
  u8g2.clearBuffer();
  if (alarmBlinkOn) {
    u8g2.setFont(u8g2_font_logisoso32_tf);
    const char* txt = "ALARM";
    int w = u8g2.getStrWidth(txt);
    u8g2.drawStr((128 - w) / 2, 42, txt);

    u8g2.setFont(u8g2_font_6x10_tf);
    const char* sub = "Taster druecken";
    int subW = u8g2.getStrWidth(sub);
    u8g2.drawStr((128 - subW) / 2, 60, sub);
  }
  u8g2.sendBuffer();
}

void drawIntervalScreen() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tf);
  const char* label = "INTERVALL";
  int labelW = u8g2.getStrWidth(label);
  u8g2.drawStr((128 - labelW) / 2, 16, label);

  char buf[8];
  snprintf(buf, sizeof(buf), "%lu MIN", (unsigned long)INTERVAL_MINUTES);
  u8g2.setFont(u8g2_font_logisoso32_tn);
  int numW = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - numW) / 2, 58, buf);

  u8g2.sendBuffer();
}

void updateDisplay() {
  if (state == COUNTING) {
    if (showIntervalUntil != 0 && millis() < showIntervalUntil) {
      drawIntervalScreen();
      return;
    }
    showIntervalUntil = 0;

    // einmal pro Sekunde reicht, spart I2C-Traffic
    if (millis() - lastDisplayUpdate >= 1000) {
      lastDisplayUpdate = millis();
      drawCountdown();
    }
  } else { // ALARMING
    if (millis() - alarmBlinkAt >= 500) {
      alarmBlinkAt = millis();
      alarmBlinkOn = !alarmBlinkOn;
      drawAlarmScreen();
    }
  }
}

// ----- WiFi-Reconnect (nicht blockierend) --------------------------------
// WiFiManager speichert die WLAN-Zugangsdaten im eigenen NVS-Bereich des
// ESP32-WiFi-Treibers, daher genuegt WiFi.reconnect() - es muessen keine
// SSID/Passwort mehr im Sketch vorgehalten werden.
void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiCheck < WIFI_CHECK_INTERVAL_MS) return;
  lastWifiCheck = millis();
  Serial.println("[WiFi] Verbindung verloren, versuche Reconnect...");
  WiFi.reconnect();
}

// ----- MQTT -------------------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  String t = String(topic);

  if (t == TOPIC_CMD) {
    if (msg == "stop" && state == ALARMING) {
      Serial.println("[MQTT] Stopp-Kommando empfangen.");
      stopAlarmAndRestart();
    }
  } else if (t == TOPIC_SET_INTERVAL) {
    Serial.println("[MQTT] Neues Intervall per MQTT: " + msg);
    applyIntervalChange(msg.toInt());
  }
}

// Meldet die Entitäten einmalig per Home-Assistant-MQTT-Discovery an.
void publishDiscovery() {
  String dev = "\"device\":{\"identifiers\":[\"wachtimer\"],\"name\":\"Wach-Timer\",\"model\":\"ESP32\",\"manufacturer\":\"DIY\"}";

  mqttClient.publish("homeassistant/sensor/wachtimer_state/config",
    ("{\"name\":\"Wachtimer Status\",\"state_topic\":\"" + String(TOPIC_STATE) +
     "\",\"unique_id\":\"wachtimer_state\"," + dev + "}").c_str(), true);

  mqttClient.publish("homeassistant/sensor/wachtimer_remaining/config",
    ("{\"name\":\"Wachtimer Restzeit\",\"state_topic\":\"" + String(TOPIC_REMAINING) +
     "\",\"unit_of_measurement\":\"s\",\"unique_id\":\"wachtimer_remaining\"," + dev + "}").c_str(), true);

  mqttClient.publish("homeassistant/number/wachtimer_interval/config",
    ("{\"name\":\"Wachtimer Intervall\",\"state_topic\":\"" + String(TOPIC_INTERVAL) +
     "\",\"command_topic\":\"" + String(TOPIC_SET_INTERVAL) +
     "\",\"min\":1,\"max\":180,\"unit_of_measurement\":\"min\",\"unique_id\":\"wachtimer_interval\"," + dev + "}").c_str(), true);

  mqttClient.publish("homeassistant/button/wachtimer_stop/config",
    ("{\"name\":\"Wachtimer Stopp\",\"command_topic\":\"" + String(TOPIC_CMD) +
     "\",\"payload_press\":\"stop\",\"unique_id\":\"wachtimer_stop\"," + dev + "}").c_str(), true);
}

void connectMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (strlen(mqttServerBuf) == 0) return;   // kein Broker konfiguriert
  if (mqttClient.connected()) return;
  if (millis() - lastMqttAttempt < MQTT_RETRY_MS) return;
  lastMqttAttempt = millis();

  Serial.print("[MQTT] Verbinde mit Broker...");
  bool ok;
  if (strlen(mqttUserBuf) > 0) {
    ok = mqttClient.connect(MQTT_CLIENT_ID, mqttUserBuf, mqttPassBuf);
  } else {
    ok = mqttClient.connect(MQTT_CLIENT_ID);
  }

  if (ok) {
    Serial.println(" verbunden.");
    mqttClient.subscribe(TOPIC_CMD);
    mqttClient.subscribe(TOPIC_SET_INTERVAL);
    if (!mqttDiscoverySent) {
      publishDiscovery();
      mqttDiscoverySent = true;
    }
    publishState();
  } else {
    Serial.println(" fehlgeschlagen, rc=" + String(mqttClient.state()));
  }
}

// Sendet den aktuellen Status an MQTT (bei Zustandswechsel sofort,
// ansonsten periodisch aus der loop()).
void publishState() {
  if (!mqttClient.connected()) return;
  mqttClient.publish(TOPIC_STATE, state == ALARMING ? "ALARM" : "COUNTING", true);
  mqttClient.publish(TOPIC_REMAINING, String(remainingSeconds()).c_str(), true);
  mqttClient.publish(TOPIC_INTERVAL, String(INTERVAL_MINUTES).c_str(), true);
  if (historyCount > 0) {
    uint8_t lastIdx = (historyHead + HISTORY_SIZE - 1) % HISTORY_SIZE;
    mqttClient.publish(TOPIC_LAST_ACK, history[lastIdx].timestamp, true);
  }
}

// ----- SignalK (WebSocket-Delta) ----------------------------------------
// SignalK-Werte werden per WebSocket-Delta gesendet, nicht per HTTP PUT -
// PUT ist fuer Aktoren mit registriertem PUT-Handler gedacht (z.B.
// Autopilot-Kommandos), nicht fuer beliebige eigene Sensordaten.
void skWebSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("[SignalK] WebSocket verbunden.");
      publishSignalK();
      break;
    case WStype_DISCONNECTED:
      Serial.println("[SignalK] WebSocket getrennt.");
      break;
    default:
      break;
  }
}

void setupSignalK() {
  if (strlen(skHostBuf) == 0) {
    skEnabled = false;
    return;
  }
  skEnabled = true;
  uint16_t skPort = atoi(skPortBuf);
  skWs.begin(skHostBuf, skPort, "/signalk/v1/stream?subscribe=none");
  if (strlen(skTokenBuf) > 0) {
    String headers = "Authorization: Bearer " + String(skTokenBuf);
    skWs.setExtraHeaders(headers.c_str());
  }
  skWs.onEvent(skWebSocketEvent);
  skWs.setReconnectInterval(5000);
}

// Meldet den Alarmzustand als SignalK-Notification (passend zur SignalK-
// Alarm-Konvention) sowie Restzeit/Intervall als eigene Pfade.
void publishSignalK() {
  if (!skEnabled || !skWs.isConnected()) return;

  const char* notifState = (state == ALARMING) ? "alarm" : "normal";
  String msgText = (state == ALARMING) ? "Wachtimer Alarm - Taster druecken" : "Wachtimer laeuft";

  String msg = "{\"updates\":[{\"source\":{\"label\":\"wachtimer\"},\"values\":[";
  msg += "{\"path\":\"notifications.wachtimer\",\"value\":{";
  msg += "\"state\":\"" + String(notifState) + "\",";
  msg += "\"message\":\"" + msgText + "\",";
  msg += "\"method\":[\"visual\"]}},";
  msg += "{\"path\":\"electrical.wachtimer.remainingSeconds\",\"value\":" + String(remainingSeconds()) + "},";
  msg += "{\"path\":\"electrical.wachtimer.intervalMinutes\",\"value\":" + String(INTERVAL_MINUTES) + "}";
  msg += "]}]}";

  skWs.sendTXT(msg);
}

// ----- OTA ---------------------------------------------------------------
void setupOTA() {
  ArduinoOTA.setHostname(MDNS_HOSTNAME);
  if (strlen(otaPassBuf) > 0) {
    ArduinoOTA.setPassword(otaPassBuf);
  }

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(6, 32, "OTA Update laeuft...");
    u8g2.drawStr(6, 46, "Bitte warten");
    u8g2.sendBuffer();
    Serial.println("[OTA] Update gestartet.");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    esp_task_wdt_reset();   // Watchdog waehrend der Uebertragung aktiv fuettern
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] Update abgeschlossen, Neustart folgt.");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.println("[OTA] Fehler Nr. " + String(error));
    otaInProgress = false;
  });

  ArduinoOTA.begin();
}

// ----- Webserver -------------------------------------------------------

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Wach-Timer</title>
<style>
  :root{
    --bg:#0a0e13; --panel:#10161d; --line:#1e2830;
    --amber:#e8a33d; --red:#d43b2f; --red-bright:#ff4433;
    --text:#d8d4c8; --text-dim:#6b7580;
    --mono:'Courier New', ui-monospace, monospace;
  }
  *{box-sizing:border-box;}
  body{
    margin:0; min-height:100vh; background:var(--bg); color:var(--text);
    font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;
    display:flex; flex-direction:column; align-items:center;
    padding:32px 16px;
  }
  h1{
    font-family:var(--mono); font-size:14px; letter-spacing:.25em;
    color:var(--text-dim); text-transform:uppercase; margin:0 0 24px;
  }
  #state{
    font-family:var(--mono); font-size:13px; letter-spacing:.15em;
    color:var(--amber); margin-bottom:8px;
  }
  #clock{
    font-family:var(--mono); font-variant-numeric:tabular-nums;
    font-size:19vw; max-font-size:96px; color:var(--amber); line-height:1;
    text-shadow:0 0 30px rgba(232,163,61,.25);
  }
  @media(min-width:600px){ #clock{ font-size:96px; } }
  .panel{
    background:var(--panel); border:1px solid var(--line); border-radius:8px;
    padding:20px; margin-top:28px; width:100%; max-width:340px;
  }
  .row{ display:flex; align-items:center; gap:10px; margin-bottom:14px; }
  .row label{ font-family:var(--mono); font-size:12px; color:var(--text-dim); flex:1; }
  input[type=number]{
    width:70px; background:var(--bg); border:1px solid var(--line);
    color:var(--amber); font-family:var(--mono); font-size:16px;
    text-align:center; padding:8px 4px; border-radius:4px;
  }
  button{
    font-family:var(--mono); font-size:13px; letter-spacing:.08em;
    border:1px solid var(--line); background:transparent; color:var(--text);
    padding:10px 16px; border-radius:4px; cursor:pointer; width:100%;
  }
  button:active{ border-color:var(--amber); color:var(--amber); }
  #stopBtn{
    margin-top:24px; width:100%; max-width:340px; padding:26px;
    border-radius:50%; aspect-ratio:1; border:none;
    background:radial-gradient(circle at 35% 30%, #ff5a4a, var(--red));
    color:#1a0705; font-weight:700; font-size:22px; letter-spacing:.05em;
    box-shadow:0 12px 40px rgba(0,0,0,.5);
  }
  #stopBtn:disabled{ opacity:.25; }
  #hint{ font-family:var(--mono); font-size:11px; color:var(--text-dim); margin-top:20px; text-align:center; }

  .panel h2{
    font-family:var(--mono); font-size:12px; letter-spacing:.1em;
    color:var(--text-dim); text-transform:uppercase; margin:0 0 14px;
  }
  #historyCanvas{ width:100%; height:110px; display:block; }
  #historyList{ font-family:var(--mono); font-size:11px; color:var(--text-dim); margin-top:10px; }
  #historyList div{ display:flex; justify-content:space-between; padding:2px 0; }
  #historyList .gap-ok{ color:var(--amber); }
  #historyList .gap-warn{ color:var(--red-bright); }

  .panel input[type=text], .panel input[type=password]{
    width:100%; background:var(--bg); border:1px solid var(--line);
    color:var(--amber); font-family:var(--mono); font-size:13px;
    padding:8px 8px; border-radius:4px;
  }
  .field{ margin-bottom:12px; }
  .field label{ display:block; font-family:var(--mono); font-size:11px; color:var(--text-dim); margin-bottom:5px; }
  .subrow{ display:flex; gap:8px; }
  .subrow .field{ flex:1; }
  .status-line{ font-family:var(--mono); font-size:11px; margin-bottom:14px; display:flex; justify-content:space-between; }
  .ok{ color:var(--amber); }
  .warn{ color:var(--red-bright); }
  .danger-btn{ border-color:var(--red); color:var(--red-bright); margin-top:6px; }
  .danger-btn:active{ border-color:var(--red-bright); }
</style>
</head>
<body>
  <h1>Wach-Timer</h1>
  <div id="state">-</div>
  <div id="clock">--:--</div>

  <div class="panel">
    <div class="row">
      <label>Intervall (1-180 Min.)</label>
      <input type="number" id="minutes" min="1" max="180" value="30">
    </div>
    <button id="applyBtn">Übernehmen</button>
  </div>

  <button id="stopBtn">STOPP</button>
  <div id="hint">Der STOPP-Button wirkt nur während ein Alarm läuft.</div>

  <div class="panel">
    <h2>Verlauf der Quittierungen</h2>
    <canvas id="historyCanvas"></canvas>
    <div id="historyList"></div>
  </div>

  <div class="panel">
    <h2>WLAN</h2>
    <div class="status-line"><span id="wifiInfo">-</span></div>
    <button class="danger-btn" id="wifiResetBtn">WLAN zurücksetzen</button>
  </div>

  <div class="panel">
    <h2>MQTT / Home Assistant</h2>
    <div class="status-line"><span>Status</span><span id="mqttStatus">-</span></div>
    <div class="field"><label>Broker-IP</label><input type="text" id="mqttServer" placeholder="192.168.1.10"></div>
    <div class="subrow">
      <div class="field"><label>Port</label><input type="text" id="mqttPort" placeholder="1883"></div>
      <div class="field"><label>Benutzer</label><input type="text" id="mqttUser" placeholder="optional"></div>
    </div>
    <div class="field"><label>Passwort</label><input type="password" id="mqttPass" placeholder="unverändert lassen"></div>
  </div>

  <div class="panel">
    <h2>SignalK</h2>
    <div class="status-line"><span>Status</span><span id="skStatus">-</span></div>
    <div class="subrow">
      <div class="field"><label>Host-IP (leer = aus)</label><input type="text" id="skHost" placeholder="192.168.1.20"></div>
      <div class="field"><label>Port</label><input type="text" id="skPort" placeholder="3000"></div>
    </div>
    <div class="field"><label>Token</label><input type="password" id="skToken" placeholder="unverändert lassen"></div>
  </div>

  <div class="panel">
    <h2>OTA-Update</h2>
    <div class="field"><label>Passwort</label><input type="password" id="otaPass" placeholder="unverändert lassen"></div>
    <button id="saveConfigBtn">Einstellungen speichern</button>
  </div>

<script>
  const stateEl = document.getElementById('state');
  const clockEl = document.getElementById('clock');
  const minutesEl = document.getElementById('minutes');
  const stopBtn = document.getElementById('stopBtn');
  const canvas = document.getElementById('historyCanvas');
  const ctx = canvas.getContext('2d');
  const historyListEl = document.getElementById('historyList');
  let editingMinutes = false;
  let currentIntervalMinutes = 30;

  minutesEl.addEventListener('focus', () => editingMinutes = true);
  minutesEl.addEventListener('blur', () => editingMinutes = false);

  function fmt(s){
    const m = Math.floor(s/60), sec = s%60;
    return String(m).padStart(2,'0') + ':' + String(sec).padStart(2,'0');
  }

  function drawHistory(entries){
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    canvas.width = w * dpr; canvas.height = h * dpr;
    ctx.setTransform(dpr,0,0,dpr,0,0);
    ctx.clearRect(0,0,w,h);

    if (entries.length === 0){
      ctx.fillStyle = '#6b7580';
      ctx.font = '11px monospace';
      ctx.fillText('Noch keine Quittierungen', 4, h/2);
      return;
    }

    const expectedSec = currentIntervalMinutes * 60;
    const maxGap = Math.max(expectedSec * 1.5, ...entries.map(e => e.gapSeconds));
    const barW = Math.max(3, w / entries.length - 2);

    entries.forEach((e, i) => {
      const barH = maxGap > 0 ? (e.gapSeconds / maxGap) * (h - 4) : 0;
      const x = i * (w / entries.length);
      const tooLong = e.gapSeconds > expectedSec * 1.15;
      ctx.fillStyle = e.gapSeconds === 0 ? '#1e2830' : (tooLong ? '#ff4433' : '#e8a33d');
      ctx.fillRect(x, h - barH, barW, barH);
    });
  }

  function renderHistoryList(entries){
    const last = entries.slice(-8).reverse();
    const expectedSec = currentIntervalMinutes * 60;
    historyListEl.innerHTML = last.map(e => {
      const cls = e.gapSeconds === 0 ? '' : (e.gapSeconds > expectedSec * 1.15 ? 'gap-warn' : 'gap-ok');
      const gapTxt = e.gapSeconds === 0 ? '-' : fmt(e.gapSeconds);
      return `<div><span>${e.time}</span><span class="${cls}">${gapTxt}</span></div>`;
    }).join('');
  }

  async function loadHistory(){
    try{
      const r = await fetch('/history');
      const entries = await r.json();
      drawHistory(entries);
      renderHistoryList(entries);
    }catch(e){ /* Netzwerkfehler ignorieren */ }
  }

  async function poll(){
    try{
      const r = await fetch('/status');
      const d = await r.json();
      stateEl.textContent = d.state === 'ALARM' ? 'ALARM' : (d.state === 'COUNTING' ? 'LÄUFT' : d.state);
      clockEl.textContent = d.state === 'ALARM' ? 'ALARM' : fmt(d.remainingSeconds);
      clockEl.style.color = d.state === 'ALARM' ? '#ff4433' : '#e8a33d';
      if (!editingMinutes) minutesEl.value = d.intervalMinutes;
      currentIntervalMinutes = d.intervalMinutes;
      stopBtn.disabled = d.state !== 'ALARM';
    }catch(e){ /* Netzwerkfehler ignorieren, naechster Poll folgt */ }
  }

  document.getElementById('applyBtn').addEventListener('click', async () => {
    const v = parseInt(minutesEl.value, 10) || 30;
    await fetch('/interval', {
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'minutes=' + v
    });
    poll();
  });

  stopBtn.addEventListener('click', async () => {
    await fetch('/stop', { method:'POST' });
    poll();
    setTimeout(loadHistory, 500);
  });

  // ----- Einstellungen-Panel (MQTT/SignalK/OTA/WLAN) -----
  const fServer = document.getElementById('mqttServer');
  const fPort   = document.getElementById('mqttPort');
  const fUser   = document.getElementById('mqttUser');
  const fPass   = document.getElementById('mqttPass');
  const fSkHost = document.getElementById('skHost');
  const fSkPort = document.getElementById('skPort');
  const fSkToken= document.getElementById('skToken');
  const fOtaPass= document.getElementById('otaPass');
  const mqttStatusEl = document.getElementById('mqttStatus');
  const skStatusEl   = document.getElementById('skStatus');
  const wifiInfoEl   = document.getElementById('wifiInfo');

  function isEditing(el){ return document.activeElement === el; }

  async function loadConfig(){
    try{
      const r = await fetch('/config');
      const d = await r.json();
      wifiInfoEl.textContent = d.wifiSsid + ' · ' + d.wifiIp + ' · ' + d.wifiRssi + ' dBm';

      if (!isEditing(fServer)) fServer.value = d.mqttServer;
      if (!isEditing(fPort))   fPort.value = d.mqttPort;
      if (!isEditing(fUser))   fUser.value = d.mqttUser;
      mqttStatusEl.textContent = d.mqttConnected ? 'verbunden' : 'getrennt';
      mqttStatusEl.className = d.mqttConnected ? 'ok' : 'warn';

      if (!isEditing(fSkHost)) fSkHost.value = d.skHost;
      if (!isEditing(fSkPort)) fSkPort.value = d.skPort;
      skStatusEl.textContent = !d.skEnabled ? 'deaktiviert' : (d.skConnected ? 'verbunden' : 'getrennt');
      skStatusEl.className = d.skConnected ? 'ok' : 'warn';
    }catch(e){ /* Netzwerkfehler ignorieren */ }
  }

  document.getElementById('saveConfigBtn').addEventListener('click', async () => {
    const body = new URLSearchParams({
      mqtt_server: fServer.value,
      mqtt_port: fPort.value,
      mqtt_user: fUser.value,
      mqtt_pass: fPass.value,
      sk_host: fSkHost.value,
      sk_port: fSkPort.value,
      sk_token: fSkToken.value,
      ota_pass: fOtaPass.value
    });
    await fetch('/config', {
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body
    });
    fPass.value = ''; fSkToken.value = ''; fOtaPass.value = '';
    loadConfig();
  });

  document.getElementById('wifiResetBtn').addEventListener('click', async () => {
    if (!confirm('WLAN-Zugangsdaten löschen und neu starten? Danach muss das WLAN über das Setup-Netz "WachTimer-Setup" neu eingerichtet werden.')) return;
    await fetch('/wifi_reset', { method:'POST' });
    alert('Neustart läuft. Falls kein bekanntes WLAN gefunden wird, verbinde dich mit dem Netz "WachTimer-Setup".');
  });

  poll();
  loadHistory();
  loadConfig();
  setInterval(poll, 1000);
  setInterval(loadHistory, 5000);
  setInterval(loadConfig, 5000);
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String stateStr = (state == ALARMING) ? "ALARM" : "COUNTING";
  String json = "{";
  json += "\"state\":\"" + stateStr + "\",";
  json += "\"remainingSeconds\":" + String(remainingSeconds()) + ",";
  json += "\"intervalMinutes\":" + String(INTERVAL_MINUTES);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetInterval() {
  if (!server.hasArg("minutes")) {
    server.send(400, "text/plain", "minutes fehlt");
    return;
  }
  int32_t minutes = server.arg("minutes").toInt();
  applyIntervalChange(minutes);
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  if (state == ALARMING) {
    Serial.println("[Timer] Web-STOPP gedrueckt -> Alarm gestoppt, Timer neu gestartet.");
    stopAlarmAndRestart();
  }
  server.send(200, "text/plain", "OK");
}

void handleHistory() {
  server.send(200, "application/json", historyToJson());
}

// Liefert aktuellen Verbindungsstatus + Konfiguration (ohne Passwoerter/
// Token im Klartext zurueckzugeben - die Eingabefelder bleiben beim Laden
// leer, ein leeres Feld beim Speichern bedeutet "unveraendert lassen").
void handleConfig() {
  String json = "{";
  json += "\"wifiSsid\":\"" + WiFi.SSID() + "\",";
  json += "\"wifiIp\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"wifiRssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"mqttServer\":\"" + String(mqttServerBuf) + "\",";
  json += "\"mqttPort\":\"" + String(mqttPortBuf) + "\",";
  json += "\"mqttUser\":\"" + String(mqttUserBuf) + "\",";
  json += "\"mqttConnected\":" + String(mqttClient.connected() ? "true" : "false") + ",";
  json += "\"skHost\":\"" + String(skHostBuf) + "\",";
  json += "\"skPort\":\"" + String(skPortBuf) + "\",";
  json += "\"skEnabled\":" + String(skEnabled ? "true" : "false") + ",";
  json += "\"skConnected\":" + String((skEnabled && skWs.isConnected()) ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleConfigSave() {
  if (server.hasArg("mqtt_server")) server.arg("mqtt_server").toCharArray(mqttServerBuf, sizeof(mqttServerBuf));
  if (server.hasArg("mqtt_port"))   server.arg("mqtt_port").toCharArray(mqttPortBuf, sizeof(mqttPortBuf));
  if (server.hasArg("mqtt_user"))   server.arg("mqtt_user").toCharArray(mqttUserBuf, sizeof(mqttUserBuf));
  if (server.hasArg("mqtt_pass") && server.arg("mqtt_pass").length() > 0) {
    server.arg("mqtt_pass").toCharArray(mqttPassBuf, sizeof(mqttPassBuf));
  }
  if (server.hasArg("sk_host")) server.arg("sk_host").toCharArray(skHostBuf, sizeof(skHostBuf));
  if (server.hasArg("sk_port")) server.arg("sk_port").toCharArray(skPortBuf, sizeof(skPortBuf));
  if (server.hasArg("sk_token") && server.arg("sk_token").length() > 0) {
    server.arg("sk_token").toCharArray(skTokenBuf, sizeof(skTokenBuf));
  }
  if (server.hasArg("ota_pass") && server.arg("ota_pass").length() > 0) {
    server.arg("ota_pass").toCharArray(otaPassBuf, sizeof(otaPassBuf));
    ArduinoOTA.setPassword(otaPassBuf);
  }

  prefs.putString("mqttServer", mqttServerBuf);
  prefs.putString("mqttPort", mqttPortBuf);
  prefs.putString("mqttUser", mqttUserBuf);
  prefs.putString("mqttPass", mqttPassBuf);
  prefs.putString("skHost", skHostBuf);
  prefs.putString("skPort", skPortBuf);
  prefs.putString("skToken", skTokenBuf);
  prefs.putString("otaPass", otaPassBuf);

  // MQTT sofort mit den neuen Werten neu verbinden
  mqttClient.disconnect();
  mqttClient.setServer(mqttServerBuf, atoi(mqttPortBuf));
  lastMqttAttempt = 0;   // erlaubt sofortigen Reconnect-Versuch im naechsten loop()

  // SignalK-Verbindung mit neuen Werten neu aufbauen (oder deaktivieren,
  // falls das Host-Feld geleert wurde)
  if (skEnabled) {
    skWs.disconnect();
  }
  setupSignalK();

  Serial.println("[Config] Einstellungen ueber Weboberflaeche aktualisiert.");
  server.send(200, "text/plain", "OK");
}

// Loescht die gespeicherten WLAN-Zugangsdaten und startet neu - danach
// oeffnet sich beim naechsten Boot wieder das "WachTimer-Setup"-Portal.
void handleWifiReset() {
  server.send(200, "text/plain", "OK");
  delay(200);
  wm.resetSettings();
  delay(200);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Gespeichertes Intervall aus dem NVS-Flash laden (Standard 30 Min.,
  // falls noch nie etwas gespeichert wurde)
  prefs.begin("wachtimer", false);
  INTERVAL_MINUTES = prefs.getUInt("interval", 30);
  INTERVAL_MS = INTERVAL_MINUTES * 60UL * 1000UL;

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastRawState = digitalRead(BUTTON_PIN);
  stableState  = lastRawState;

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  lastEncoded = (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), encoderISR, CHANGE);

  u8g2.begin();
  u8g2.setContrast(255);

  // Gespeicherte Netzwerk-/Broker-Konfiguration aus dem NVS laden (falls
  // vorhanden), damit das Setup-Portal beim erneuten Aufruf die zuletzt
  // gesetzten Werte vorausfuellt
  prefs.getString("mqttServer", "").toCharArray(mqttServerBuf, sizeof(mqttServerBuf));
  prefs.getString("mqttPort", "1883").toCharArray(mqttPortBuf, sizeof(mqttPortBuf));
  prefs.getString("mqttUser", "").toCharArray(mqttUserBuf, sizeof(mqttUserBuf));
  prefs.getString("mqttPass", "").toCharArray(mqttPassBuf, sizeof(mqttPassBuf));
  prefs.getString("skHost", "").toCharArray(skHostBuf, sizeof(skHostBuf));
  prefs.getString("skPort", "3000").toCharArray(skPortBuf, sizeof(skPortBuf));
  prefs.getString("skToken", "").toCharArray(skTokenBuf, sizeof(skTokenBuf));
  prefs.getString("otaPass", "wachtimer").toCharArray(otaPassBuf, sizeof(otaPassBuf));

  WiFiManagerParameter p_mqtt_server("mqtt_server", "MQTT Broker IP", mqttServerBuf, sizeof(mqttServerBuf));
  WiFiManagerParameter p_mqtt_port("mqtt_port", "MQTT Port", mqttPortBuf, sizeof(mqttPortBuf));
  WiFiManagerParameter p_mqtt_user("mqtt_user", "MQTT Benutzer (optional)", mqttUserBuf, sizeof(mqttUserBuf));
  WiFiManagerParameter p_mqtt_pass("mqtt_pass", "MQTT Passwort (optional)", mqttPassBuf, sizeof(mqttPassBuf));
  WiFiManagerParameter p_sk_host("sk_host", "SignalK Host-IP (leer = aus)", skHostBuf, sizeof(skHostBuf));
  WiFiManagerParameter p_sk_port("sk_port", "SignalK Port", skPortBuf, sizeof(skPortBuf));
  WiFiManagerParameter p_sk_token("sk_token", "SignalK Token (optional)", skTokenBuf, sizeof(skTokenBuf));
  WiFiManagerParameter p_ota_pass("ota_pass", "OTA-Passwort", otaPassBuf, sizeof(otaPassBuf));

  wm.setSaveConfigCallback(saveConfigCallback);
  wm.addParameter(&p_mqtt_server);
  wm.addParameter(&p_mqtt_port);
  wm.addParameter(&p_mqtt_user);
  wm.addParameter(&p_mqtt_pass);
  wm.addParameter(&p_sk_host);
  wm.addParameter(&p_sk_port);
  wm.addParameter(&p_sk_token);
  wm.addParameter(&p_ota_pass);

  // Nach 3 Minuten ohne Konfiguration im Setup-Portal weiterlaufen
  // (der Timer funktioniert dann offline weiter, nur Web/MQTT/SignalK
  // sind nicht erreichbar)
  wm.setConfigPortalTimeout(180);

  // Verbindet mit dem zuletzt bekannten WLAN. Gelingt das nicht, wird
  // ein eigenes Setup-WLAN "WachTimer-Setup" aufgemacht, in dem sich
  // ein Konfigurationsportal öffnet (WLAN + obige Parameter).
  bool wifiOk = wm.autoConnect("WachTimer-Setup");

  if (shouldSaveConfig) {
    strncpy(mqttServerBuf, p_mqtt_server.getValue(), sizeof(mqttServerBuf) - 1);
    strncpy(mqttPortBuf, p_mqtt_port.getValue(), sizeof(mqttPortBuf) - 1);
    strncpy(mqttUserBuf, p_mqtt_user.getValue(), sizeof(mqttUserBuf) - 1);
    strncpy(mqttPassBuf, p_mqtt_pass.getValue(), sizeof(mqttPassBuf) - 1);
    strncpy(skHostBuf, p_sk_host.getValue(), sizeof(skHostBuf) - 1);
    strncpy(skPortBuf, p_sk_port.getValue(), sizeof(skPortBuf) - 1);
    strncpy(skTokenBuf, p_sk_token.getValue(), sizeof(skTokenBuf) - 1);
    strncpy(otaPassBuf, p_ota_pass.getValue(), sizeof(otaPassBuf) - 1);

    prefs.putString("mqttServer", mqttServerBuf);
    prefs.putString("mqttPort", mqttPortBuf);
    prefs.putString("mqttUser", mqttUserBuf);
    prefs.putString("mqttPass", mqttPassBuf);
    prefs.putString("skHost", skHostBuf);
    prefs.putString("skPort", skPortBuf);
    prefs.putString("skToken", skTokenBuf);
    prefs.putString("otaPass", otaPassBuf);
    Serial.println("[Config] Neue Netzwerk-/Broker-Konfiguration gespeichert.");
  }

  if (wifiOk) {
    Serial.print("[WiFi] Verbunden, IP-Adresse: ");
    Serial.println(WiFi.localIP());
    if (MDNS.begin(MDNS_HOSTNAME)) {
      Serial.println("[WiFi] Erreichbar unter http://" + String(MDNS_HOSTNAME) + ".local");
    }
    configTzTime(TZ_INFO, NTP_SERVER);   // fuer Zeitstempel im Verlaufsprotokoll
  } else {
    Serial.println("[WiFi] Kein WLAN verbunden - Timer laeuft trotzdem lokal weiter.");
  }

  // Watchdog: resettet den ESP32, falls loop() mal WDT_TIMEOUT_S lang
  // haengen bleibt (z.B. blockierender WLAN/I2C-Fehler)
  // Struct-basierte API (ESP32-Arduino-Core 3.x / ESP-IDF 5.x).
  esp_task_wdt_config_t twdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,       // keine Idle-Tasks ueberwachen, nur unseren loop()
    .trigger_panic = true,     // bei Timeout Reset statt nur Warnung
  };
  esp_task_wdt_init(&twdtConfig);
  esp_task_wdt_add(NULL);

  setupOTA();
  setupSignalK();

  mqttClient.setServer(mqttServerBuf, atoi(mqttPortBuf));
  mqttClient.setCallback(mqttCallback);
  connectMqtt();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/interval", HTTP_POST, handleSetInterval);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/history", HTTP_GET, handleHistory);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/config", HTTP_POST, handleConfigSave);
  server.on("/wifi_reset", HTTP_POST, handleWifiReset);
  server.begin();

  startCountdown();
}

void loop() {
  esp_task_wdt_reset();   // Watchdog fuettern - muss jeden Durchlauf passieren

  ArduinoOTA.handle();
  if (otaInProgress) return;   // waehrend eines Updates alles andere pausieren

  switch (state) {
    case COUNTING:
      handleCounting();
      break;
    case ALARMING:
      handleAlarming();
      break;
  }
  updateDisplay();

  maintainWifi();

  if (mqttClient.connected()) {
    mqttClient.loop();
  } else {
    connectMqtt();
  }

  if (skEnabled) {
    skWs.loop();
  }

  if (millis() - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = millis();
    publishState();
    publishSignalK();
  }

  server.handleClient();
}
