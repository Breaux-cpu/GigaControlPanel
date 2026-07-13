/*
 * GIGA Control Panel
 * ------------------
 * Multi-tab touch control center for:
 *   Arduino GIGA R1 WiFi + GIGA Display Shield + Arduino 4 Relays Shield
 *
 * Tabs (left icon rail):
 *   Home      — live status cards, A0 gauge, ALL-OFF kill button
 *   Relays    — 4 relay switches with indicator dots (shield pins 4/7/8/12)
 *   Motors    — 2 motor channels (enable / direction / speed) for an H-bridge
 *   Sensors   — live line chart for A0 + bars for A0..A3
 *   IMU       — Display Shield BMI270: live accelerometer + gyroscope
 *   Audio     — Display Shield PDM microphone level + tone out the audio jack
 *   WiFi      — scan, pick a network, on-screen keyboard, connect/disconnect
 *   Bluetooth — BLE peripheral: control relays & motors from a phone
 *   Settings  — brightness, sensor rate, system info
 *
 * Required libraries (Library Manager):
 *   lvgl                      9.x     (tested with 9.5.0; core 4.6.0 targets v9)
 *   Arduino_GigaDisplayTouch  latest
 *   Arduino_GigaDisplay       latest  (backlight control)
 *   Arduino_BMI270_BMM150     latest  (shield IMU, on Wire1)
 *   Arduino_AdvancedAnalog    latest  (DAC tone on the audio jack)
 *   ArduinoBLE                latest
 *   MQTT                      2.5.x   (256dpi/arduino-mqtt; MQTT remote tab)
 * (Arduino_H7_Video, WiFi and PDM ship with the "Arduino Mbed OS GIGA Boards" core.)
 *
 * Board: Tools > Board > Arduino Mbed OS GIGA Boards > Arduino GIGA R1 WiFi
 */

#include <Arduino_H7_Video.h>
#include <Arduino_GigaDisplayTouch.h>
#include <Arduino_GigaDisplay.h>
#include <lvgl.h>
#include <WiFi.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>
#include <PDM.h>
#include <Arduino_AdvancedAnalog.h>
#include <MQTTClient.h>             /* 256dpi/arduino-mqtt -- remote monitor + control over MQTT */
#include <Arduino_Portenta_OTA.h>   /* WiFi firmware update (GIGA shares Portenta H7's OTA path) */
#include "drivers/Watchdog.h"        /* hardware watchdog -- auto-recover from any future freeze */

/* ================= hardware configuration ================= */

/* Arduino 4 Relays Shield — fixed pin mapping, active HIGH */
static const uint8_t RELAY_PIN[4]   = { 4, 7, 8, 12 };
static const char   *RELAY_NAME[4]  = { "Relay 1", "Relay 2", "Relay 3", "Relay 4" };

/* Motor channels — wire to an external H-bridge (e.g. L298N: PWM->EN, DIR->IN).
 * Pins chosen to NOT collide with the relay shield (4/7/8/12). */
struct MotorCfg { uint8_t pwmPin; uint8_t dirPin; const char *name; };
static const MotorCfg MOTOR[2] = { { 2, 3, "Motor A" }, { 5, 6, "Motor B" } };

/* Analog inputs shown on the Sensors tab */
static const uint8_t ANALOG_PIN[4]  = { A0, A1, A2, A3 };
static const char   *ANALOG_NAME[4] = { "A0", "A1", "A2", "A3" };

#define BLE_DEVICE_NAME "GIGA-Control"

/* mbed-core gotcha: Serial writes BLOCK the sketch forever if a host monitor
 * attached once and stopped draining. Only print while a monitor is present. */
#define DBG(...) do { if (Serial) { Serial.println(__VA_ARGS__); } } while (0)

/* debug bisect switches — turn shield peripherals off to isolate a freeze */
#define EN_IMU 1     /* BMI270 on Wire1 (shared with the GT911 touch controller!) */
#define EN_MIC 1     /* PDM microphone (ISR capture) */

/* Hardware watchdog: if the sketch ever hangs (a bug nobody's found yet,
 * a peripheral fault, anything), the board resets itself instead of
 * needing a manual power-cycle -- see setup()/loop() for start + the
 * per-iteration kick. WiFi.begin()/scanNetworks()/checkCaptivePortal()
 * are the only calls in this file that can legitimately block long enough
 * to matter (WiFi.begin() alone can take up to ~7s to connect *after* an
 * internal scan that can itself take several seconds -- confirmed by
 * reading WiFi.cpp: begin() with no explicit security always re-scans
 * first), so each of those gets an explicit kick on either side of the
 * call on top of the per-loop() kick. */
static void wdKick() { mbed::Watchdog::get_instance().kick(); }
static uint32_t wdTimeout = 20000;   /* set for real in setup(); global so it can be
                                      * reused when performReconScan() pauses/resumes
                                      * the watchdog around its long connect() calls */

/* ---- persistent settings (flash-backed, survive reboot & re-upload) ---- */
#define EN_KV 1      /* persistent settings across reboots (WiFi creds, BLE, brightness, rate) */
#if EN_KV
#include "kvstore_global_api.h"
static void kvSaveStr(const char *key, const char *val) { kv_set(key, val, strlen(val), 0); }
static bool kvLoadStr(const char *key, char *buf, size_t n) {
  size_t got = 0;
  if (kv_get(key, buf, n - 1, &got) != MBED_SUCCESS) return false;
  buf[got] = '\0';
  return got > 0;
}
static void kvSaveU8(const char *key, uint8_t v) { kv_set(key, &v, 1, 0); }
static uint8_t kvLoadU8(const char *key, uint8_t dflt) {
  uint8_t v; size_t got = 0;
  return (kv_get(key, &v, 1, &got) == MBED_SUCCESS && got == 1) ? v : dflt;
}
static void kvRemove(const char *key) { kv_remove(key); }
#else
static void kvSaveStr(const char *, const char *) {}
static bool kvLoadStr(const char *, char *, size_t) { return false; }
static void kvSaveU8(const char *, uint8_t) {}
static uint8_t kvLoadU8(const char *, uint8_t dflt) { return dflt; }
static void kvRemove(const char *) {}
#endif

/* ================= palette ================= */

#define C_BG_TOP   0x0B1026   /* deep navy  (screen gradient top)    */
#define C_BG_BOT   0x1D2E5C   /* indigo     (screen gradient bottom) */
#define C_CARD     0x141E38   /* card body                           */
#define C_CARD_BRD 0x27355A   /* card border                         */
#define C_DOT_OFF  0x2A3A5E   /* indicator dot, off                  */
#define C_ACCENT   0x38BDF8   /* cyan accent                         */
#define C_OK       0x4ADE80   /* green                               */
#define C_WARN     0xFBBF24   /* amber                               */
#define C_DANGER   0xF87171   /* red                                 */
#define C_TEXT     0xE2E8F0   /* primary text                        */
#define C_MUTED    0x94A3B8   /* secondary text                      */

/* Bigger fonts if the active lv_conf.h enables them, else fall back */
#if LV_FONT_MONTSERRAT_22
  #define FONT_TITLE &lv_font_montserrat_22
#else
  #define FONT_TITLE LV_FONT_DEFAULT
#endif
#if LV_FONT_MONTSERRAT_16
  #define FONT_CARD &lv_font_montserrat_16
#else
  #define FONT_CARD LV_FONT_DEFAULT
#endif

/* ================= globals ================= */

Arduino_H7_Video          Display(800, 480, GigaDisplayShield);
Arduino_GigaDisplayTouch  TouchDetector;
GigaDisplayBacklight      Backlight;
BoschSensorClass          imu(Wire1);          /* Display Shield BMI270 */
AdvancedDAC               dac0(A12);           /* audio jack tone out   */

/* state */
static bool     relayState[4] = { false, false, false, false };
static uint8_t  motorSpeed[2] = { 0, 0 };      /* 0..100 %      */
static bool     motorFwd[2]   = { true, true };
static bool     motorEn[2]    = { false, false };
static bool     bleEnabled    = false;
static bool     bleReady      = false;
static bool     imuOk         = false;
static bool     micOk         = false;
static bool     toneOn        = false;
static uint16_t toneFreq      = 440;
static uint16_t sineLut[64];
static char     pendingSsid[64];
static char     pendingPass[64];
static char     ssidCache[8][40];   /* capped low to bound peak memory during scan population */

/* microphone capture (PDM callback runs in interrupt context) */
static short         micBuf[512];
static volatile int  micSamples = 0;

/* widgets that get updated at runtime */
static lv_obj_t *tabview;
static lv_obj_t *relaySw[4],  *relayDot[4], *dashDot[4];
static lv_obj_t *motorSlider[2], *motorDirSw[2], *motorEnSw[2], *motorValLbl[2];
static lv_obj_t *sensorBar[4], *sensorLbl[4];
static lv_obj_t *chart;          static lv_chart_series_t *chartSer;
static lv_obj_t *gaugeArc, *gaugeLbl;
static lv_obj_t *dashWifiLbl, *dashBleLbl, *dashUpLbl;
static lv_obj_t *wifiStatusLbl, *wifiList, *scanBtn;
static lv_obj_t *bleStatusLbl, *bleSw;
static lv_obj_t *pwModal = NULL, *pwTa, *pwTitle, *pwKb;
static lv_obj_t *ssidModal = NULL, *ssidTa, *ssidKb;
static lv_obj_t *reconModal = NULL, *reconTa, *reconKb;
static lv_obj_t *reconList, *reconStatusLbl, *reconHistLbl;
static char     targetIp[40] = "";
static uint16_t reconPorts[8];      /* capped at 8, same proven-safe limit as the port list below */
static int      reconPortCount = 0;
static lv_obj_t *accBar[3], *gyroBar[3], *accLbl, *gyroLbl, *imuStatusLbl;
static lv_obj_t *micBar, *micLbl, *micChart, *toneFreqLbl;
static lv_chart_series_t *micSer;
static lv_obj_t *sysInfoLbl;
static lv_timer_t *sensorTimer;

/* ---- MQTT (remote monitor + control) ----
 * All topics live under MQTT_PREFIX. The GIGA publishes its own state
 * (relays / A0) and subscribes to a single command topic that mirrors the
 * BLE relay characteristic exactly (a 0..15 bitmask). mqttNet is a plain
 * WiFiClient, so mqttClient.connect() carries the same "connect() to an
 * unreachable host isn't bounded by any timeout call on this core" caveat
 * the Recon scan already documents -- performMqttConnect() pauses the
 * watchdog around it for that reason, same as performReconScan(). */
#define MQTT_PREFIX   "giga-control"
#define MQTT_CLIENTID "giga-control-r1"
static WiFiClient    mqttNet;
static MQTTClient    mqttClient(256);
static char          mqttHost[64] = "";
static uint16_t      mqttPort     = 1883;
static bool          mqttEnabled  = false;   /* user wants a connection (drives auto-reconnect) */
static volatile bool mqttRelaysDirty = false;/* a relay changed -- republish on the next loop() */
static uint16_t      mqttConnectPending = 0; /* loop() runs the (possibly-blocking) connect */
static uint8_t       mqttClosePending   = 0;
static bool          mqttSubmitted      = false;
static lv_obj_t *mqttModal = NULL, *mqttTa, *mqttKb;

/* ---- OTA (WiFi firmware update, MQTT-triggered) ----
 * Publish a packaged .ota image's URL to MQTT_PREFIX "/ota/update"; the GIGA
 * pulls it over the WiFi it's already on, stages it to QSPI, and lets the
 * bootloader flash it on reset. The set-then-run split keeps the (blocking,
 * multi-second) download out of the MQTT callback -- loop() runs it. USB DFU
 * stays the recovery path if an image is ever bad. Build/serve/publish with
 * tools/giga-ota.sh on jessy. */
static volatile bool otaRequested = false;
static String        otaUrl;                 /* full URL, taken from the MQTT payload */
static lv_obj_t *mqttStatusLbl, *mqttBrokerLbl, *mqttActivityLbl, *mqttSw;
/* The MQTT tab is built LAZILY -- its widgets exist only while the tab is
 * actually on screen, then are freed on leaving it. This 11th tab's
 * permanent widgets (~1.7KB of LVGL pool) don't fit alongside the boot-time
 * WiFi/captive-portal allocations on the already-maxed 64KB pool (proven:
 * a permanent version boot-looped via OOM). Transient widgets, like a
 * modal, are fine even at the low free the tab causes -- and at boot/idle
 * the page is empty, so idle free stays at the stable ~6.8KB. loop()
 * reconciles built-state against the active tab, outside any LVGL event
 * dispatch (same safety rule as every other deferred action here). */
static lv_obj_t *mqttTabPage  = NULL;
static bool      mqttViewBuilt = false;
/* The broker keyboard modal (~2.2KB) and the built tab (~1.6KB) don't fit
 * together in the pool, so opening the modal first frees the tab (which the
 * modal covers anyway). Both actions are deferred to loop() via these flags
 * so the tab -- which holds the very button whose click requests the modal
 * -- is never deleted from inside its own event callback (the delete-in-
 * callback hazard that hard-froze this board twice before). */
static volatile bool mqttModalWant = false;
static bool          mqttModalOpen = false;

/* styles */
static lv_style_t stCard, stCardTitle, stMuted, stBig;

/* BLE GATT: one service, three characteristics
 *   relay  (1 byte, R/W)   bit0..bit3 = relay 1..4
 *   motor  (3 bytes, W)    [index 0/1, speed 0..100, dir 0=rev 1=fwd]
 *   sensor (2 bytes, R/N)  raw A0 reading, little-endian            */
BLEService              ctrlService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic   relayChar  ("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic       motorChar  ("19B10002-E8F2-537E-4F6C-D104768A1214", BLEWrite, 3);
BLEUnsignedShortCharacteristic sensorChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

/* ================= small helpers ================= */

/* indicator dot (plain object — no widget-config dependency) */
static lv_obj_t *makeDot(lv_obj_t *parent, int size) {
  lv_obj_t *d = lv_obj_create(parent);
  lv_obj_set_size(d, size, size);
  lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(d, 0, 0);
  lv_obj_set_style_bg_color(d, lv_color_hex(C_DOT_OFF), 0);
  lv_obj_remove_flag(d, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(d, LV_OBJ_FLAG_CLICKABLE);
  return d;
}

static void setDot(lv_obj_t *d, bool on) {
  lv_obj_set_style_bg_color(d, lv_color_hex(on ? C_OK : C_DOT_OFF), 0);
  lv_obj_set_style_shadow_width(d, on ? 16 : 0, 0);      /* green glow */
  lv_obj_set_style_shadow_color(d, lv_color_hex(C_OK), 0);
  lv_obj_set_style_shadow_opa(d, LV_OPA_70, 0);
}

static void applyRelay(int i, bool on) {
  relayState[i] = on;
  digitalWrite(RELAY_PIN[i], on ? HIGH : LOW);
  setDot(relayDot[i], on);
  setDot(dashDot[i], on);
  if (on) lv_obj_add_state(relaySw[i], LV_STATE_CHECKED);
  else    lv_obj_remove_state(relaySw[i], LV_STATE_CHECKED);
  mqttRelaysDirty = true;   /* republished on the next loop() if MQTT is connected */
}

static void applyMotor(int i) {
  digitalWrite(MOTOR[i].dirPin, motorFwd[i] ? HIGH : LOW);
  analogWrite(MOTOR[i].pwmPin, motorEn[i] ? (motorSpeed[i] * 255) / 100 : 0);
  lv_label_set_text_fmt(motorValLbl[i], "%d%%  %s  %s",
                        motorSpeed[i],
                        motorFwd[i] ? "FWD" : "REV",
                        motorEn[i] ? "ON" : "OFF");
  lv_obj_set_style_text_color(motorValLbl[i],
      lv_color_hex(motorEn[i] ? C_OK : C_MUTED), 0);
}

static void allOff() {
  for (int i = 0; i < 4; i++) applyRelay(i, false);
  for (int i = 0; i < 2; i++) {
    motorEn[i] = false;
    lv_obj_remove_state(motorEnSw[i], LV_STATE_CHECKED);
    applyMotor(i);
  }
}

/* card factory: rounded, bordered, soft shadow */
static lv_obj_t *makeCard(lv_obj_t *parent, int32_t w, int32_t h) {
  lv_obj_t *c = lv_obj_create(parent);
  lv_obj_set_size(c, w, h);
  lv_obj_add_style(c, &stCard, 0);
  lv_obj_remove_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  return c;
}

static lv_obj_t *cardTitle(lv_obj_t *card, const char *icon, const char *txt) {
  lv_obj_t *l = lv_label_create(card);
  lv_label_set_text_fmt(l, "%s  %s", icon, txt);
  lv_obj_add_style(l, &stCardTitle, 0);
  lv_obj_align(l, LV_ALIGN_TOP_LEFT, 0, 0);
  return l;
}

/* symmetric bar for signed sensor values */
static lv_obj_t *makeSignedBar(lv_obj_t *parent, int32_t min, int32_t max, int y) {
  lv_obj_t *b = lv_bar_create(parent);
  lv_bar_set_range(b, min, max);
  lv_bar_set_mode(b, LV_BAR_MODE_SYMMETRICAL);
  lv_obj_set_size(b, 260, 12);
  lv_obj_align(b, LV_ALIGN_TOP_LEFT, 0, y);
  lv_obj_set_style_bg_color(b, lv_color_hex(0x0E1730), LV_PART_MAIN);
  lv_obj_set_style_bg_color(b, lv_color_hex(C_ACCENT), LV_PART_INDICATOR);
  return b;
}

/* ================= event callbacks ================= */

static void relaySwCb(lv_event_t *e) {
  int i = (int)(uintptr_t)lv_event_get_user_data(e);
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
  applyRelay(i, lv_obj_has_state(sw, LV_STATE_CHECKED));
  if (bleReady) relayChar.writeValue((relayState[0] << 0) | (relayState[1] << 1) |
                                     (relayState[2] << 2) | (relayState[3] << 3));
}

static void motorSliderCb(lv_event_t *e) {
  int i = (int)(uintptr_t)lv_event_get_user_data(e);
  motorSpeed[i] = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
  applyMotor(i);
}

static void motorDirCb(lv_event_t *e) {
  int i = (int)(uintptr_t)lv_event_get_user_data(e);
  motorFwd[i] = lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED);
  applyMotor(i);
}

static void motorEnCb(lv_event_t *e) {
  int i = (int)(uintptr_t)lv_event_get_user_data(e);
  motorEn[i] = lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED);
  applyMotor(i);
}

static void allOffCb(lv_event_t *e) { (void)e; allOff(); }

static void brightnessCb(lv_event_t *e) {
  Backlight.set(lv_slider_get_value((lv_obj_t *)lv_event_get_target(e)));
}

/* save only on release, not on every drag step (flash wear) */
static void brightnessSaveCb(lv_event_t *e) {
  kvSaveU8("/kv/bright", (uint8_t)lv_slider_get_value((lv_obj_t *)lv_event_get_target(e)));
}

static void sensorRateCb(lv_event_t *e) {
  static const uint16_t rates[] = { 100, 300, 1000 };
  uint32_t sel = lv_dropdown_get_selected((lv_obj_t *)lv_event_get_target(e));
  if (sel < 3 && sensorTimer) lv_timer_set_period(sensorTimer, rates[sel]);
  kvSaveU8("/kv/rate", (uint8_t)sel);
}

/* ---------------- audio ---------------- */

static void onPdmData() {                     /* interrupt context: copy only */
  int bytes = PDM.available();
  if (bytes > (int)sizeof(micBuf)) bytes = sizeof(micBuf);
  PDM.read(micBuf, bytes);
  micSamples = bytes / 2;
}

/* One full sine cycle sampled at 64 points -- the shape never depends on
 * toneFreq (that's set via the DAC's sample clock below), so it's built
 * once here rather than recomputed with 64 sinf() calls on every tone-on
 * and every frequency-slider drag tick (toneFreqCb re-begins the tone at
 * each new frequency, which used to mean re-running this loop mid-drag). */
static void initSineLut() {
  for (int i = 0; i < 64; i++)
    sineLut[i] = 2048 + (int)(2000.0f * sinf(i * 6.2831853f / 64.0f));
}

static void toneStart() {
  if (dac0.begin(AN_RESOLUTION_12, (uint32_t)toneFreq * 64, 64, 32)) toneOn = true;
}

static void toneStop() {
  toneOn = false;
  dac0.stop();
}

static void toneSwCb(lv_event_t *e) {
  if (lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED)) toneStart();
  else toneStop();
}

static void toneFreqCb(lv_event_t *e) {
  toneFreq = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
  lv_label_set_text_fmt(toneFreqLbl, "%u Hz", toneFreq);
  if (toneOn) { toneStop(); toneStart(); }    /* re-begin at the new rate */
}

/* ---------------- WiFi ---------------- */

/* A successful WiFi.begin() only proves link-layer association + DHCP --
 * it says nothing about whether the internet is actually reachable. Many
 * networks (hotels, airports, coffee shops) intercept all HTTP traffic
 * behind a browser sign-in page ("captive portal") until you accept it
 * there. This device has no browser to show that page, so the best it can
 * do is detect the situation (see checkCaptivePortal()) and tell the user
 * to complete sign-in on another device connected to the same network. */
static volatile uint8_t portalCheckPending = 0;
static volatile bool    wifiPortalSuspected = false;
static volatile bool    wifiConnectFailed = false;

static void updateWifiStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    if (wifiPortalSuspected) {
      lv_label_set_text_fmt(wifiStatusLbl,
          LV_SYMBOL_WARNING "  Connected to %s -- sign-in may be required\n"
          "IP %u.%u.%u.%u  (open a browser on another device to sign in)",
          WiFi.SSID(), ip[0], ip[1], ip[2], ip[3]);
      lv_obj_set_style_text_color(wifiStatusLbl, lv_color_hex(C_WARN), 0);
    } else {
      lv_label_set_text_fmt(wifiStatusLbl,
          LV_SYMBOL_WIFI "  Connected to %s\nIP %u.%u.%u.%u    RSSI %ld dBm",
          WiFi.SSID(), ip[0], ip[1], ip[2], ip[3], (long)WiFi.RSSI());
      lv_obj_set_style_text_color(wifiStatusLbl, lv_color_hex(C_OK), 0);
    }
  } else if (wifiConnectFailed) {
    lv_label_set_text_fmt(wifiStatusLbl,
        LV_SYMBOL_CLOSE "  Couldn't connect to %s\ncheck the password and try again", pendingSsid);
    lv_obj_set_style_text_color(wifiStatusLbl, lv_color_hex(C_DANGER), 0);
  } else {
    lv_label_set_text(wifiStatusLbl, LV_SYMBOL_CLOSE "  Not connected");
    lv_obj_set_style_text_color(wifiStatusLbl, lv_color_hex(C_MUTED), 0);
  }
}

/* Standard captive-portal detection technique (the same one iOS/Android/
 * Windows use): fetch a URL known to return one specific tiny response
 * (HTTP 204, empty body) when reached directly. A captive portal
 * intercepts the request and returns something else instead -- its own
 * login page, a redirect, or nothing at all. Must run from loop(), never
 * an LVGL callback (same rule as every other blocking WiFi call in this
 * file) -- DNS + connect + response can take several seconds. */
static void checkCaptivePortal() {
  DBG("portal: checking");
  wdKick();
  WiFiClient client;
  client.setSocketTimeout(4000);
  bool suspected = true;   /* assume no confirmed internet until proven otherwise */
  if (client.connect("connectivitycheck.gstatic.com", 80)) {
    client.print("GET /generate_204 HTTP/1.1\r\nHost: connectivitycheck.gstatic.com\r\nConnection: close\r\n\r\n");
    char statusLine[48] = {0};
    uint32_t start = millis();
    size_t n = 0;
    while (millis() - start < 4000 && n < sizeof(statusLine) - 1) {
      wdKick();
      if (client.available()) {
        char c = client.read();
        if (c == '\n' || c == '\r') break;
        statusLine[n++] = c;
      }
    }
    statusLine[n] = '\0';
    if (Serial) { Serial.print("portal: response "); Serial.println(statusLine); }
    suspected = (strstr(statusLine, "204") == NULL);
    client.stop();
  } else {
    DBG("portal: check server unreachable");
  }
  wifiPortalSuspected = suspected;
  DBG(suspected ? "portal: suspected (or genuinely offline)" : "portal: clear, internet confirmed");
  updateWifiStatus();
}

/* Blocking WiFi work must NOT run inside an LVGL event callback (touch events
 * dispatch from lv_timer_handler; re-entering the renderer there deadlocks).
 * Callbacks only set a countdown; loop() runs the work a few frames later,
 * after the "Scanning/Connecting..." label has been painted normally. */
static volatile uint8_t scanPending = 0, connectPending = 0;
static volatile uint8_t kbClosePending = 0;
static volatile bool    kbConnectAfterClose = false;

static void requestConnect() {
  wifiConnectFailed = false;
  wifiPortalSuspected = false;
  lv_label_set_text_fmt(wifiStatusLbl, LV_SYMBOL_REFRESH "  Connecting to %s ...", pendingSsid);
  lv_obj_set_style_text_color(wifiStatusLbl, lv_color_hex(C_WARN), 0);
  connectPending = 30;
}

static void performConnect() {
  WiFi.disconnect();
  wdKick();   /* WiFi.begin() with no explicit security re-scans internally
               * before connecting -- can legitimately take several seconds */
  int st = strlen(pendingPass) ? WiFi.begin(pendingSsid, pendingPass)
                               : WiFi.begin(pendingSsid);
  wdKick();
  if (st == WL_CONNECTED) {          /* remember the network for next boot */
    kvSaveStr("/kv/wifi_ssid", pendingSsid);
    kvSaveStr("/kv/wifi_pass", pendingPass);
    DBG("wifi: creds saved");
    portalCheckPending = 15;         /* loop() verifies real internet a moment later */
  } else {
    wifiConnectFailed = true;
    DBG("wifi: connect failed");
  }
  updateWifiStatus();
}

/* The modal (card + textarea + keyboard) is created fresh on each open and
 * deleted on close. Keeping it permanently resident after first use (an
 * earlier version of this code) left only ~4.8KB free indefinitely -- too
 * thin a margin for hours of continuous dashboard/IMU/audio timer churn.
 * A transient spike while the modal is open, returning to the healthy
 * ~11KB idle baseline once closed, is the safer trade-off at this pool size. */
static void closePwModal() {
  if (pwModal) { lv_obj_delete(pwModal); pwModal = NULL; }
  lv_obj_set_style_bg_opa(lv_layer_top(), LV_OPA_TRANSP, 0);
  lv_obj_remove_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);
}

/* kbCb is the keyboard's OWN event handler (LV_EVENT_READY fires when the
 * user taps its checkmark key). Deleting the modal -- or touching pwModal's
 * style at all -- synchronously from within that same widget's own event
 * dispatch reproducibly hard-froze the board (confirmed with a serial test
 * command that injects a synthetic LV_EVENT_READY: the board died within
 * a second of that call, every time, whether closePwModal() used
 * lv_obj_delete() or the "safer" lv_obj_delete_async()). This mirrors the
 * existing "blocking WiFi work must not run inside an LVGL event callback"
 * rule in this file -- the same fix applies: kbCb only captures the typed
 * password and arms a countdown; loop() does the actual close (and the
 * connect) a few frames later, fully outside of any LVGL event dispatch. */
static void kbCb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    strlcpy(pendingPass, lv_textarea_get_text(pwTa), sizeof(pendingPass));
    kbConnectAfterClose = true;
    kbClosePending = 5;
  } else if (code == LV_EVENT_CANCEL) {
    kbConnectAfterClose = false;
    kbClosePending = 5;
  }
}

static void openPwModal() {
  lv_obj_set_style_bg_color(lv_layer_top(), lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(lv_layer_top(), LV_OPA_50, 0);
  lv_obj_add_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);

  pwModal = lv_obj_create(lv_layer_top());
  lv_obj_set_size(pwModal, 560, 380);
  lv_obj_center(pwModal);
  lv_obj_add_style(pwModal, &stCard, 0);
  lv_obj_remove_flag(pwModal, LV_OBJ_FLAG_SCROLLABLE);

  pwTitle = lv_label_create(pwModal);
  lv_label_set_text_fmt(pwTitle, LV_SYMBOL_WIFI "  Password for %s", pendingSsid);
  lv_obj_add_style(pwTitle, &stCardTitle, 0);
  lv_obj_align(pwTitle, LV_ALIGN_TOP_MID, 0, 0);

  pwTa = lv_textarea_create(pwModal);
  lv_textarea_set_one_line(pwTa, true);
  lv_textarea_set_password_mode(pwTa, true);
  lv_textarea_set_placeholder_text(pwTa, "password");
  lv_obj_set_width(pwTa, 400);
  lv_obj_align(pwTa, LV_ALIGN_TOP_MID, 0, 40);

  pwKb = lv_keyboard_create(pwModal);
  lv_obj_set_size(pwKb, 520, 230);
  lv_obj_align(pwKb, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_keyboard_set_textarea(pwKb, pwTa);
  lv_obj_add_event_cb(pwKb, kbCb, LV_EVENT_ALL, NULL);
  DBG("modal: open");
}

/* Manual/hidden-SSID entry ("Other network..." button). Same create-fresh,
 * defer-close-out-of-the-callback pattern as the password modal above --
 * ssidKbCb only captures text and arms a countdown; loop() does the actual
 * close, then chains straight into openPwModal() for the typed SSID so an
 * open (no-password) hidden network still works via the normal blank-
 * password path in performConnect(). */
static volatile uint8_t ssidClosePending = 0;
static volatile bool    ssidSubmitted = false;

static void closeSsidModal() {
  if (ssidModal) { lv_obj_delete(ssidModal); ssidModal = NULL; }
  lv_obj_set_style_bg_opa(lv_layer_top(), LV_OPA_TRANSP, 0);
  lv_obj_remove_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);
}

static void ssidKbCb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    strlcpy(pendingSsid, lv_textarea_get_text(ssidTa), sizeof(pendingSsid));
    ssidSubmitted = true;
    ssidClosePending = 5;
  } else if (code == LV_EVENT_CANCEL) {
    ssidSubmitted = false;
    ssidClosePending = 5;
  }
}

static void openSsidModal() {
  lv_obj_set_style_bg_color(lv_layer_top(), lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(lv_layer_top(), LV_OPA_50, 0);
  lv_obj_add_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);

  ssidModal = lv_obj_create(lv_layer_top());
  lv_obj_set_size(ssidModal, 560, 380);
  lv_obj_center(ssidModal);
  lv_obj_add_style(ssidModal, &stCard, 0);
  lv_obj_remove_flag(ssidModal, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title = lv_label_create(ssidModal);
  lv_label_set_text(title, LV_SYMBOL_WIFI "  Enter network name (SSID)");
  lv_obj_add_style(title, &stCardTitle, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

  ssidTa = lv_textarea_create(ssidModal);
  lv_textarea_set_one_line(ssidTa, true);
  lv_textarea_set_placeholder_text(ssidTa, "network name");
  lv_obj_set_width(ssidTa, 400);
  lv_obj_align(ssidTa, LV_ALIGN_TOP_MID, 0, 40);

  ssidKb = lv_keyboard_create(ssidModal);
  lv_obj_set_size(ssidKb, 520, 230);
  lv_obj_align(ssidKb, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_keyboard_set_textarea(ssidKb, ssidTa);
  lv_obj_add_event_cb(ssidKb, ssidKbCb, LV_EVENT_ALL, NULL);
  DBG("ssid modal: open");
}

static void otherNetworkCb(lv_event_t *e) { (void)e; openSsidModal(); }

/* ---------------- Recon (network reconnaissance / port scan) ----------------
 * For authorized security testing only -- only scan hosts/networks covered
 * by an actual engagement's written authorization.
 *
 * TCP connect-scan against a user-entered target IP across a small fixed
 * list of common ports. The GIGA's WiFi library (WiFi.h/WiFi.cpp, read
 * earlier) only exposes station/AP mode via WiFiClient -- no monitor mode,
 * no raw 802.11 frame access -- so this is deliberately scoped to what a
 * TCP/IP client stack can actually do, not a general pentest suite.
 *
 * Capped at 8 ports (was 16): a real scan against a live host pushed the
 * LVGL heap down to 280 bytes free -- each result row costs real pool
 * space, and 16 of them is roughly double what the WiFi scan list (also
 * capped at 8, and proven stable all session) uses. 8 keeps this at the
 * same, already-validated memory footprint. */
static const uint16_t COMMON_PORTS[] = { 21, 22, 23, 80, 443, 445, 3389, 8080 };
#define NUM_COMMON_PORTS (sizeof(COMMON_PORTS) / sizeof(COMMON_PORTS[0]))

/* Same create-fresh, defer-close-out-of-the-callback pattern as the
 * password/SSID modals: reconKbCb only captures the typed IP and arms a
 * countdown; loop() closes the modal and starts the scan a few frames
 * later, never touching anything from inside the keyboard's own event
 * dispatch (see kbCb's comment for why that reproducibly hard-froze the
 * board earlier this session). */
static volatile uint8_t reconClosePending = 0;
static volatile bool    reconSubmitted = false;

static void closeReconModal() {
  if (reconModal) { lv_obj_delete(reconModal); reconModal = NULL; }
  lv_obj_set_style_bg_opa(lv_layer_top(), LV_OPA_TRANSP, 0);
  lv_obj_remove_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);
}

/* Accepts "<ip>" or "<ip> <port,port,...>" (space-separated) in the single
 * text field -- a second textarea would need the keyboard to switch focus
 * between two fields, a new interaction pattern this session hasn't proven
 * safe yet, so this reuses the existing single-field modal completely
 * unchanged instead. Falls back to the default COMMON_PORTS list when no
 * ports are typed. Always capped at 8 (see COMMON_PORTS' own comment for
 * why that specific number, not a rounder one). */
static void parseReconInput(const char *raw) {
  char buf[80];
  strlcpy(buf, raw, sizeof(buf));
  char *space = strchr(buf, ' ');
  if (space) *space = '\0';
  strlcpy(targetIp, buf, sizeof(targetIp));

  reconPortCount = 0;
  if (space) {
    char *tok = strtok(space + 1, ",");
    while (tok && reconPortCount < 8) {
      long p = strtol(tok, NULL, 10);
      if (p > 0 && p <= 65535) reconPorts[reconPortCount++] = (uint16_t)p;
      tok = strtok(NULL, ",");
    }
  }
  if (reconPortCount == 0) {
    for (size_t i = 0; i < NUM_COMMON_PORTS && i < 8; i++) reconPorts[reconPortCount++] = COMMON_PORTS[i];
  }
}

static void reconKbCb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    parseReconInput(lv_textarea_get_text(reconTa));
    reconSubmitted = true;
    reconClosePending = 5;
  } else if (code == LV_EVENT_CANCEL) {
    reconSubmitted = false;
    reconClosePending = 5;
  }
}

/* Frees the previous scan's result rows (if any) before creating the modal
 * below, not after -- reproducibly starved the modal's own allocation
 * (textarea + keyboard) of contiguous heap on a *second* scan otherwise:
 * a full 8-row result list plus a freshly-created modal don't fit
 * together in this pool at the same time, and the failure doesn't surface
 * where it happens (LVGL v9 defers layout, so it faults on the *next*
 * lv_timer_handler() pass, well after openReconModal() itself returns
 * cleanly) -- confirmed by bisecting against the already-committed v1
 * Recon tab, which has the exact same bug, just never exercised by a
 * second scan in one session before. Old results disappearing the moment
 * the user reopens the modal to type a new target is the acceptable
 * trade-off, not a regression -- they're about to be replaced anyway. */
static void openReconModal() {
  if (reconList) lv_obj_clean(reconList);

  lv_obj_set_style_bg_color(lv_layer_top(), lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(lv_layer_top(), LV_OPA_50, 0);
  lv_obj_add_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);

  reconModal = lv_obj_create(lv_layer_top());
  lv_obj_set_size(reconModal, 560, 380);
  lv_obj_center(reconModal);
  lv_obj_add_style(reconModal, &stCard, 0);
  lv_obj_remove_flag(reconModal, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title = lv_label_create(reconModal);
  lv_label_set_text(title, LV_SYMBOL_GPS "  Target IP, optionally + ports");
  lv_obj_add_style(title, &stCardTitle, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

  reconTa = lv_textarea_create(reconModal);
  lv_textarea_set_one_line(reconTa, true);
  lv_textarea_set_placeholder_text(reconTa, "192.168.1.1 22,80,443");
  lv_obj_set_width(reconTa, 400);
  lv_obj_align(reconTa, LV_ALIGN_TOP_MID, 0, 40);

  reconKb = lv_keyboard_create(reconModal);
  lv_obj_set_size(reconKb, 520, 230);
  lv_obj_align(reconKb, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_keyboard_set_textarea(reconKb, reconTa);
  lv_obj_add_event_cb(reconKb, reconKbCb, LV_EVENT_ALL, NULL);
  DBG("recon modal: open");
}

static void openReconModalCb(lv_event_t *e) { (void)e; openReconModal(); }

/* Blocking (WiFiClient::connect() per port) -- must run from loop(), never
 * an LVGL callback, same rule as every other network call in this file.
 *
 * setSocketTimeout() does NOT bound the connect() phase on this core:
 * reading MbedClient.cpp shows connect(IPAddress, port) creates a raw
 * TCPSocket and calls sock->connect() directly -- sock->set_timeout()
 * is only ever invoked for the SSL variant and for I/O *after* a
 * successful connect. So a filtered/non-responding port's connect() call
 * blocks for whatever the underlying mbed network stack's own default is,
 * not the 400ms this code originally assumed -- confirmed the hard way:
 * the watchdog fired mid-scan during testing. Rather than reach for a raw
 * non-blocking-socket rewrite, the watchdog is deliberately paused for
 * this one bounded, user-initiated operation (TCP connect attempts always
 * eventually give up on their own; this just isn't a hang the watchdog
 * needs to catch) and resumed immediately after. */
/* A banner-grab step (read the open socket's greeting after connect())
 * was tried here and pulled back out: it reproducibly caused the board to
 * hang and watchdog-reset several seconds to minutes *after* the scan had
 * already completed and reported success -- including once with zero
 * further activity in between, ruling out a simple re-entrancy bug in the
 * scan loop itself. Root cause not confirmed (suspected: the mbed WiFi
 * stack's connect()/read()/stop() cycle leaves a pending async operation
 * that later fires into freed state), but the delayed, non-deterministic
 * nature makes it unsafe to ship. If this is revisited, test it in
 * isolation for a long soak (many minutes idle after a scan with an open
 * port) before trusting it again. */

/* Scan history: last 3 scans' target + open/total summary. Deliberately
 * plain C arrays, not LVGL objects -- this data lives in ordinary global
 * RAM, completely separate from the 64KB LVGL pool that's already the
 * tightest resource on this board (4372 bytes free was the observed floor
 * for a single 8-port scan). Displayed via ONE label updated in place
 * (reconHistLbl, created once in buildRecon()), never new per-entry
 * widgets -- that's what capped the result list itself at 8 rows earlier. */
#define RECON_HISTORY_SIZE 3
static char reconHistTarget[RECON_HISTORY_SIZE][40];
static int  reconHistOpen[RECON_HISTORY_SIZE];
static int  reconHistTotal[RECON_HISTORY_SIZE];
static int  reconHistCount = 0;   /* how many slots are populated, <= RECON_HISTORY_SIZE */
static int  reconHistNext  = 0;   /* ring-buffer write position */

static void recordReconHistory(const char *target, int openCount, int total) {
  strlcpy(reconHistTarget[reconHistNext], target, sizeof(reconHistTarget[0]));
  reconHistOpen[reconHistNext] = openCount;
  reconHistTotal[reconHistNext] = total;
  reconHistNext = (reconHistNext + 1) % RECON_HISTORY_SIZE;
  if (reconHistCount < RECON_HISTORY_SIZE) reconHistCount++;
}

static void updateReconHistoryLabel() {
  if (reconHistCount == 0) {
    lv_label_set_text(reconHistLbl, "No scans yet this session");
    return;
  }
  char buf[160];
  size_t pos = 0;
  buf[0] = '\0';
  for (int i = 0; i < reconHistCount; i++) {
    int idx = (reconHistNext - 1 - i + RECON_HISTORY_SIZE) % RECON_HISTORY_SIZE;  /* most recent first */
    int n = snprintf(buf + pos, sizeof(buf) - pos, "%s%s (%d/%d)",
                      i ? "   |   " : "", reconHistTarget[idx], reconHistOpen[idx], reconHistTotal[idx]);
    if (n < 0 || (size_t)n >= sizeof(buf) - pos) break;
    pos += n;
  }
  lv_label_set_text(reconHistLbl, buf);
}

static void performReconScan() {
  DBG("recon: scan begin");
  mbed::Watchdog &watchdog = mbed::Watchdog::get_instance();
  watchdog.stop();

  if (reconList) lv_obj_clean(reconList);
  lv_label_set_text_fmt(reconStatusLbl, LV_SYMBOL_REFRESH "  Scanning %s ...", targetIp);
  lv_obj_set_style_text_color(reconStatusLbl, lv_color_hex(C_WARN), 0);

  IPAddress target;
  if (!target.fromString(targetIp)) {
    lv_label_set_text_fmt(reconStatusLbl, LV_SYMBOL_CLOSE "  Invalid IP: %s", targetIp);
    lv_obj_set_style_text_color(reconStatusLbl, lv_color_hex(C_DANGER), 0);
    DBG("recon: invalid target");
    watchdog.start(wdTimeout);
    return;
  }

  int openCount = 0;
  for (int i = 0; reconList && i < reconPortCount; i++) {
    WiFiClient client;
    bool open = client.connect(target, reconPorts[i]);
    if (open) {
      client.stop();
      openCount++;
    }
    if (Serial) {   /* audit-trail log, not just a debug aid, for a security tool */
      Serial.print("recon: port "); Serial.print(reconPorts[i]);
      Serial.println(open ? " open" : " closed");
    }

    lv_obj_t *row = lv_obj_create(reconList);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, 40);
    lv_obj_set_style_bg_color(row, lv_color_hex(open ? 0x14301F : 0x1B2A4A), 0);
    lv_obj_set_style_radius(row, 8, 0);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *lbl = lv_label_create(row);
    lv_label_set_text_fmt(lbl, "%s   port %-5u  %s",
        open ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE, reconPorts[i], open ? "open" : "closed");
    lv_obj_set_style_text_color(lbl, lv_color_hex(open ? C_OK : C_MUTED), 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 8, 0);
  }

  watchdog.start(wdTimeout);
  wdKick();
  lv_label_set_text_fmt(reconStatusLbl, LV_SYMBOL_OK "  Scan of %s complete -- %d/%d ports open",
      targetIp, openCount, reconPortCount);
  lv_obj_set_style_text_color(reconStatusLbl, lv_color_hex(C_TEXT), 0);
  recordReconHistory(targetIp, openCount, reconPortCount);
  updateReconHistoryLabel();
  DBG("recon: scan end");
}

static void wifiItemCb(lv_event_t *e) {
  int i = (int)(uintptr_t)lv_event_get_user_data(e);
  DBG("item: enter");
  strlcpy(pendingSsid, ssidCache[i], sizeof(pendingSsid));
  pendingPass[0] = '\0';
  bool open = (WiFi.encryptionType(i) == ENC_TYPE_NONE);
  DBG("item: enc read");
  if (open) requestConnect();  /* open network */
  else openPwModal();
  DBG("item: done");
}

static void wifiHint() {
  lv_obj_t *hint = lv_label_create(wifiList);
  lv_label_set_text(hint, "Tap Scan, then pick a network");
  lv_obj_set_style_text_color(hint, lv_color_hex(C_MUTED), 0);
}

static void wifiScanCb(lv_event_t *e) {
  (void)e;
  DBG("cb: enter");
  lv_label_set_text(wifiStatusLbl, LV_SYMBOL_REFRESH "  Scanning ...");
  lv_obj_set_style_text_color(wifiStatusLbl, lv_color_hex(C_WARN), 0);
  DBG("cb: label set");
  if (wifiList) lv_obj_clean(wifiList);
  DBG("cb: list cleaned");
  scanPending = 30;                        /* loop() runs the scan shortly */
}

static void performScan() {
  DBG("scan: begin");
  wdKick();
  if (Serial) { Serial.print("scan: status="); Serial.println(WiFi.status());
                Serial.print("scan: fw=");     Serial.println(WiFi.firmwareVersion()); }
  int n = WiFi.scanNetworks();
  wdKick();
  if (Serial) { Serial.print("scan: found "); Serial.println(n); }
  if (n > 8) n = 8;
  for (int i = 0; wifiList && i < n; i++) {
    strlcpy(ssidCache[i], WiFi.SSID(i), sizeof(ssidCache[i]));
    static char line[64];
    bool open = (WiFi.encryptionType(i) == ENC_TYPE_NONE);
    snprintf(line, sizeof(line), LV_SYMBOL_WIFI "   %s   (%ld dBm)%s",
             ssidCache[i], (long)WiFi.RSSI(i), open ? "  open" : "");
    lv_obj_t *btn = lv_button_create(wifiList);
    lv_obj_set_width(btn, LV_PCT(100));
    lv_obj_set_height(btn, 48);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x1B2A4A), 0);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_add_event_cb(btn, wifiItemCb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, line);
    lv_obj_set_style_text_color(lbl, lv_color_hex(C_TEXT), 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 0, 0);
  }
  updateWifiStatus();
}

static void wifiDisconnectCb(lv_event_t *e) {
  (void)e;
  WiFi.disconnect();
  wifiPortalSuspected = false;
  wifiConnectFailed = false;
  kvRemove("/kv/wifi_ssid");        /* Disconnect also forgets the network */
  kvRemove("/kv/wifi_pass");
  updateWifiStatus();
}

/* ---------------- BLE ---------------- */

static void bleStart() {
  if (!BLE.begin()) {
    lv_label_set_text(bleStatusLbl, LV_SYMBOL_WARNING "  BLE init failed");
    lv_obj_set_style_text_color(bleStatusLbl, lv_color_hex(C_DANGER), 0);
    lv_obj_remove_state(bleSw, LV_STATE_CHECKED);
    bleEnabled = false;
    return;
  }
  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(ctrlService);
  if (!bleReady) {                 /* add the GATT table only once */
    ctrlService.addCharacteristic(relayChar);
    ctrlService.addCharacteristic(motorChar);
    ctrlService.addCharacteristic(sensorChar);
    BLE.addService(ctrlService);
    bleReady = true;
  }
  relayChar.writeValue(0);
  BLE.advertise();
  bleEnabled = true;
}

static void bleStop() {
  BLE.stopAdvertise();
  BLE.end();
  bleEnabled = false;
}

static void bleSwCb(lv_event_t *e) {
  if (lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED)) bleStart();
  else bleStop();
  kvSaveU8("/kv/ble_on", bleEnabled ? 1 : 0);   /* bleStart may have failed */
}

static void blePoll() {
  BLE.poll();
  if (relayChar.written()) {
    uint8_t m = relayChar.value();
    for (int i = 0; i < 4; i++) applyRelay(i, m & (1 << i));
  }
  if (motorChar.written() && motorChar.valueLength() >= 3) {
    const uint8_t *v = motorChar.value();
    int i = v[0] & 1;
    motorSpeed[i] = v[1] > 100 ? 100 : v[1];
    motorFwd[i]   = v[2] != 0;
    motorEn[i]    = motorSpeed[i] > 0;
    lv_slider_set_value(motorSlider[i], motorSpeed[i], LV_ANIM_OFF);
    if (motorFwd[i]) lv_obj_add_state(motorDirSw[i], LV_STATE_CHECKED);
    else             lv_obj_remove_state(motorDirSw[i], LV_STATE_CHECKED);
    if (motorEn[i])  lv_obj_add_state(motorEnSw[i], LV_STATE_CHECKED);
    else             lv_obj_remove_state(motorEnSw[i], LV_STATE_CHECKED);
    applyMotor(i);
  }
}

/* ================= periodic UI timers ================= */

/* Cached so statusTimerCb's BLE stream (a separate, slower timer) reads the
 * same sample the UI just showed instead of taking its own independent
 * analogRead() -- one fewer ADC conversion per tick, and the value a BLE
 * central sees now always matches what's on screen instead of occasionally
 * differing by whatever changed between two split-second-apart reads. */
static int lastA0 = 0;

static void sensorTimerCb(lv_timer_t *t) {
  (void)t;
  for (int i = 0; i < 4; i++) {
    int v = analogRead(ANALOG_PIN[i]);
    lv_bar_set_value(sensorBar[i], v, LV_ANIM_OFF);
    lv_label_set_text_fmt(sensorLbl[i], "%s   %4d   %d.%02d V",
        ANALOG_NAME[i], v, (v * 33) / 10230, ((v * 330) / 1023) % 100);
    if (i == 0) {
      lastA0 = v;
      lv_chart_set_next_value(chart, chartSer, v);
      int pct = (v * 100) / 1023;
      lv_arc_set_value(gaugeArc, pct);
      lv_label_set_text_fmt(gaugeLbl, "%d%%", pct);
    }
  }
}

static void imuTimerCb(lv_timer_t *t) {
  (void)t;
  if (!imuOk) return;
  if (lv_tabview_get_tab_active(tabview) != 4) return;  /* tab 4 = IMU; nothing to update off-tab */
  float x, y, z;
  if (imu.accelerationAvailable() && imu.readAcceleration(x, y, z)) {
    lv_bar_set_value(accBar[0], (int)(x * 1000), LV_ANIM_OFF);
    lv_bar_set_value(accBar[1], (int)(y * 1000), LV_ANIM_OFF);
    lv_bar_set_value(accBar[2], (int)(z * 1000), LV_ANIM_OFF);
    lv_label_set_text_fmt(accLbl, "X %+.2f   Y %+.2f   Z %+.2f  g", x, y, z);
  }
  if (imu.gyroscopeAvailable() && imu.readGyroscope(x, y, z)) {
    lv_bar_set_value(gyroBar[0], (int)x, LV_ANIM_OFF);
    lv_bar_set_value(gyroBar[1], (int)y, LV_ANIM_OFF);
    lv_bar_set_value(gyroBar[2], (int)z, LV_ANIM_OFF);
    lv_label_set_text_fmt(gyroLbl, "X %+.0f   Y %+.0f   Z %+.0f  dps", x, y, z);
  }
}

static void audioTimerCb(lv_timer_t *t) {
  (void)t;
  if (!micOk) return;
  if (lv_tabview_get_tab_active(tabview) != 5) return;  /* tab 5 = Audio; nothing to update off-tab */
  int n = micSamples;
  if (n <= 0) return;
  uint64_t acc = 0;
  for (int i = 0; i < n; i++) acc += (int32_t)micBuf[i] * micBuf[i];
  int rms = (int)sqrtf((float)(acc / n));
  lv_bar_set_value(micBar, rms > 8000 ? 8000 : rms, LV_ANIM_OFF);
  lv_label_set_text_fmt(micLbl, "level %d", rms);
  lv_chart_set_next_value(micChart, micSer, rms > 8000 ? 8000 : rms);
}

static void statusTimerCb(lv_timer_t *t) {
  (void)t;
  if (Serial) {                                   /* serial-link heartbeat */
    lv_mem_monitor_t m; lv_mem_monitor(&m);
    Serial.print("hb "); Serial.print(millis());
    Serial.print(" free "); Serial.print(m.free_size);
    Serial.print(" big "); Serial.print(m.free_biggest_size);
    Serial.print(" frag "); Serial.println(m.frag_pct);
  }

  /* Every label below belongs to exactly one tab -- same "don't update
   * widgets nobody can see" principle already applied to imuTimerCb/
   * audioTimerCb. Each block is skipped unless its own tab is active; the
   * timer still fires every second regardless, so a tab picks back up
   * within at most 1s of switching to it -- no real staleness, just no
   * wasted formatting/style work while off-tab. The BLE characteristic
   * stream at the end is the one exception: a phone can be subscribed
   * regardless of which tab is on screen, so it always runs. */
  uint32_t activeTab = lv_tabview_get_tab_active(tabview);

  if (activeTab == 0) {                            /* dashboard: wifi */
    if (WiFi.status() == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      lv_label_set_text_fmt(dashWifiLbl, "%s\n%u.%u.%u.%u  (%ld dBm)",
          WiFi.SSID(), ip[0], ip[1], ip[2], ip[3], (long)WiFi.RSSI());
      lv_obj_set_style_text_color(dashWifiLbl, lv_color_hex(C_OK), 0);
    } else {
      lv_label_set_text(dashWifiLbl, "Offline\nuse the WiFi tab");
      lv_obj_set_style_text_color(dashWifiLbl, lv_color_hex(C_MUTED), 0);
    }
  }

  /* WiFi tab: catch connection drops (weak signal, router reboot, ...)
   * while the user is just sitting on the tab -- otherwise the label only
   * updates on an explicit scan/connect/disconnect action and can go
   * stale. Skipped while a scan/connect/portal-check/modal-close is in
   * flight so this doesn't clobber a transient "Scanning..." message. */
  if (activeTab == 6 && !scanPending && !connectPending && !portalCheckPending && !kbClosePending)
    updateWifiStatus();

  if (activeTab == 0 || activeTab == 7) {          /* dashboard + BLE tab */
    const char *bleTxt; uint32_t bleCol;
    if (!bleEnabled)                { bleTxt = "Disabled";                 bleCol = C_MUTED; }
    else if (BLE.central())         { bleTxt = "Central connected";        bleCol = C_OK;    }
    else                            { bleTxt = "Advertising as\n" BLE_DEVICE_NAME; bleCol = C_ACCENT; }
    if (activeTab == 0) {
      lv_label_set_text(dashBleLbl, bleTxt);
      lv_obj_set_style_text_color(dashBleLbl, lv_color_hex(bleCol), 0);
    }
    if (activeTab == 7) {
      lv_label_set_text_fmt(bleStatusLbl, LV_SYMBOL_BLUETOOTH "  %s", bleTxt);
      lv_obj_set_style_text_color(bleStatusLbl, lv_color_hex(bleCol), 0);
    }
  }

  if (activeTab == 0) {                            /* uptime */
    uint32_t s = millis() / 1000;
    lv_label_set_text_fmt(dashUpLbl, "Uptime  %luh %02lum %02lus",
        (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
  }

  if (activeTab == 8) {                            /* settings: system info */
    uint32_t s = millis() / 1000;
    IPAddress ip = WiFi.localIP();
    lv_label_set_text_fmt(sysInfoLbl,
        "Uptime      %luh %02lum %02lus\n"
        "WiFi        %s\n"
        "IP          %u.%u.%u.%u\n"
        "IMU         %s\n"
        "Microphone  %s\n"
        "LVGL        %d.%d.%d",
        (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60),
        WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "offline",
        ip[0], ip[1], ip[2], ip[3],
        imuOk ? "OK (BMI270)" : "not found",
        micOk ? "OK (PDM)" : "not found",
        LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
  }

  /* stream A0 to a subscribed BLE central -- reuse sensorTimerCb's sample
   * rather than taking a fresh independent reading (see lastA0 comment).
   * Always runs regardless of active tab -- a phone can be subscribed no
   * matter what's on screen. */
  if (bleEnabled && BLE.central()) sensorChar.writeValue((uint16_t)lastA0);

  /* Stream A0 to MQTT subscribers too (~1 Hz, this timer's own cadence).
   * Same always-runs rationale as the BLE stream: a remote subscriber is
   * independent of which tab is on screen. Publishing from a timer callback
   * (inside lv_timer_handler, inside loop()) is the same safe context the
   * BLE writeValue above already uses -- not an LVGL event dispatch. */
  if (mqttEnabled && mqttClient.connected()) {
    char b[8];
    snprintf(b, sizeof(b), "%d", lastA0);
    mqttClient.publish(MQTT_PREFIX "/sensor/a0", b);
  }
}

/* ================= theme / styles ================= */

static void initTheme() {
  /* full-screen vertical gradient background */
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(C_BG_TOP), 0);
  lv_obj_set_style_bg_grad_color(lv_screen_active(), lv_color_hex(C_BG_BOT), 0);
  lv_obj_set_style_bg_grad_dir(lv_screen_active(), LV_GRAD_DIR_VER, 0);

  lv_style_init(&stCard);
  lv_style_set_bg_color(&stCard, lv_color_hex(C_CARD));
  lv_style_set_bg_opa(&stCard, LV_OPA_COVER);
  lv_style_set_radius(&stCard, 16);
  lv_style_set_border_width(&stCard, 1);
  lv_style_set_border_color(&stCard, lv_color_hex(C_CARD_BRD));
  lv_style_set_shadow_width(&stCard, 22);
  lv_style_set_shadow_color(&stCard, lv_color_hex(0x000000));
  lv_style_set_shadow_opa(&stCard, LV_OPA_40);
  lv_style_set_shadow_offset_y(&stCard, 6);
  lv_style_set_pad_all(&stCard, 14);
  lv_style_set_text_color(&stCard, lv_color_hex(C_TEXT));

  lv_style_init(&stCardTitle);
  lv_style_set_text_color(&stCardTitle, lv_color_hex(C_ACCENT));
  lv_style_set_text_font(&stCardTitle, FONT_CARD);

  lv_style_init(&stMuted);
  lv_style_set_text_color(&stMuted, lv_color_hex(C_MUTED));

  lv_style_init(&stBig);
  lv_style_set_text_color(&stBig, lv_color_hex(C_TEXT));
  lv_style_set_text_font(&stBig, FONT_TITLE);
}

static void styleTab(lv_obj_t *tab) {
  lv_obj_set_style_bg_opa(tab, LV_OPA_TRANSP, 0);
  lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_style_pad_all(tab, 14, 0);
  lv_obj_set_style_pad_row(tab, 14, 0);
  lv_obj_set_style_pad_column(tab, 14, 0);
}

/* ================= tab builders ================= */

static void buildDashboard(lv_obj_t *tab) {
  /* network card */
  lv_obj_t *c = makeCard(tab, 300, 130);
  cardTitle(c, LV_SYMBOL_WIFI, "Network");
  dashWifiLbl = lv_label_create(c);
  lv_obj_set_width(dashWifiLbl, 270);   /* defensive: wrap a long SSID instead of overflowing the tile */
  lv_label_set_long_mode(dashWifiLbl, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_align(dashWifiLbl, LV_ALIGN_LEFT_MID, 0, 12);
  lv_label_set_text(dashWifiLbl, "...");

  /* bluetooth card */
  c = makeCard(tab, 300, 130);
  cardTitle(c, LV_SYMBOL_BLUETOOTH, "Bluetooth");
  dashBleLbl = lv_label_create(c);
  lv_obj_align(dashBleLbl, LV_ALIGN_LEFT_MID, 0, 12);
  lv_label_set_text(dashBleLbl, "Disabled");

  /* A0 gauge card (spans two rows) */
  c = makeCard(tab, 250, 274);
  cardTitle(c, LV_SYMBOL_EYE_OPEN, "A0 level");
  gaugeArc = lv_arc_create(c);
  lv_obj_set_size(gaugeArc, 190, 190);
  lv_obj_align(gaugeArc, LV_ALIGN_BOTTOM_MID, 0, -6);
  lv_arc_set_rotation(gaugeArc, 135);
  lv_arc_set_bg_angles(gaugeArc, 0, 270);
  lv_arc_set_range(gaugeArc, 0, 100);
  lv_arc_set_value(gaugeArc, 0);
  lv_obj_remove_style(gaugeArc, NULL, LV_PART_KNOB);      /* read-only gauge */
  lv_obj_remove_flag(gaugeArc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(gaugeArc, 14, LV_PART_MAIN);
  lv_obj_set_style_arc_width(gaugeArc, 14, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(gaugeArc, lv_color_hex(C_DOT_OFF), LV_PART_MAIN);
  lv_obj_set_style_arc_color(gaugeArc, lv_color_hex(C_ACCENT), LV_PART_INDICATOR);
  gaugeLbl = lv_label_create(gaugeArc);
  lv_obj_add_style(gaugeLbl, &stBig, 0);
  lv_label_set_text(gaugeLbl, "0%");
  lv_obj_center(gaugeLbl);

  /* relays quick-view card */
  c = makeCard(tab, 300, 130);
  cardTitle(c, LV_SYMBOL_POWER, "Relays");
  for (int i = 0; i < 4; i++) {
    dashDot[i] = makeDot(c, 26);
    lv_obj_align(dashDot[i], LV_ALIGN_LEFT_MID, i * 60, 6);
    lv_obj_t *n = lv_label_create(c);
    lv_label_set_text_fmt(n, "%d", i + 1);
    lv_obj_add_style(n, &stMuted, 0);
    lv_obj_align(n, LV_ALIGN_LEFT_MID, i * 60 + 8, 34);
  }

  /* system card: uptime + ALL OFF */
  c = makeCard(tab, 300, 130);
  cardTitle(c, LV_SYMBOL_SETTINGS, "System");
  dashUpLbl = lv_label_create(c);
  lv_obj_add_style(dashUpLbl, &stMuted, 0);
  lv_obj_align(dashUpLbl, LV_ALIGN_TOP_LEFT, 0, 26);
  lv_label_set_text(dashUpLbl, "Uptime 0s");

  lv_obj_t *btn = lv_button_create(c);
  lv_obj_set_size(btn, 100, 44);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(C_DANGER), 0);
  lv_obj_set_style_radius(btn, 10, 0);
  lv_obj_add_event_cb(btn, allOffCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *bt = lv_label_create(btn);
  lv_label_set_text(bt, "ALL OFF");
  lv_obj_center(bt);
}

static void buildRelays(lv_obj_t *tab) {
  for (int i = 0; i < 4; i++) {
    lv_obj_t *c = makeCard(tab, 314, 128);
    cardTitle(c, LV_SYMBOL_POWER, RELAY_NAME[i]);
    lv_obj_t *pin = lv_label_create(c);
    lv_label_set_text_fmt(pin, "shield pin D%d", RELAY_PIN[i]);
    lv_obj_add_style(pin, &stMuted, 0);
    lv_obj_align(pin, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    relayDot[i] = makeDot(c, 30);
    lv_obj_align(relayDot[i], LV_ALIGN_RIGHT_MID, -100, 4);

    relaySw[i] = lv_switch_create(c);
    lv_obj_set_size(relaySw[i], 72, 36);
    lv_obj_align(relaySw[i], LV_ALIGN_RIGHT_MID, 0, 4);
    lv_obj_set_style_bg_color(relaySw[i], lv_color_hex(C_OK), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(relaySw[i], relaySwCb, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)i);
  }
}

static void buildMotors(lv_obj_t *tab) {
  for (int i = 0; i < 2; i++) {
    lv_obj_t *c = makeCard(tab, 642, 128);
    cardTitle(c, LV_SYMBOL_CHARGE, MOTOR[i].name);
    lv_obj_t *pin = lv_label_create(c);
    lv_label_set_text_fmt(pin, "PWM D%d   DIR D%d", MOTOR[i].pwmPin, MOTOR[i].dirPin);
    lv_obj_add_style(pin, &stMuted, 0);
    lv_obj_align(pin, LV_ALIGN_TOP_RIGHT, 0, 0);

    motorSlider[i] = lv_slider_create(c);
    lv_slider_set_range(motorSlider[i], 0, 100);
    lv_obj_set_size(motorSlider[i], 330, 14);
    lv_obj_align(motorSlider[i], LV_ALIGN_LEFT_MID, 4, 18);
    lv_obj_set_style_bg_color(motorSlider[i], lv_color_hex(C_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(motorSlider[i], lv_color_hex(C_ACCENT), LV_PART_KNOB);
    lv_obj_add_event_cb(motorSlider[i], motorSliderCb, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)i);

    motorValLbl[i] = lv_label_create(c);
    lv_obj_align(motorValLbl[i], LV_ALIGN_LEFT_MID, 360, 18);
    lv_label_set_text(motorValLbl[i], "0%  FWD  OFF");
    lv_obj_add_style(motorValLbl[i], &stMuted, 0);

    lv_obj_t *dl = lv_label_create(c);
    lv_label_set_text(dl, "REV / FWD");
    lv_obj_add_style(dl, &stMuted, 0);
    lv_obj_align(dl, LV_ALIGN_RIGHT_MID, -175, 18);
    motorDirSw[i] = lv_switch_create(c);
    lv_obj_align(motorDirSw[i], LV_ALIGN_RIGHT_MID, -110, 18);
    lv_obj_add_state(motorDirSw[i], LV_STATE_CHECKED);   /* default FWD */
    lv_obj_add_event_cb(motorDirSw[i], motorDirCb, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)i);

    lv_obj_t *el = lv_label_create(c);
    lv_label_set_text(el, "Enable");
    lv_obj_add_style(el, &stMuted, 0);
    lv_obj_align(el, LV_ALIGN_RIGHT_MID, -56, 18);
    motorEnSw[i] = lv_switch_create(c);
    lv_obj_align(motorEnSw[i], LV_ALIGN_RIGHT_MID, 0, 18);
    lv_obj_set_style_bg_color(motorEnSw[i], lv_color_hex(C_OK), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(motorEnSw[i], motorEnCb, LV_EVENT_VALUE_CHANGED, (void *)(uintptr_t)i);
  }

  lv_obj_t *note = lv_label_create(tab);
  lv_label_set_text(note, "Wire PWM to the H-bridge enable/speed input and DIR to its direction input.\n"
                          "Speed and direction are also writable over BLE (see Bluetooth tab).");
  lv_obj_add_style(note, &stMuted, 0);
}

static void buildSensors(lv_obj_t *tab) {
  /* live chart for A0 */
  lv_obj_t *c = makeCard(tab, 642, 220);
  cardTitle(c, LV_SYMBOL_EYE_OPEN, "A0 live (raw 0-1023)");
  chart = lv_chart_create(c);
  lv_obj_set_size(chart, 600, 160);
  lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart, 60);
  lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1023);
  lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_div_line_count(chart, 4, 8);
  lv_obj_set_style_bg_color(chart, lv_color_hex(0x0E1730), 0);
  lv_obj_set_style_border_width(chart, 0, 0);
  lv_obj_set_style_line_color(chart, lv_color_hex(C_CARD_BRD), LV_PART_MAIN);
  lv_obj_set_style_width(chart, 0, LV_PART_INDICATOR);   /* hide point dots */
  lv_obj_set_style_height(chart, 0, LV_PART_INDICATOR);
  chartSer = lv_chart_add_series(chart, lv_color_hex(C_ACCENT), LV_CHART_AXIS_PRIMARY_Y);

  /* A0..A3 bar cards */
  for (int i = 0; i < 4; i++) {
    lv_obj_t *bc = makeCard(tab, 314, 92);
    sensorLbl[i] = lv_label_create(bc);
    lv_label_set_text_fmt(sensorLbl[i], "%s   ----", ANALOG_NAME[i]);
    lv_obj_align(sensorLbl[i], LV_ALIGN_TOP_LEFT, 0, 0);
    sensorBar[i] = lv_bar_create(bc);
    lv_bar_set_range(sensorBar[i], 0, 1023);
    lv_obj_set_size(sensorBar[i], 280, 14);
    lv_obj_align(sensorBar[i], LV_ALIGN_BOTTOM_LEFT, 0, -4);
    lv_obj_set_style_bg_color(sensorBar[i], lv_color_hex(0x0E1730), LV_PART_MAIN);
    lv_obj_set_style_bg_color(sensorBar[i], lv_color_hex(C_ACCENT), LV_PART_INDICATOR);
  }
}

static void buildImu(lv_obj_t *tab) {
  lv_obj_t *c = makeCard(tab, 314, 250);
  cardTitle(c, LV_SYMBOL_LOOP, "Accelerometer");
  accLbl = lv_label_create(c);
  lv_obj_add_style(accLbl, &stMuted, 0);
  lv_obj_align(accLbl, LV_ALIGN_TOP_LEFT, 0, 28);
  lv_label_set_text(accLbl, "--");
  static const char *axes[3] = { "X", "Y", "Z" };
  for (int i = 0; i < 3; i++) {
    lv_obj_t *al = lv_label_create(c);
    lv_label_set_text(al, axes[i]);
    lv_obj_add_style(al, &stMuted, 0);
    lv_obj_align(al, LV_ALIGN_TOP_LEFT, 0, 62 + i * 40);
    accBar[i] = makeSignedBar(c, -2000, 2000, 66 + i * 40);
    lv_obj_align(accBar[i], LV_ALIGN_TOP_LEFT, 20, 66 + i * 40);
  }

  c = makeCard(tab, 314, 250);
  cardTitle(c, LV_SYMBOL_REFRESH, "Gyroscope");
  gyroLbl = lv_label_create(c);
  lv_obj_add_style(gyroLbl, &stMuted, 0);
  lv_obj_align(gyroLbl, LV_ALIGN_TOP_LEFT, 0, 28);
  lv_label_set_text(gyroLbl, "--");
  for (int i = 0; i < 3; i++) {
    lv_obj_t *gl = lv_label_create(c);
    lv_label_set_text(gl, axes[i]);
    lv_obj_add_style(gl, &stMuted, 0);
    lv_obj_align(gl, LV_ALIGN_TOP_LEFT, 0, 62 + i * 40);
    gyroBar[i] = makeSignedBar(c, -500, 500, 66 + i * 40);
    lv_obj_align(gyroBar[i], LV_ALIGN_TOP_LEFT, 20, 66 + i * 40);
  }

  lv_obj_t *info = makeCard(tab, 642, 74);
  imuStatusLbl = lv_label_create(info);
  lv_obj_align(imuStatusLbl, LV_ALIGN_LEFT_MID, 0, 0);
  lv_label_set_text(imuStatusLbl, "BMI270 6-axis IMU on the Display Shield (I2C on Wire1).");
  lv_obj_add_style(imuStatusLbl, &stMuted, 0);
}

static void buildAudio(lv_obj_t *tab) {
  /* microphone card */
  lv_obj_t *c = makeCard(tab, 642, 210);
  cardTitle(c, LV_SYMBOL_AUDIO, "Microphone (Display Shield PDM)");
  micLbl = lv_label_create(c);
  lv_obj_add_style(micLbl, &stMuted, 0);
  lv_obj_align(micLbl, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_label_set_text(micLbl, "level --");
  micBar = lv_bar_create(c);
  lv_bar_set_range(micBar, 0, 8000);
  lv_obj_set_size(micBar, 600, 16);
  lv_obj_align(micBar, LV_ALIGN_TOP_LEFT, 0, 32);
  lv_obj_set_style_bg_color(micBar, lv_color_hex(0x0E1730), LV_PART_MAIN);
  lv_obj_set_style_bg_color(micBar, lv_color_hex(C_OK), LV_PART_INDICATOR);
  micChart = lv_chart_create(c);
  lv_obj_set_size(micChart, 600, 110);
  lv_obj_align(micChart, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_chart_set_type(micChart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(micChart, 80);
  lv_chart_set_axis_range(micChart, LV_CHART_AXIS_PRIMARY_Y, 0, 8000);
  lv_chart_set_update_mode(micChart, LV_CHART_UPDATE_MODE_SHIFT);
  lv_obj_set_style_bg_color(micChart, lv_color_hex(0x0E1730), 0);
  lv_obj_set_style_border_width(micChart, 0, 0);
  lv_obj_set_style_width(micChart, 0, LV_PART_INDICATOR);
  lv_obj_set_style_height(micChart, 0, LV_PART_INDICATOR);
  micSer = lv_chart_add_series(micChart, lv_color_hex(C_OK), LV_CHART_AXIS_PRIMARY_Y);

  /* speaker card */
  c = makeCard(tab, 642, 150);
  cardTitle(c, LV_SYMBOL_VOLUME_MAX, "Speaker / audio jack tone (DAC0)");
  lv_obj_t *fs = lv_slider_create(c);
  lv_slider_set_range(fs, 100, 2000);
  lv_slider_set_value(fs, toneFreq, LV_ANIM_OFF);
  lv_obj_set_size(fs, 380, 14);
  lv_obj_align(fs, LV_ALIGN_LEFT_MID, 4, 14);
  lv_obj_set_style_bg_color(fs, lv_color_hex(C_ACCENT), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(fs, lv_color_hex(C_ACCENT), LV_PART_KNOB);
  lv_obj_add_event_cb(fs, toneFreqCb, LV_EVENT_VALUE_CHANGED, NULL);
  toneFreqLbl = lv_label_create(c);
  lv_obj_align(toneFreqLbl, LV_ALIGN_LEFT_MID, 420, 14);
  lv_label_set_text_fmt(toneFreqLbl, "%u Hz", toneFreq);
  lv_obj_t *pl = lv_label_create(c);
  lv_label_set_text(pl, "Play");
  lv_obj_add_style(pl, &stMuted, 0);
  lv_obj_align(pl, LV_ALIGN_RIGHT_MID, -90, 14);
  lv_obj_t *tsw = lv_switch_create(c);
  lv_obj_align(tsw, LV_ALIGN_RIGHT_MID, 0, 14);
  lv_obj_set_style_bg_color(tsw, lv_color_hex(C_OK), LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(tsw, toneSwCb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_t *note = lv_label_create(c);
  lv_label_set_text(note, "Sine wave out the 3.5 mm jack — plug in headphones or a powered speaker.");
  lv_obj_add_style(note, &stMuted, 0);
  lv_obj_align(note, LV_ALIGN_BOTTOM_LEFT, 0, 0);
}

static void buildWifi(lv_obj_t *tab) {
  lv_obj_t *c = makeCard(tab, 642, 118);   /* +22px over the original 96 -- headroom for a
                                             * wrapped 3rd status line (long SSID, portal msg) */
  cardTitle(c, LV_SYMBOL_WIFI, "Wireless network");
  wifiStatusLbl = lv_label_create(c);
  lv_obj_set_width(wifiStatusLbl, 610);              /* defensive: wrap instead of overflow the
                                                       * card on a long SSID or status message */
  lv_label_set_long_mode(wifiStatusLbl, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_align(wifiStatusLbl, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_label_set_text(wifiStatusLbl, "Not connected");
  lv_obj_add_style(wifiStatusLbl, &stMuted, 0);

  scanBtn = lv_button_create(c);
  lv_obj_set_size(scanBtn, 110, 40);
  lv_obj_align(scanBtn, LV_ALIGN_RIGHT_MID, 0, 6);
  lv_obj_set_style_bg_color(scanBtn, lv_color_hex(C_ACCENT), 0);
  lv_obj_add_event_cb(scanBtn, wifiScanCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *sl = lv_label_create(scanBtn);
  lv_label_set_text(sl, LV_SYMBOL_REFRESH " Scan");
  lv_obj_center(sl);

  lv_obj_t *dcBtn = lv_button_create(c);
  lv_obj_set_size(dcBtn, 130, 40);
  lv_obj_align(dcBtn, LV_ALIGN_RIGHT_MID, -120, 6);
  lv_obj_set_style_bg_color(dcBtn, lv_color_hex(0x334155), 0);
  lv_obj_add_event_cb(dcBtn, wifiDisconnectCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *dl = lv_label_create(dcBtn);
  lv_label_set_text(dl, LV_SYMBOL_CLOSE " Disconnect");
  lv_obj_center(dl);

  lv_obj_t *otherBtn = lv_button_create(c);
  lv_obj_set_size(otherBtn, 150, 40);
  lv_obj_align(otherBtn, LV_ALIGN_RIGHT_MID, -260, 6);
  lv_obj_set_style_bg_color(otherBtn, lv_color_hex(0x334155), 0);
  lv_obj_add_event_cb(otherBtn, otherNetworkCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *ol = lv_label_create(otherBtn);
  lv_label_set_text(ol, LV_SYMBOL_PLUS " Other...");
  lv_obj_center(ol);

  /* NOTE: lv_list is NOT used here on purpose — rendering lv_list items
   * (both lv_list_add_text and lv_list_add_button) hard-freezes the GIGA
   * with this core's LVGL 9 display driver. A plain flex column of regular
   * buttons renders fine and does the same job. */
  wifiList = lv_obj_create(tab);
  lv_obj_set_size(wifiList, 642, 300);
  lv_obj_set_style_bg_color(wifiList, lv_color_hex(C_CARD), 0);
  lv_obj_set_style_border_color(wifiList, lv_color_hex(C_CARD_BRD), 0);
  lv_obj_set_style_radius(wifiList, 16, 0);
  lv_obj_set_flex_flow(wifiList, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(wifiList, 6, 0);
  wifiHint();
}

static void buildBluetooth(lv_obj_t *tab) {
  lv_obj_t *c = makeCard(tab, 642, 96);
  cardTitle(c, LV_SYMBOL_BLUETOOTH, "BLE peripheral");
  bleStatusLbl = lv_label_create(c);
  lv_obj_align(bleStatusLbl, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_label_set_text(bleStatusLbl, LV_SYMBOL_BLUETOOTH "  Disabled");
  lv_obj_add_style(bleStatusLbl, &stMuted, 0);

  bleSw = lv_switch_create(c);
  lv_obj_set_size(bleSw, 72, 36);
  lv_obj_align(bleSw, LV_ALIGN_RIGHT_MID, 0, 6);
  lv_obj_set_style_bg_color(bleSw, lv_color_hex(C_ACCENT), LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(bleSw, bleSwCb, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t *info = makeCard(tab, 642, 250);
  cardTitle(info, LV_SYMBOL_LIST, "How to control from a phone");
  lv_obj_t *txt = lv_label_create(info);
  lv_obj_set_width(txt, 600);
  lv_obj_align(txt, LV_ALIGN_TOP_LEFT, 0, 30);
  lv_label_set_text(txt,
    "1. Enable BLE above, then open a BLE app (nRF Connect, LightBlue).\n"
    "2. Connect to \"" BLE_DEVICE_NAME "\", service 19B10000-....\n\n"
    "Characteristics:\n"
    "  19B10001  Relays   1 byte  R/W   bit0..bit3 = relay 1..4\n"
    "  19B10002  Motors   3 bytes W     [motor 0/1, speed 0-100, dir 0/1]\n"
    "  19B10003  Sensor   2 bytes R/N   raw A0 reading (notify ~1 Hz)\n\n"
    "Tip: WiFi and BLE share one radio module. If either misbehaves while\n"
    "both are active, use one at a time.");
  lv_obj_add_style(txt, &stMuted, 0);
}

static void buildSettings(lv_obj_t *tab) {
  /* display card */
  lv_obj_t *c = makeCard(tab, 314, 150);
  cardTitle(c, LV_SYMBOL_IMAGE, "Display");
  lv_obj_t *bl = lv_label_create(c);
  lv_label_set_text(bl, "Brightness");
  lv_obj_add_style(bl, &stMuted, 0);
  lv_obj_align(bl, LV_ALIGN_TOP_LEFT, 0, 34);
  lv_obj_t *bs = lv_slider_create(c);
  lv_slider_set_range(bs, 10, 100);
  lv_slider_set_value(bs, kvLoadU8("/kv/bright", 85), LV_ANIM_OFF);
  lv_obj_set_size(bs, 260, 12);
  lv_obj_align(bs, LV_ALIGN_TOP_LEFT, 4, 64);
  lv_obj_add_event_cb(bs, brightnessCb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(bs, brightnessSaveCb, LV_EVENT_RELEASED, NULL);

  /* sensors card */
  c = makeCard(tab, 314, 150);
  cardTitle(c, LV_SYMBOL_EYE_OPEN, "Sensor update rate");
  lv_obj_t *dd = lv_dropdown_create(c);
  lv_dropdown_set_options(dd, "Fast (100 ms)\nNormal (300 ms)\nSlow (1 s)");
  lv_dropdown_set_selected(dd, kvLoadU8("/kv/rate", 1) < 3 ? kvLoadU8("/kv/rate", 1) : 1);
  lv_obj_set_width(dd, 240);
  lv_obj_align(dd, LV_ALIGN_TOP_LEFT, 0, 40);
  lv_obj_add_event_cb(dd, sensorRateCb, LV_EVENT_VALUE_CHANGED, NULL);

  /* system info card */
  c = makeCard(tab, 314, 260);
  cardTitle(c, LV_SYMBOL_LIST, "System");
  sysInfoLbl = lv_label_create(c);
  lv_obj_set_width(sysInfoLbl, 286);   /* defensive: wrap a long SSID on the WiFi line instead
                                        * of overflowing the card */
  lv_label_set_long_mode(sysInfoLbl, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_add_style(sysInfoLbl, &stMuted, 0);
  lv_obj_align(sysInfoLbl, LV_ALIGN_TOP_LEFT, 0, 30);
  lv_label_set_text(sysInfoLbl, "...");

  /* safety card */
  c = makeCard(tab, 314, 260);
  cardTitle(c, LV_SYMBOL_WARNING, "Safety");
  lv_obj_t *note = lv_label_create(c);
  lv_label_set_text(note, "Kill all outputs:\nrelays off, motors stopped.");
  lv_obj_add_style(note, &stMuted, 0);
  lv_obj_align(note, LV_ALIGN_TOP_LEFT, 0, 34);
  lv_obj_t *btn = lv_button_create(c);
  lv_obj_set_size(btn, 160, 56);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_style_bg_color(btn, lv_color_hex(C_DANGER), 0);
  lv_obj_set_style_radius(btn, 12, 0);
  lv_obj_add_event_cb(btn, allOffCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *bt = lv_label_create(btn);
  lv_label_set_text(bt, "ALL OFF");
  lv_obj_center(bt);
}

static void buildRecon(lv_obj_t *tab) {
  lv_obj_t *c = makeCard(tab, 642, 96);
  cardTitle(c, LV_SYMBOL_GPS, "Network reconnaissance (TCP port scan)");
  reconStatusLbl = lv_label_create(c);
  lv_obj_set_width(reconStatusLbl, 460);
  lv_label_set_long_mode(reconStatusLbl, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_align(reconStatusLbl, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_label_set_text(reconStatusLbl, "Enter a target IP to scan common ports");
  lv_obj_add_style(reconStatusLbl, &stMuted, 0);

  lv_obj_t *scanBtn2 = lv_button_create(c);
  lv_obj_set_size(scanBtn2, 190, 40);
  lv_obj_align(scanBtn2, LV_ALIGN_RIGHT_MID, 0, 6);
  lv_obj_set_style_bg_color(scanBtn2, lv_color_hex(C_ACCENT), 0);
  lv_obj_add_event_cb(scanBtn2, openReconModalCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *sl = lv_label_create(scanBtn2);
  lv_label_set_text(sl, LV_SYMBOL_GPS " Set target & scan");
  lv_obj_center(sl);

  /* Same plain flex-column + lv_button pattern as wifiList -- never
   * lv_list, confirmed multiple times this session to hard-freeze this
   * LVGL/display-driver combination. */
  reconList = lv_obj_create(tab);
  lv_obj_set_size(reconList, 642, 260);
  lv_obj_set_style_bg_color(reconList, lv_color_hex(C_CARD), 0);
  lv_obj_set_style_border_color(reconList, lv_color_hex(C_CARD_BRD), 0);
  lv_obj_set_style_radius(reconList, 16, 0);
  lv_obj_set_flex_flow(reconList, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(reconList, 6, 0);

  /* Created once here, updated in place after every scan -- never a new
   * widget per history entry, so this doesn't touch the LVGL pool the same
   * way the (memory-expensive) result rows above do. */
  lv_obj_t *histCard = makeCard(tab, 642, 60);
  reconHistLbl = lv_label_create(histCard);
  lv_obj_set_width(reconHistLbl, 610);
  lv_label_set_long_mode(reconHistLbl, LV_LABEL_LONG_MODE_WRAP);
  lv_label_set_text(reconHistLbl, "No scans yet this session");
  lv_obj_add_style(reconHistLbl, &stMuted, 0);

  lv_obj_t *note = lv_label_create(tab);
  lv_label_set_text(note,
    "TCP connect-scan across common ports (FTP/SSH/Telnet/HTTP/HTTPS/SMB/RDP/\n"
    "HTTP-alt) or a custom comma-separated list you type after the IP\n"
    "(e.g. \"192.168.1.1 22,80,8080\"), capped at 8. For authorized security\n"
    "testing only -- only scan hosts/networks you have explicit permission to test.");
  lv_obj_add_style(note, &stMuted, 0);
}

/* ================= MQTT (remote monitor + control) ================= */

static void mqttSetStatus(const char *txt, uint32_t color) {
  if (!mqttStatusLbl) return;
  lv_label_set_text(mqttStatusLbl, txt);
  lv_obj_set_style_text_color(mqttStatusLbl, lv_color_hex(color), 0);
}

static void mqttSetActivity(const char *txt) {
  if (mqttActivityLbl) lv_label_set_text(mqttActivityLbl, txt);
}

static void mqttUpdateBrokerLbl() {
  if (!mqttBrokerLbl) return;
  if (mqttHost[0]) lv_label_set_text_fmt(mqttBrokerLbl, LV_SYMBOL_UPLOAD "  Broker: %s:%u", mqttHost, mqttPort);
  else             lv_label_set_text(mqttBrokerLbl, LV_SYMBOL_UPLOAD "  No broker set -- tap \"Set broker\"");
}

/* Publish the 4 relays as one retained 0..15 bitmask -- same encoding as
 * the BLE relay characteristic, so a remote controller speaks one language
 * over either transport. Retained so a subscriber that connects later
 * immediately learns the current state. */
static void mqttPublishRelays() {
  char buf[4];
  int m = 0;
  for (int i = 0; i < 4; i++) if (relayState[i]) m |= (1 << i);
  snprintf(buf, sizeof(buf), "%d", m);
  mqttClient.publish(MQTT_PREFIX "/relays", buf, true, 0);
}

/* Report OTA progress both to the remote (MQTT) and the on-screen activity
 * line, so an update is visible from either side. */
static void otaPublish(const char *s) {
  if (mqttClient.connected()) mqttClient.publish(MQTT_PREFIX "/ota/status", s, false, 0);
  mqttSetActivity(s);
}

/* Run a WiFi firmware update. Called from loop() when an OTA was requested
 * over MQTT -- never from the callback, since the download blocks for seconds
 * (the UI freezes; the OTA library feeds the watchdog via wdKick throughout).
 * On success it does not return: the board resets and the bootloader flashes
 * the staged image. Any early return leaves the running firmware untouched. */
static void runOta() {
  otaRequested = false;
  String url = otaUrl;
  if (url.length() == 0) { otaPublish("ota: no url given"); return; }
  bool is_https = url.startsWith("https:");

  otaPublish("ota: starting");
  wdKick();

  Arduino_Portenta_OTA_QSPI ota(QSPI_FLASH_FATFS_MBR, 2);
  if (!ota.isOtaCapable()) { otaPublish("ota: bootloader too old -- update it over USB once"); return; }
  ota.setFeedWatchdogFunc(wdKick);

  if (ota.begin() != Arduino_Portenta_OTA::Error::None) { otaPublish("ota: storage init failed"); return; }

  otaPublish("ota: downloading");
  int const dl = ota.downloadAndDecompress(url.c_str(), is_https);
  if (dl <= 0) {
    char b[40];
    snprintf(b, sizeof b, "ota: download failed (%d)", dl);
    otaPublish(b);
    return;
  }

  if (ota.update() != Arduino_Portenta_OTA::Error::None) { otaPublish("ota: staging failed"); return; }

  otaPublish("ota: applying, rebooting");
  delay(600);   /* let the MQTT publish flush before the reset */
  ota.reset();  /* no return -- bootloader takes over on reboot */
}

/* Incoming command handler. Runs from mqttClient.loop(), which we call from
 * our own loop() -- so this is NOT inside an LVGL event dispatch, and
 * touching widgets here is safe, exactly like blePoll() updating switches
 * from loop() context (the file's established rule). */
static void mqttOnMessage(String &topic, String &payload) {
  if (topic == MQTT_PREFIX "/relay/set") {
    int m = payload.toInt();                 /* 0..15 bitmask, mirrors relayChar */
    for (int i = 0; i < 4; i++) applyRelay(i, m & (1 << i));   /* applyRelay syncs dots+switches */
    if (bleReady) relayChar.writeValue((relayState[0] << 0) | (relayState[1] << 1) |
                                       (relayState[2] << 2) | (relayState[3] << 3));
    mqttSetActivity(LV_SYMBOL_DOWNLOAD "  rx relay/set");
    /* applyRelay set mqttRelaysDirty; loop() will echo the new state back. */
  } else if (topic == MQTT_PREFIX "/ota/update") {
    otaUrl = payload;             /* full .ota URL from the publisher (jessy) */
    otaRequested = true;          /* loop() runs runOta() -- not here */
    mqttSetActivity(LV_SYMBOL_DOWNLOAD "  rx ota/update");
  }
}

/* Blocking on an unreachable broker (the WiFiClient TCP connect phase, not
 * bounded by mqttClient.setTimeout() -- same core limitation the Recon scan
 * documents), so the watchdog is paused around the connect exactly like
 * performReconScan(). Always run from loop() via mqttConnectPending, never
 * from an LVGL callback. */
static void performMqttConnect() {
  if (mqttHost[0] == '\0') {
    mqttSetStatus(LV_SYMBOL_WARNING "  No broker set", C_WARN);
    mqttEnabled = false;
    if (mqttSw) lv_obj_remove_state(mqttSw, LV_STATE_CHECKED);
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    mqttSetStatus(LV_SYMBOL_WARNING "  WiFi offline -- connect WiFi first", C_WARN);
    mqttEnabled = false;
    if (mqttSw) lv_obj_remove_state(mqttSw, LV_STATE_CHECKED);
    return;
  }

  if (mqttStatusLbl) {   /* NULL whenever the (lazy) MQTT tab isn't currently on screen */
    lv_label_set_text_fmt(mqttStatusLbl, LV_SYMBOL_REFRESH "  Connecting to %s:%u ...", mqttHost, mqttPort);
    lv_obj_set_style_text_color(mqttStatusLbl, lv_color_hex(C_WARN), 0);
  }
  DBG("mqtt: connecting");

  mbed::Watchdog &watchdog = mbed::Watchdog::get_instance();
  watchdog.stop();

  mqttClient.begin(mqttHost, mqttPort, mqttNet);
  mqttClient.setOptions(30 /*keepAlive s*/, true /*cleanSession*/, 3000 /*timeout ms*/);
  mqttClient.onMessage(mqttOnMessage);
  mqttClient.setWill(MQTT_PREFIX "/status", "offline", true, 0);   /* LWT: broker announces our drop */

  char user[64] = "", pass[64] = "";
  bool haveUser = kvLoadStr("/kv/mqtt_user", user, sizeof(user));
  bool havePass = kvLoadStr("/kv/mqtt_pass", pass, sizeof(pass));
  bool ok = haveUser ? (havePass ? mqttClient.connect(MQTT_CLIENTID, user, pass)
                                  : mqttClient.connect(MQTT_CLIENTID, user))
                     : mqttClient.connect(MQTT_CLIENTID);

  watchdog.start(wdTimeout);
  wdKick();

  if (ok) {
    mqttClient.publish(MQTT_PREFIX "/status", "online", true, 0);
    mqttClient.subscribe(MQTT_PREFIX "/relay/set");
    mqttClient.subscribe(MQTT_PREFIX "/ota/update");
    mqttPublishRelays();
    if (mqttStatusLbl) {
      lv_label_set_text_fmt(mqttStatusLbl, LV_SYMBOL_OK "  Connected to %s:%u", mqttHost, mqttPort);
      lv_obj_set_style_text_color(mqttStatusLbl, lv_color_hex(C_OK), 0);
    }
    if (mqttSw) lv_obj_add_state(mqttSw, LV_STATE_CHECKED);
    mqttSetActivity("subscribed " MQTT_PREFIX "/relay/set");
    DBG("mqtt: connected");
  } else {
    mqttSetStatus(LV_SYMBOL_CLOSE "  Connect failed -- check broker/WiFi", C_DANGER);
    if (mqttSw) lv_obj_remove_state(mqttSw, LV_STATE_CHECKED);
    DBG("mqtt: connect failed");
    /* leave mqttEnabled as-is: if the user asked to be connected, loop()
     * will retry on a slow cadence; the switch reflects the live state. */
  }
}

static void mqttDisconnect() {
  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_PREFIX "/status", "offline", true, 0);
    mqttClient.disconnect();
  }
  mqttEnabled = false;
  mqttConnectPending = 0;
  mqttSetStatus(LV_SYMBOL_BLUETOOTH "  Disconnected", C_MUTED);
  mqttSetActivity("--");
}

static void mqttSwCb(lv_event_t *e) {
  bool on = lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED);
  if (on) {
    if (mqttHost[0] == '\0') {
      lv_obj_remove_state(mqttSw, LV_STATE_CHECKED);
      mqttSetStatus(LV_SYMBOL_WARNING "  Set a broker first", C_WARN);
      return;
    }
    mqttEnabled = true;
    mqttConnectPending = 5;              /* loop() runs the blocking connect a few frames later */
    mqttSetStatus(LV_SYMBOL_REFRESH "  Connecting ...", C_WARN);
  } else {
    mqttDisconnect();
  }
  kvSaveU8("/kv/mqtt_on", mqttEnabled ? 1 : 0);
}

/* Broker-entry modal -- exact clone of the Recon/SSID/password deferred-
 * close pattern: mqttKbCb only captures text and arms a countdown, loop()
 * does the delete outside any LVGL event dispatch. No residual widget list
 * to free here (unlike the Recon modal), so no clean-on-open needed. */
static void closeMqttModal() {
  if (mqttModal) { lv_obj_delete(mqttModal); mqttModal = NULL; }
  lv_obj_set_style_bg_opa(lv_layer_top(), LV_OPA_TRANSP, 0);
  lv_obj_remove_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);
  mqttModalOpen = false;   /* loop()'s reconcile rebuilds the tab view next iteration */
}

/* Accepts "host" or "host:port" in the single field; defaults to 1883.
 * Stores the raw string in KVStore (port is 16-bit, so it rides along in
 * the string rather than needing its own typed key). */
static void parseMqttBroker(const char *raw) {
  char buf[80];
  strlcpy(buf, raw, sizeof(buf));
  /* trim leading/trailing spaces the on-screen keyboard makes easy to add */
  char *s = buf;
  while (*s == ' ') s++;
  char *end = s + strlen(s);
  while (end > s && end[-1] == ' ') *--end = '\0';

  char *colon = strrchr(s, ':');
  uint16_t port = 1883;
  if (colon) {
    long p = strtol(colon + 1, NULL, 10);
    if (p > 0 && p <= 65535) { port = (uint16_t)p; *colon = '\0'; }
  }
  strlcpy(mqttHost, s, sizeof(mqttHost));
  mqttPort = port;
  kvSaveStr("/kv/mqtt_broker", raw);
  mqttUpdateBrokerLbl();
}

static void mqttKbCb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    parseMqttBroker(lv_textarea_get_text(mqttTa));
    mqttSubmitted = true;
    mqttClosePending = 5;
  } else if (code == LV_EVENT_CANCEL) {
    mqttSubmitted = false;
    mqttClosePending = 5;
  }
}

static void openMqttModal() {
  lv_obj_set_style_bg_color(lv_layer_top(), lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(lv_layer_top(), LV_OPA_50, 0);
  lv_obj_add_flag(lv_layer_top(), LV_OBJ_FLAG_CLICKABLE);

  mqttModal = lv_obj_create(lv_layer_top());
  lv_obj_set_size(mqttModal, 560, 380);
  lv_obj_center(mqttModal);
  lv_obj_add_style(mqttModal, &stCard, 0);
  lv_obj_remove_flag(mqttModal, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title = lv_label_create(mqttModal);
  lv_label_set_text(title, LV_SYMBOL_UPLOAD "  Broker host, optionally :port");
  lv_obj_add_style(title, &stCardTitle, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

  mqttTa = lv_textarea_create(mqttModal);
  lv_textarea_set_one_line(mqttTa, true);
  lv_textarea_set_placeholder_text(mqttTa, "192.168.1.10:1883");
  if (mqttHost[0]) lv_textarea_set_text(mqttTa, mqttHost);
  lv_obj_set_width(mqttTa, 400);
  lv_obj_align(mqttTa, LV_ALIGN_TOP_MID, 0, 40);

  mqttKb = lv_keyboard_create(mqttModal);
  lv_obj_set_size(mqttKb, 520, 230);
  lv_obj_align(mqttKb, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_keyboard_set_textarea(mqttKb, mqttTa);
  lv_obj_add_event_cb(mqttKb, mqttKbCb, LV_EVENT_ALL, NULL);
  mqttModalOpen = true;
  DBG("mqtt modal: open");
}

/* Only *requests* the modal -- loop() frees the tab and opens it, so the tab
 * (containing this very button) is never deleted inside its own callback. */
static void openMqttModalCb(lv_event_t *e) { (void)e; mqttModalWant = true; }

/* Reflect the live engine state onto the (possibly just-rebuilt) view. */
static void mqttRefreshView() {
  if (!mqttStatusLbl) return;
  if (mqttEnabled && mqttClient.connected()) {
    lv_label_set_text_fmt(mqttStatusLbl, LV_SYMBOL_OK "  Connected to %s:%u", mqttHost, mqttPort);
    lv_obj_set_style_text_color(mqttStatusLbl, lv_color_hex(C_OK), 0);
  } else if (mqttEnabled) {
    lv_label_set_text(mqttStatusLbl, LV_SYMBOL_REFRESH "  Connecting ...");
    lv_obj_set_style_text_color(mqttStatusLbl, lv_color_hex(C_WARN), 0);
  } else {
    lv_label_set_text(mqttStatusLbl, LV_SYMBOL_BLUETOOTH "  Disconnected");
    lv_obj_set_style_text_color(mqttStatusLbl, lv_color_hex(C_MUTED), 0);
  }
  if (mqttSw) {
    if (mqttEnabled) lv_obj_add_state(mqttSw, LV_STATE_CHECKED);
    else             lv_obj_remove_state(mqttSw, LV_STATE_CHECKED);
  }
}

/* Builds the MQTT tab's widgets INTO an already-visible page. Called lazily
 * from loop() when the user switches to the tab (never at boot), and the
 * widgets are freed again on leaving -- see the mqttViewBuilt note above for
 * why this tab can't be permanent. Deliberately card-less and frugal (five
 * plain widgets, no shadowed cards, topic reference kept in the README) so
 * the transient footprint stays modest even so. */
static void buildMqttContent(lv_obj_t *page) {
  lv_obj_t *title = lv_label_create(page);
  lv_label_set_text(title, LV_SYMBOL_UPLOAD "  MQTT remote");
  lv_obj_add_style(title, &stCardTitle, 0);
  lv_obj_set_width(title, LV_PCT(100));   /* full-width -> next items wrap to their own rows */

  mqttStatusLbl = lv_label_create(page);
  lv_obj_add_style(mqttStatusLbl, &stMuted, 0);
  lv_obj_set_width(mqttStatusLbl, LV_PCT(100));

  mqttBrokerLbl = lv_label_create(page);
  lv_obj_set_width(mqttBrokerLbl, LV_PCT(100));
  lv_label_set_long_mode(mqttBrokerLbl, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_add_style(mqttBrokerLbl, &stMuted, 0);
  mqttUpdateBrokerLbl();

  lv_obj_t *setBtn = lv_button_create(page);
  lv_obj_set_size(setBtn, 180, 46);
  lv_obj_set_style_bg_color(setBtn, lv_color_hex(0x1B2A4A), 0);
  lv_obj_add_event_cb(setBtn, openMqttModalCb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *sbl = lv_label_create(setBtn);
  lv_label_set_text(sbl, LV_SYMBOL_UPLOAD " Set broker");
  lv_obj_center(sbl);

  mqttSw = lv_switch_create(page);
  lv_obj_set_size(mqttSw, 72, 36);
  lv_obj_set_style_bg_color(mqttSw, lv_color_hex(C_ACCENT), LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(mqttSw, mqttSwCb, LV_EVENT_VALUE_CHANGED, NULL);

  mqttRefreshView();   /* show current connection state, not a stale default */
  /* mqttActivityLbl left NULL -- mqttSetActivity() is NULL-safe. */
}

/* Lazy stub: remember the page, build nothing now (boot stays on the
 * dashboard, so the MQTT page must cost zero pool at boot). */
static void buildMqtt(lv_obj_t *tab) {
  mqttTabPage = tab;
}

/* Reconciled from loop(), never an event callback: build the tab's widgets
 * when it becomes active, free them when it stops being active. Freeing on
 * leave is what keeps idle/boot free at the stable ~6.8KB. */
static void mqttFreeView() {
  if (!mqttViewBuilt) return;
  lv_obj_clean(mqttTabPage);
  mqttStatusLbl = mqttBrokerLbl = mqttSw = NULL;
  mqttViewBuilt = false;
}

/* Free the MQTT tab the instant the active tab changes AWAY from it --
 * fired from the tabview's own change event, which runs before LVGL renders
 * the newly-active tab. Doing it here (rather than one loop() later in
 * mqttReconcileView) matters when connected: the connection leaves only
 * ~4.5KB free while viewing this tab, and the incoming tab's render
 * transient would OOM if the MQTT widgets were still allocated alongside it.
 * We only delete mqttTabPage's children (not the tabview whose callback this
 * is), so this isn't the delete-in-own-callback hazard. */
static void mqttTabChangeCb(lv_event_t *e) {
  (void)e;
  if (mqttModalOpen) return;
  if (lv_tabview_get_tab_active(tabview) != 10 && mqttViewBuilt) mqttFreeView();
}

static void mqttReconcileView() {
  if (!mqttTabPage) return;

  /* A modal was requested (Set-broker button or serial 'q'): free the tab
   * first so the keyboard has pool room, then open. Both here in loop(),
   * never inside the button's own callback. */
  if (mqttModalWant) {
    mqttModalWant = false;
    if (!mqttModalOpen) { mqttFreeView(); openMqttModal(); }
  }
  if (mqttModalOpen) return;   /* leave the tab empty behind the modal */

  /* Build on arrival. Freeing on leave is handled early by mqttTabChangeCb;
   * this also frees as a backstop in case the event was ever missed. */
  bool onTab = (lv_tabview_get_tab_active(tabview) == 10);
  if (onTab && !mqttViewBuilt)  { buildMqttContent(mqttTabPage); mqttViewBuilt = true; }
  else if (!onTab && mqttViewBuilt) mqttFreeView();
}

/* ================= setup / loop ================= */

void setup() {
  Serial.begin(115200);

  /* watchdog first, before anything that could hang -- see wdKick() comment.
   * wdTimeout is global (not a setup()-local) so performReconScan() can
   * restart the watchdog with the same value after deliberately pausing it. */
  mbed::Watchdog &watchdog = mbed::Watchdog::get_instance();
  uint32_t wdMax = watchdog.get_max_timeout();
  wdTimeout = wdMax < 20000 ? wdMax : 20000;
  watchdog.start(wdTimeout);
  if (Serial) {
    Serial.print("watchdog: started timeout="); Serial.print(wdTimeout);
    Serial.print(" max="); Serial.println(wdMax);
  }

  /* hardware first */
  for (int i = 0; i < 4; i++) { pinMode(RELAY_PIN[i], OUTPUT); digitalWrite(RELAY_PIN[i], LOW); }
  for (int i = 0; i < 2; i++) {
    pinMode(MOTOR[i].pwmPin, OUTPUT); pinMode(MOTOR[i].dirPin, OUTPUT);
    analogWrite(MOTOR[i].pwmPin, 0);  digitalWrite(MOTOR[i].dirPin, HIGH);
  }

  /* display + touch (Display.begin() initialises LVGL) */
  Display.begin();
  TouchDetector.begin();
  Backlight.begin();
  Backlight.set(kvLoadU8("/kv/bright", 85));

  /* shield sensors — begin() failures leave the tab in "not found" state */
#if EN_IMU
  imuOk = imu.begin() == 1;
#endif
#if EN_MIC
  PDM.onReceive(onPdmData);
  micOk = PDM.begin(1, 16000) != 0;
#endif
  initSineLut();

  initTheme();

  /* left icon rail */
  tabview = lv_tabview_create(lv_screen_active());
  lv_tabview_set_tab_bar_position(tabview, LV_DIR_LEFT);
  lv_tabview_set_tab_bar_size(tabview, 72);
  lv_obj_set_style_bg_opa(tabview, LV_OPA_TRANSP, 0);
  lv_obj_set_style_bg_opa(lv_tabview_get_content(tabview), LV_OPA_TRANSP, 0);

  lv_obj_t *tDash = lv_tabview_add_tab(tabview, LV_SYMBOL_HOME);
  lv_obj_t *tRel  = lv_tabview_add_tab(tabview, LV_SYMBOL_POWER);
  lv_obj_t *tMot  = lv_tabview_add_tab(tabview, LV_SYMBOL_CHARGE);
  lv_obj_t *tSen  = lv_tabview_add_tab(tabview, LV_SYMBOL_EYE_OPEN);
  lv_obj_t *tImu  = lv_tabview_add_tab(tabview, LV_SYMBOL_LOOP);
  lv_obj_t *tAud  = lv_tabview_add_tab(tabview, LV_SYMBOL_AUDIO);
  lv_obj_t *tWifi = lv_tabview_add_tab(tabview, LV_SYMBOL_WIFI);
  lv_obj_t *tBle  = lv_tabview_add_tab(tabview, LV_SYMBOL_BLUETOOTH);
  lv_obj_t *tSet  = lv_tabview_add_tab(tabview, LV_SYMBOL_SETTINGS);
  /* Recon appended last (index 9) rather than inserted earlier, so every
   * existing tab index (used throughout statusTimerCb/imuTimerCb/
   * audioTimerCb's tab-gating and the serial commands below) stays exactly
   * as it was -- no risk of an off-by-one from renumbering. */
  lv_obj_t *tRecon = lv_tabview_add_tab(tabview, LV_SYMBOL_GPS);
  /* MQTT appended last (index 10), same reasoning as Recon -- appending
   * never renumbers an existing tab, so all the tab-index checks in the
   * timers and serial commands stay valid. */
  lv_obj_t *tMqtt = lv_tabview_add_tab(tabview, LV_SYMBOL_UPLOAD);
  styleTab(tDash); styleTab(tRel); styleTab(tMot); styleTab(tSen);
  styleTab(tImu);  styleTab(tAud); styleTab(tWifi); styleTab(tBle); styleTab(tSet);
  styleTab(tRecon); styleTab(tMqtt);

  /* style the tab bar buttons (v9: real buttons inside the bar) */
  lv_obj_t *tb = lv_tabview_get_tab_bar(tabview);
  lv_obj_set_style_bg_color(tb, lv_color_hex(C_BG_TOP), 0);
  lv_obj_set_style_bg_opa(tb, LV_OPA_COVER, 0);
  for (uint32_t i = 0; i < lv_obj_get_child_count(tb); i++) {
    lv_obj_t *b = lv_obj_get_child(tb, i);
    lv_obj_set_style_bg_opa(b, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(b, lv_color_hex(C_MUTED), 0);
    lv_obj_set_style_text_color(b, lv_color_hex(C_ACCENT), LV_STATE_CHECKED);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(C_ACCENT), LV_STATE_CHECKED);
    lv_obj_set_style_border_side(b, LV_BORDER_SIDE_RIGHT, LV_STATE_CHECKED);
    lv_obj_set_style_border_width(b, 3, LV_STATE_CHECKED);
  }

  buildDashboard(tDash);
  buildRelays(tRel);
  buildMotors(tMot);
  buildSensors(tSen);
  buildImu(tImu);
  buildAudio(tAud);
  buildWifi(tWifi);
  buildBluetooth(tBle);
  buildSettings(tSet);
  buildRecon(tRecon);
  buildMqtt(tMqtt);
  lv_obj_add_event_cb(tabview, mqttTabChangeCb, LV_EVENT_VALUE_CHANGED, NULL);   /* free MQTT tab early on leave */

  /* Restore a saved broker address so it's shown/prefilled, but do NOT
   * auto-connect at boot: a blocking connect() to an unreachable broker
   * would stall the UI coming up (and WiFi may not be associated yet). The
   * user toggles the connect switch when they want it. */
  { char b[80]; if (kvLoadStr("/kv/mqtt_broker", b, sizeof(b))) parseMqttBroker(b); }

  updateWifiStatus();
  static const uint16_t rates[] = { 100, 300, 1000 };
  uint8_t rate = kvLoadU8("/kv/rate", 1);
  sensorTimer = lv_timer_create(sensorTimerCb, rates[rate < 3 ? rate : 1], NULL);
  lv_timer_create(statusTimerCb, 1000, NULL);
  lv_timer_create(imuTimerCb, 150, NULL);
  lv_timer_create(audioTimerCb, 120, NULL);

  /* restore persisted radios */
  if (kvLoadU8("/kv/ble_on", 0)) {
    lv_obj_add_state(bleSw, LV_STATE_CHECKED);
    bleStart();
  }
  if (kvLoadStr("/kv/wifi_ssid", pendingSsid, sizeof(pendingSsid))) {
    kvLoadStr("/kv/wifi_pass", pendingPass, sizeof(pendingPass));
    DBG("wifi: reconnecting to saved network");
    requestConnect();               /* loop() connects a moment after boot */
  }

  DBG("GIGA Control Panel ready");
}

void loop() {
  wdKick();
  lv_timer_handler();
  mqttReconcileView();     /* build the MQTT tab's widgets only while it's on screen */
  if (bleEnabled) blePoll();

  /* MQTT servicing: loop() keepalive + inbound messages when connected,
   * republish relays when they've changed, and a slow auto-reconnect if the
   * link drops while the user still wants it. mqttClient.loop() is non-
   * blocking when the socket is healthy; a dropped socket is caught by the
   * per-loop() watchdog kick either way. The reconnect itself is deferred
   * through mqttConnectPending (watchdog-paused in performMqttConnect). */
  if (mqttEnabled) {
    mqttClient.loop();
    if (mqttClient.connected()) {
      if (mqttRelaysDirty) { mqttPublishRelays(); mqttRelaysDirty = false; }
    } else if (mqttConnectPending == 0) {
      mqttSetStatus(LV_SYMBOL_REFRESH "  Reconnecting ...", C_WARN);
      mqttConnectPending = 750;            /* ~3s at 4ms/loop between attempts */
    }
  }

  /* WiFi firmware update, requested over MQTT. Runs here (not the callback)
   * because it blocks for the whole download; feeds the watchdog internally. */
  if (otaRequested) runOta();

  /* keep the DAC fed while a tone plays */
  if (toneOn && dac0.available()) {
    SampleBuffer buf = dac0.dequeue();
    for (size_t i = 0; i < buf.size(); i++) buf[i] = sineLut[i % 64];
    dac0.write(buf);
  }

  /* serial remote-drive (debugging): s=scan  w=WiFi tab  h=Home  t=click Scan */
  if (Serial.available()) {
    char cmd = Serial.read();
    if      (cmd == 's') wifiScanCb(NULL);
    else if (cmd == 'w') lv_tabview_set_active(tabview, 6, LV_ANIM_OFF);
    else if (cmd == 'h') lv_tabview_set_active(tabview, 0, LV_ANIM_OFF);
    else if (cmd == 't') lv_obj_send_event(scanBtn, LV_EVENT_CLICKED, NULL);
    else if (cmd >= '1' && cmd <= '9') {
      lv_tabview_set_active(tabview, cmd - '1', LV_ANIM_OFF);
      if (Serial) { Serial.print("tab set "); Serial.println(cmd - '1'); }
    }
    else if (cmd == '0') {              /* tab 9 = Recon, appended after the '1'-'9' range */
      lv_tabview_set_active(tabview, 9, LV_ANIM_OFF);
      if (Serial) { Serial.print("tab set "); Serial.println(9); }
    }
    /* password-modal test: p=open (dummy SSID)  c=close
     * r=simulate tapping the keyboard's checkmark (LV_EVENT_READY) with a
     * dummy password -- exercises the real connect-and-close path. */
    else if (cmd == 'p') { strlcpy(pendingSsid, "TestNet", sizeof(pendingSsid)); openPwModal(); }
    else if (cmd == 'c') closePwModal();
    else if (cmd == 'r') {
      if (pwTa) lv_textarea_set_text(pwTa, "dummyPassword123");
      if (pwKb) lv_obj_send_event(pwKb, LV_EVENT_READY, NULL);
    }
    else if (cmd == 'n') {              /* click the first scanned network */
      if (wifiList && lv_obj_get_child_count(wifiList))
        lv_obj_send_event(lv_obj_get_child(wifiList, 0), LV_EVENT_CLICKED, NULL);
    }
    else if (cmd == 'g') checkCaptivePortal();   /* run the connectivity check directly */
    else if (cmd == 'b') {                       /* toggle BLE, same as tapping the switch */
      if (lv_obj_has_state(bleSw, LV_STATE_CHECKED)) lv_obj_remove_state(bleSw, LV_STATE_CHECKED);
      else lv_obj_add_state(bleSw, LV_STATE_CHECKED);
      lv_obj_send_event(bleSw, LV_EVENT_VALUE_CHANGED, NULL);
    }
    else if (cmd == 'o') otherNetworkCb(NULL);   /* open the manual-SSID modal */
    else if (cmd == 'y') {                       /* submit a dummy hidden SSID */
      if (ssidTa) lv_textarea_set_text(ssidTa, "DummyHiddenNet");
      if (ssidKb) lv_obj_send_event(ssidKb, LV_EVENT_READY, NULL);
    }
    else if (cmd == 'x') {   /* deliberately hang forever -- verifies the watchdog resets us */
      DBG("watchdog test: spinning with no kicks, should self-reset shortly");
      while (1) { /* no wdKick() on purpose */ }
    }
    else if (cmd == 'i') openReconModal();       /* open the recon target-IP modal */
    else if (cmd == 'k') {                       /* submit a dummy target (RFC 5737 TEST-NET-1,
                                                   * non-routable -- safe for automated testing) with
                                                   * an explicit custom port list, exercising both the
                                                   * new parsing path and the default-list fallback */
      if (reconTa) lv_textarea_set_text(reconTa, "192.0.2.1 22,80,443");
      if (reconKb) lv_obj_send_event(reconKb, LV_EVENT_READY, NULL);
    }
    else if (cmd == 'j') {                       /* same, but no ports typed -- tests the fallback
                                                   * to the default COMMON_PORTS list */
      if (reconTa) lv_textarea_set_text(reconTa, "192.0.2.1");
      if (reconKb) lv_obj_send_event(reconKb, LV_EVENT_READY, NULL);
    }
    else if (cmd == 'M') {                        /* jump to the MQTT tab (index 10, past the 1-9/0 range) */
      lv_tabview_set_active(tabview, 10, LV_ANIM_OFF);
      if (Serial) { Serial.print("tab set "); Serial.println(10); }
    }
    else if (cmd == 'q') mqttModalWant = true;    /* request the broker-entry modal (loop opens it) */
    else if (cmd == 'Q') {                        /* set a broker + submit (public test broker as the
                                                   * example; use a LAN broker IP by hand for a full test) */
      if (mqttTa) lv_textarea_set_text(mqttTa, "test.mosquitto.org:1883");
      if (mqttKb) lv_obj_send_event(mqttKb, LV_EVENT_READY, NULL);
    }
    else if (cmd == 'v') {                        /* toggle the MQTT connect switch (connect/disconnect) */
      if (mqttSw) {
        if (lv_obj_has_state(mqttSw, LV_STATE_CHECKED)) lv_obj_remove_state(mqttSw, LV_STATE_CHECKED);
        else lv_obj_add_state(mqttSw, LV_STATE_CHECKED);
        lv_obj_send_event(mqttSw, LV_EVENT_VALUE_CHANGED, NULL);
      }
    }
  }

  if (scanPending    && --scanPending == 0)    { DBG("loop: scan start"); performScan(); DBG("loop: scan end"); }
  if (connectPending && --connectPending == 0) performConnect();
  if (kbClosePending && --kbClosePending == 0) {
    closePwModal();
    if (kbConnectAfterClose) requestConnect();
  }
  if (ssidClosePending && --ssidClosePending == 0) {
    closeSsidModal();
    if (ssidSubmitted) openPwModal();   /* chain into the normal password/connect flow */
  }
  if (reconClosePending && --reconClosePending == 0) {
    closeReconModal();
    if (reconSubmitted) performReconScan();
  }
  if (mqttClosePending && --mqttClosePending == 0) closeMqttModal();
  if (mqttConnectPending && --mqttConnectPending == 0) performMqttConnect();
  if (portalCheckPending && --portalCheckPending == 0) checkCaptivePortal();
  delay(4);
}
