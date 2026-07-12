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
static lv_obj_t *accBar[3], *gyroBar[3], *accLbl, *gyroLbl, *imuStatusLbl;
static lv_obj_t *micBar, *micLbl, *micChart, *toneFreqLbl;
static lv_chart_series_t *micSer;
static lv_obj_t *sysInfoLbl;
static lv_timer_t *sensorTimer;

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

static void toneStart() {
  for (int i = 0; i < 64; i++)
    sineLut[i] = 2048 + (int)(2000.0f * sinf(i * 6.2831853f / 64.0f));
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
          LV_SYMBOL_WARNING "  Connected to %s -- internet not confirmed\n"
          "IP %u.%u.%u.%u    this network may need sign-in (open a browser on another device)",
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
        LV_SYMBOL_CLOSE "  Couldn't connect to %s -- check the password", pendingSsid);
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
  WiFiClient client;
  client.setSocketTimeout(4000);
  bool suspected = true;   /* assume no confirmed internet until proven otherwise */
  if (client.connect("connectivitycheck.gstatic.com", 80)) {
    client.print("GET /generate_204 HTTP/1.1\r\nHost: connectivitycheck.gstatic.com\r\nConnection: close\r\n\r\n");
    char statusLine[48] = {0};
    uint32_t start = millis();
    size_t n = 0;
    while (millis() - start < 4000 && n < sizeof(statusLine) - 1) {
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
  int st = strlen(pendingPass) ? WiFi.begin(pendingSsid, pendingPass)
                               : WiFi.begin(pendingSsid);
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
  if (Serial) { Serial.print("scan: status="); Serial.println(WiFi.status());
                Serial.print("scan: fw=");     Serial.println(WiFi.firmwareVersion()); }
  int n = WiFi.scanNetworks();
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

static void sensorTimerCb(lv_timer_t *t) {
  (void)t;
  for (int i = 0; i < 4; i++) {
    int v = analogRead(ANALOG_PIN[i]);
    lv_bar_set_value(sensorBar[i], v, LV_ANIM_OFF);
    lv_label_set_text_fmt(sensorLbl[i], "%s   %4d   %d.%02d V",
        ANALOG_NAME[i], v, (v * 33) / 10230, ((v * 330) / 1023) % 100);
    if (i == 0) {
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
  /* dashboard: wifi */
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    lv_label_set_text_fmt(dashWifiLbl, "%s\n%u.%u.%u.%u  (%ld dBm)",
        WiFi.SSID(), ip[0], ip[1], ip[2], ip[3], (long)WiFi.RSSI());
    lv_obj_set_style_text_color(dashWifiLbl, lv_color_hex(C_OK), 0);
  } else {
    lv_label_set_text(dashWifiLbl, "Offline\nuse the WiFi tab");
    lv_obj_set_style_text_color(dashWifiLbl, lv_color_hex(C_MUTED), 0);
  }
  /* WiFi tab: catch connection drops (weak signal, router reboot, ...)
   * while the user is just sitting on the tab -- otherwise the label only
   * updates on an explicit scan/connect/disconnect action and can go
   * stale. Skipped while a scan/connect/portal-check/modal-close is in
   * flight so this doesn't clobber a transient "Scanning..." message. */
  if (!scanPending && !connectPending && !portalCheckPending && !kbClosePending)
    updateWifiStatus();
  /* dashboard + tab: BLE */
  const char *bleTxt; uint32_t bleCol;
  if (!bleEnabled)                { bleTxt = "Disabled";                 bleCol = C_MUTED; }
  else if (BLE.central())         { bleTxt = "Central connected";        bleCol = C_OK;    }
  else                            { bleTxt = "Advertising as\n" BLE_DEVICE_NAME; bleCol = C_ACCENT; }
  lv_label_set_text(dashBleLbl, bleTxt);
  lv_obj_set_style_text_color(dashBleLbl, lv_color_hex(bleCol), 0);
  lv_label_set_text_fmt(bleStatusLbl, LV_SYMBOL_BLUETOOTH "  %s", bleTxt);
  lv_obj_set_style_text_color(bleStatusLbl, lv_color_hex(bleCol), 0);
  /* uptime */
  uint32_t s = millis() / 1000;
  lv_label_set_text_fmt(dashUpLbl, "Uptime  %luh %02lum %02lus",
      (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
  /* settings: system info */
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
  /* stream A0 to a subscribed BLE central */
  if (bleEnabled && BLE.central()) sensorChar.writeValue((uint16_t)analogRead(ANALOG_PIN[0]));
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
  lv_obj_t *c = makeCard(tab, 642, 96);
  cardTitle(c, LV_SYMBOL_WIFI, "Wireless network");
  wifiStatusLbl = lv_label_create(c);
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

/* ================= setup / loop ================= */

void setup() {
  Serial.begin(115200);

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
  styleTab(tDash); styleTab(tRel); styleTab(tMot); styleTab(tSen);
  styleTab(tImu);  styleTab(tAud); styleTab(tWifi); styleTab(tBle); styleTab(tSet);

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
  lv_timer_handler();
  if (bleEnabled) blePoll();

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
  }

  if (scanPending    && --scanPending == 0)    { DBG("loop: scan start"); performScan(); DBG("loop: scan end"); }
  if (connectPending && --connectPending == 0) performConnect();
  if (kbClosePending && --kbClosePending == 0) {
    closePwModal();
    if (kbConnectAfterClose) requestConnect();
  }
  if (portalCheckPending && --portalCheckPending == 0) checkCaptivePortal();
  delay(4);
}
