# GIGA Control Panel

Multi-tab touch control center for the **Arduino GIGA R1 WiFi** + **GIGA Display
Shield** + **Arduino 4 Relays Shield**, built with LVGL 9.

## Tabs (icon rail, left edge)

| Tab | What it does |
|-----|--------------|
| **Home** | Status cards (WiFi, BLE, relays, system), live A0 gauge, uptime, red **ALL OFF** kill button (relays + motors) |
| **Relays** | Toggle each of the 4 shield relays (pins D4 / D7 / D8 / D12), with indicator LEDs |
| **Motors** | 2 motor channels for an external H-bridge: speed slider (PWM), FWD/REV, enable |
| **Sensors** | Scrolling live chart of A0 + value bars for A0–A3 (raw + volts), update-rate control |
| **IMU** | Display Shield BMI270: live accelerometer (g) and gyroscope (dps) bars |
| **Audio** | Display Shield PDM microphone level meter + history chart; sine-tone generator out the audio jack (DAC0 / A12) with frequency slider |
| **WiFi** | Scan networks, tap one, type the password on the on-screen keyboard, connect / disconnect; open networks connect directly. "Other network…" lets you type a hidden/out-of-range SSID by hand. After connecting, automatically checks for a captive portal (sign-in page) and flags it if found; shows a specific error if the connect attempt fails |
| **BLE** | Toggle a BLE peripheral (`GIGA-Control`) that lets a phone control relays & motors and stream A0 |
| **Settings** | Display brightness, sensor update rate, system info, safety notes |
| **Recon** | TCP connect-scan of a user-entered target IP across 8 common ports. **For authorized security testing only** — see the Recon section below |

## Pin map

| Function | Pins | Notes |
|----------|------|-------|
| Relays 1–4 | D4, D7, D8, D12 | Fixed by the 4 Relays Shield, active HIGH |
| Motor A | PWM D2, DIR D3 | Wire to H-bridge (e.g. L298N EN + IN) |
| Motor B | PWM D5, DIR D6 | " |
| Sensors | A0–A3 | 10-bit reads, 3.3 V reference |
| Tone out | A12 (DAC0) | Routed to the Display Shield audio jack |
| IMU | Wire1 (I²C) | BMI270 on the Display Shield |

Motor pins were chosen to not collide with the relay shield. **The GIGA is a
3.3 V board** — don't feed 5 V sensor outputs into the analog pins.

## Build & upload (from jessy / this UNO Q)

Everything is already set up on this board. With the GIGA plugged into the USB
hub:

```bash
arduino-cli compile --fqbn arduino:mbed_giga:giga ~/GigaControlPanel
arduino-cli board list                                   # find the port (ttyACM0/1)
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_giga:giga ~/GigaControlPanel
```

Libraries: `lvgl` **9.x**, `Arduino_GigaDisplayTouch`, `Arduino_GigaDisplay`,
`ArduinoBLE`, `Arduino_BMI270_BMM150`, `Arduino_AdvancedAnalog` (PDM and
Arduino_H7_Video ship with the core).

If the upload fails with "No DFU capable USB device" and the board disappears
from `lsusb`, the dock swallowed the DFU re-enumeration — replug the GIGA's USB
cable or double-tap its RESET button, then upload again.

## Serial debug commands (115200 baud)

The sketch accepts single-character commands for remote testing:
`1`–`9` switch tabs, `0` jumps to tab 9 (Recon, appended after the `1`-`9`
range so it doesn't renumber any existing tab), `s` run a WiFi scan
directly, `w`/`h` jump to WiFi/Home, `t` synthesize a tap on the Scan
button, `n` synthesize a tap on the first scanned network button, `p`/`c`
open/close the password modal directly (with a dummy SSID), `r` simulate
tapping the keyboard's checkmark (`LV_EVENT_READY`) with a dummy password —
exercises the real connect-and-close path without needing a physical
touch, which is how the `kbCb` freeze (see Caveats) was actually
reproduced and fixed. `g` runs the captive-portal connectivity check
directly (useful to test it without needing an actual captive-portal
network). `o`/`y` open the manual-SSID modal and submit a dummy hidden
SSID (chains into the password modal, same as tapping "Other network…"
for real). `b` toggles BLE, same as tapping its switch. `i`/`k` open the
Recon target-IP modal and submit a dummy target (RFC 5737 `192.0.2.1`,
guaranteed non-routable — safe for automated testing, though note this is
also the *slowest* case for a scan, see the Recon section for why). `x`
deliberately hangs forever with no watchdog kicks — used to verify the
watchdog actually resets the board (see Caveats); don't send it unless you
mean it, the board will reboot a few seconds later. A timestamped `hb`
heartbeat prints once a second while a monitor is
attached, including live LVGL heap stats (`free`/`big`/`frag`) — useful for
catching memory issues before they cause a freeze.

## WiFi connect states

The WiFi tab's status line distinguishes:

| State | What it means |
|-------|----------------|
| **Not connected** (grey) | No attempt made yet, or explicitly disconnected |
| **Connecting to X …** (amber) | `WiFi.begin()` in progress (blocks `loop()` for a few seconds — normal) |
| **Couldn't connect to X — check the password** (red) | `WiFi.begin()` didn't return `WL_CONNECTED` (wrong password, network out of range, or the network doesn't exist) |
| **Connected to X** (green) | Link + DHCP OK, *and* a follow-up connectivity check confirmed real internet access |
| **Connected to X — internet not confirmed** (amber) | Link + DHCP OK, but the connectivity check didn't get the expected response — most likely a captive portal (hotel/airport/coffee-shop sign-in page), possibly just no real internet |

**Captive portals**: a network that requires "sign in to continue" in a browser will still let this device associate and get an IP — `WiFi.begin()` reports success — but blocks real traffic until sign-in completes. This device has no browser to show that page, so it can only *detect* the situation (a standard technique: fetch `http://connectivitycheck.gstatic.com/generate_204`, which a captive portal intercepts) and tell you to complete sign-in on another device (phone/laptop) connected to the same network. It cannot complete sign-in itself. This check runs automatically ~0.6s after every successful connect (including the auto-reconnect-on-boot), and can be re-run manually via the `g` serial command.

**Hidden/manual SSID entry**: tap "Other network…" on the WiFi tab to type a network name by hand (for hidden or out-of-range networks not in the scan list). Submitting it opens the normal password keyboard for that SSID — leave the password blank and submit for an open network. Built with the same create-fresh, defer-close-out-of-the-callback pattern as the password modal (see Caveats) to avoid the same class of freeze.

**Known gap, not implemented, and not planned**: no WPA2-Enterprise (802.1X username+password, the kind schools/corporate offices use) support. Checked the installed WiFi library headers directly — only `begin(ssid)` and `begin(ssid, passphrase, security)` exist, no EAP/802.1X anywhere. Supporting it would mean dropping below the Arduino WiFi library to raw mbed network APIs, too large an effort for what's fundamentally a home-network control panel.

## Recon tab — for authorized security testing only

**Only scan hosts and networks you have explicit permission to test.** This is a standard TCP port scanner, the same category of tool as `nmap`, and carries the same expectation of authorized use.

Tap "Set target & scan", type an IP address, submit — it TCP connect-scans 8 common ports (FTP 21, SSH 22, Telnet 23, HTTP 80, HTTPS 443, SMB 445, RDP 3389, HTTP-alt 8080) and lists each as open/closed. Results also print to serial (`recon: port <N> open/closed`) for an audit trail.

**What this device's WiFi hardware genuinely cannot do**: monitor mode, raw 802.11 frame access, packet injection. Checked `WiFi.h`/`WiFi.cpp` directly — the library only exposes station/AP mode via `WiFiClient`. This isn't a missing feature to add later; the Murata WiFi module + Arduino WiFi library don't expose that layer at all. (For that class of testing, jessy's onboard `ath10k_snoc` adapter *does* support monitor mode per `iw list` — see the security-toolkit notes on that device instead.)

**Known limitation, found during testing, not just assumed**: `WiFiClient::setSocketTimeout()` does **not** bound the `connect()` phase on this core — confirmed by reading `MbedClient.cpp`: the timeout is only ever applied for the SSL variant and for I/O *after* a successful connect, never before a plain `connect()`. Scanning a **live, responsive** host is fast (proven against a real host on the local network: 8 ports in ~0.12s). Scanning an **unresponsive/offline** host is slow — each port's `connect()` falls back to the underlying mbed network stack's own default timeout, observed to take tens of seconds *total* across 8 ports rather than the sub-second the code originally assumed, and the whole UI is unresponsive for that entire duration (the scan runs synchronously in `loop()`, blocking `lv_timer_handler()` along with everything else). It always eventually completes on its own — confirmed by letting one run to completion rather than assuming — no crash, no permanent hang. The hardware watchdog is deliberately paused for the duration of a scan specifically because of this (a slow scan against an unresponsive host isn't the kind of hang the watchdog should "fix" by rebooting mid-scan) and resumed immediately after. If this becomes an actual annoyance, the real fix is a non-blocking `TCPSocket` + manual poll loop instead of `WiFiClient::connect()`, not a different timeout call.

## BLE protocol (service `19B10000-E8F2-537E-4F6C-D104768A1214`)

| Char | UUID suffix | Size | Access | Payload |
|------|-------------|------|--------|---------|
| Relays | `19B10001` | 1 B | read/write | bit0..bit3 = relay 1..4 |
| Motors | `19B10002` | 3 B | write | `[motor 0/1, speed 0–100, dir 0=REV 1=FWD]` |
| Sensor | `19B10003` | 2 B | read/notify | raw A0, little-endian, ~1 Hz |

Test with **nRF Connect** or **LightBlue** on your phone.

## Caveats

- **Never use `lv_list` in this sketch.** Rendering any lv_list item
  (`lv_list_add_text` or `lv_list_add_button`) hard-freezes the board with
  this core's LVGL 9 display driver — the whole UI and USB serial lock up.
  The WiFi network list is a plain flex column of regular `lv_button`s for
  exactly this reason.
- **`LV_MEM_SIZE` is 64KB** (set in the core's `lv_conf_9.h`, not this repo —
  see "LVGL heap" below). Larger pools (128KB, 256KB) were tried to fix the
  original scan-freeze bug and made things *worse*: 256KB hard-froze the
  board within a second of boot, reproducibly, on a fully idle UI with no
  interaction at all. 128KB booted fine but froze under load. The actual
  fix was reducing peak allocation, not growing the pool: the WiFi scan list
  is capped to 8 networks (`ssidCache[8][40]`, was 15), and the password
  modal is rebuilt fresh on each open/close rather than kept permanently
  resident (a resident keyboard widget left only ~4.8KB free indefinitely —
  too thin a margin for hours of continuous dashboard/IMU/audio timer churn).
- **A "list of items" widget list costs more LVGL pool space than it looks
  like it should.** The Recon results list originally scanned 16 ports;
  populating 16 result rows (same `lv_obj` + `lv_label` + per-item
  `lv_obj_set_style_*` pattern as the WiFi scan list) against a real host
  drove the LVGL heap down to **280 bytes free** — a near-OOM condition,
  confirmed by actually running the scan rather than assuming it was fine
  because it compiled. Cut to 8 ports (matching the WiFi scan list's
  already-proven-stable cap) and it settled at a stable ~4.3KB free
  instead. If you're tempted to make a results list longer than 8 items
  anywhere in this file, verify the actual heap headroom with the `hb`
  telemetry rather than assuming it'll be fine.
- **`WiFiClient::setSocketTimeout()` does not bound `connect()` on this
  core.** Found while testing the Recon scan, not assumed: reading
  `MbedClient.cpp` shows `connect(IPAddress, port)` calls `sock->connect()`
  directly on a freshly created `TCPSocket` — `set_timeout()` is only ever
  wired up for the SSL connect variant and for I/O *after* a successful
  plain connect, never before one. A scan against an unresponsive host can
  therefore block far longer than the timeout value passed to
  `setSocketTimeout()` would suggest (confirmed: tripped the watchdog
  during testing before this was understood). See the Recon section above
  for the actual fix (the watchdog is deliberately paused for the
  duration of a scan) and the real numbers observed.
- **Closing the password modal must NOT happen synchronously from inside
  the keyboard's own event callback.** `kbCb` fires `LV_EVENT_READY` when
  the user taps the keyboard's checkmark. An earlier version of this code
  called `closePwModal()` (which deletes the modal, i.e. deletes the very
  keyboard whose event is currently dispatching) directly from within
  `kbCb` — this reproducibly hard-froze the board, confirmed with a serial
  test command (`r`) that injects a synthetic `LV_EVENT_READY` without
  needing a physical touch. Wrapping the delete in `lv_obj_delete_async()`
  did **not** fix it — same freeze either way. The actual fix mirrors the
  existing "blocking WiFi work must not run inside an LVGL event callback"
  rule in this file: `kbCb` now only captures the typed password and arms
  `kbClosePending` (a countdown, same pattern as `scanPending`/
  `connectPending`); `loop()` does the actual `closePwModal()` +
  `requestConnect()` a few frames later, fully outside of any LVGL event
  dispatch. Confirmed stable for 90+ seconds including the real
  open→type→checkmark→close→connect-attempt→idle sequence.
- WiFi scan/connect are blocking calls run from `loop()` (never from an LVGL
  event callback) — the screen pauses a few seconds during a scan. Normal.
- `Serial` prints are guarded by `if (Serial)` (the `DBG` macro): on this mbed
  core an unguarded print blocks forever if a monitor attached once and went
  away.
- WiFi and BLE share the GIGA's Murata radio module. Ran both simultaneously
  for 3+ minutes (WiFi connected + BLE advertising, switching tabs and
  rescanning throughout) with no issues — stable heap, no drops. If either
  still gets flaky in practice, fall back to using one at a time.
- **Hardware watchdog** (`mbed::Watchdog`, `drivers/Watchdog.h`): started in
  `setup()` at `min(20000ms, get_max_timeout())`, kicked once per `loop()`
  iteration plus explicitly around `performScan()`, `performConnect()`, and
  `checkCaptivePortal()` — the only calls that can legitimately block long
  enough to matter (`WiFi.begin()` with no explicit security re-scans
  internally before connecting, confirmed by reading `WiFi.cpp`). If the
  sketch ever hangs — a bug nobody's found yet, a peripheral fault, anything
  — the board resets itself instead of needing a manual power-cycle.
  Verified with a real test, not just code review: the `x` serial command
  spins forever with no kicks, and `dmesg` confirmed an actual USB
  disconnect/reconnect (the board resetting) followed by a clean reboot.
- **`imuTimerCb`/`audioTimerCb` only run while their own tab is active**
  (checked via `lv_tabview_get_tab_active(tabview)`) — no point polling the
  IMU over I²C or computing mic RMS for bars nobody can see. `sensorTimerCb`
  deliberately stays ungated since the Dashboard's A0 gauge depends on it
  running on every tab.
- **Settings now persist across reboots** via mbed KVStore (`kvstore_global_api.h`,
  keys prefixed `/kv/`): WiFi SSID/password (auto-reconnects on boot), BLE
  enabled state, display brightness, sensor update rate. Disconnecting WiFi
  from the UI also forgets the saved network. Toggle `EN_KV` to `0` near the
  top of the sketch to stub all of this out for debugging without touching
  call sites.
- Relays switch mains-capable contacts — treat the shield's screw terminals
  with respect if you put line voltage on them.
- Three status labels (`wifiStatusLbl`, `dashWifiLbl`, `sysInfoLbl`) display
  a live SSID (up to 32 chars) or a generated message and previously had no
  width/wrap constraint — a long real-world SSID or the captive-portal
  message could overflow its card. Fixed with explicit
  `lv_obj_set_width()` + `lv_label_set_long_mode(..., LV_LABEL_LONG_MODE_WRAP)`
  on all three, the WiFi status card grew 96→118px tall for wrap headroom,
  and the portal/failure message text was shortened. Verified functionally
  (compiles, connects, fails-gracefully, no crash) but **not yet visually
  confirmed on the actual screen** — see Pending below.

## Pending (needs the user physically at the board)

Two things from the latest optimization pass couldn't be finished remotely:

1. **Hardware verification.** Everything validated this session was driven
   over serial with only the heartbeat/memory telemetry as a witness.
   Relays, motors (needs an external H-bridge — confirm one's actually
   wired up), sensor voltage tracking, IMU response to physical movement,
   audio tone/mic, and BLE control from a real phone app have not been
   confirmed against actual hardware.
2. **Visual UI check.** The text-overflow fixes above and the new "Other
   network…" button placement (150×40 at `LV_ALIGN_RIGHT_MID` offset
   `-260,6` on the WiFi card) are reasoned through the numbers but not
   seen on screen.

## LVGL heap (`LV_MEM_SIZE`)

This lives in the **core**, not this repo:
`~/.arduino15/packages/arduino/hardware/mbed_giga/4.6.0/libraries/Arduino_H7_Video/src/lv_conf_9.h`,
currently `(64 * 1024U)`. A `arduino-cli core update`/reinstall will revert
any edit here back to the stock value (also 64KB by default, so a stock
reinstall happens to match what this sketch expects — but don't assume that
stays true across core versions).
