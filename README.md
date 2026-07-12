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
| **WiFi** | Scan networks, tap one, type the password on the on-screen keyboard, connect / disconnect; open networks connect directly. After connecting, automatically checks for a captive portal (sign-in page) and flags it if found; shows a specific error if the connect attempt fails |
| **BLE** | Toggle a BLE peripheral (`GIGA-Control`) that lets a phone control relays & motors and stream A0 |
| **Settings** | Display brightness, sensor update rate, system info, safety notes |

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
`1`–`9` switch tabs, `s` run a WiFi scan directly, `w`/`h` jump to WiFi/Home,
`t` synthesize a tap on the Scan button, `n` synthesize a tap on the first
scanned network button, `p`/`c` open/close the password modal directly
(with a dummy SSID), `r` simulate tapping the keyboard's checkmark
(`LV_EVENT_READY`) with a dummy password — exercises the real
connect-and-close path without needing a physical touch, which is how the
`kbCb` freeze (see Caveats) was actually reproduced and fixed. `g` runs
the captive-portal connectivity check directly (useful to test it without
needing an actual captive-portal network). A timestamped `hb` heartbeat
prints once a second while a monitor is attached, including live LVGL heap
stats (`free`/`big`/`frag`) — useful for catching memory issues before they
cause a freeze.

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

**Known gaps, not implemented**: no way to enter a hidden/non-broadcasting SSID by hand (only scanned networks can be picked); no WPA2-Enterprise (802.1X username+password) support, only the personal PSK mode `WiFi.begin(ssid, pass)` provides.

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
- WiFi and BLE share the GIGA's Murata radio module. Coexistence mostly works,
  but if either gets flaky, use one at a time.
- **Settings now persist across reboots** via mbed KVStore (`kvstore_global_api.h`,
  keys prefixed `/kv/`): WiFi SSID/password (auto-reconnects on boot), BLE
  enabled state, display brightness, sensor update rate. Disconnecting WiFi
  from the UI also forgets the saved network. Toggle `EN_KV` to `0` near the
  top of the sketch to stub all of this out for debugging without touching
  call sites.
- Relays switch mains-capable contacts — treat the shield's screw terminals
  with respect if you put line voltage on them.

## LVGL heap (`LV_MEM_SIZE`)

This lives in the **core**, not this repo:
`~/.arduino15/packages/arduino/hardware/mbed_giga/4.6.0/libraries/Arduino_H7_Video/src/lv_conf_9.h`,
currently `(64 * 1024U)`. A `arduino-cli core update`/reinstall will revert
any edit here back to the stock value (also 64KB by default, so a stock
reinstall happens to match what this sketch expects — but don't assume that
stays true across core versions).
