#!/usr/bin/env bash
#
# giga-ota.sh — build the GIGA Control Panel firmware, package it as an .ota
# image, serve it over the LAN, and tell the GIGA (over MQTT) to pull and
# apply it. Run from jessy; the GIGA fetches from jessy over the WiFi it's
# already on. The very first OTA-capable firmware must be flashed once over
# USB (this file's OTA client has to already be running on the board); after
# that, this script updates it over WiFi.
#
# Recovery: a bad image is un-bricked with the usual USB DFU flash
# (arduino-cli upload ...). OTA never removes that safety net.
#
# SPDX-License-Identifier: MIT

set -euo pipefail

# ---- config (override via env) -------------------------------------------
SKETCH_DIR="${SKETCH_DIR:-$HOME/GigaControlPanel}"
FQBN="${FQBN:-arduino:mbed_giga:giga}"
BROKER="${BROKER:-127.0.0.1}"          # mosquitto broker the GIGA is connected to
MQTT_PORT="${MQTT_PORT:-1883}"
HTTP_PORT="${HTTP_PORT:-8099}"
LAN_IP="${LAN_IP:-}"                    # auto-detected if empty; the address the GIGA reaches jessy at
TOPIC_PREFIX="${TOPIC_PREFIX:-giga-control}"
TOOLS="${TOOLS:-$HOME/.local/share/giga-ota-tools}"

WORK="$(mktemp -d)"
HTTP_PID=""
SUB_PID=""
cleanup() {
  [ -n "$SUB_PID" ] && kill "$SUB_PID" 2>/dev/null || true
  [ -n "$HTTP_PID" ] && kill "$HTTP_PID" 2>/dev/null || true
  rm -rf "$WORK"
}
trap cleanup EXIT

say() { printf '\033[1;38;5;208m▸ %s\033[0m\n' "$*"; }   # AutoZone orange, because why not

# ---- 0. tools (fetch + build once, cached in $TOOLS) ----------------------
# lzss.py loads a compiled ./lzss.so; bin2ota.py needs the crccheck module.
# Both are set up here so the script is self-contained on a fresh machine.
mkdir -p "$TOOLS"
base=https://raw.githubusercontent.com/arduino-libraries/ArduinoIoTCloud/master/extras/tools
for t in lzss.py lzss.c bin2ota.py; do
  [ -f "$TOOLS/$t" ] || { say "fetching $t"; curl -fsSL "$base/$t" -o "$TOOLS/$t"; }
done
[ -f "$TOOLS/lzss.so" ] || { say "building lzss.so"; gcc -shared -fPIC -o "$TOOLS/lzss.so" "$TOOLS/lzss.c"; }
PY="$TOOLS/venv/bin/python"
# crccheck can't go in the PEP-668 system Python, so keep it in a small venv.
[ -x "$PY" ] || { say "creating tools venv (+ crccheck)"; python3 -m venv "$TOOLS/venv"; "$TOOLS/venv/bin/pip" install --quiet crccheck; }

# ---- 1. compile -----------------------------------------------------------
say "compiling $SKETCH_DIR for $FQBN"
arduino-cli compile --fqbn "$FQBN" --output-dir "$WORK" "$SKETCH_DIR" >/dev/null
BIN="$(ls "$WORK"/*.ino.bin | head -1)"
[ -f "$BIN" ] || { echo "no .bin produced"; exit 1; }
say "built $(basename "$BIN") ($(du -h "$BIN" | cut -f1))"

# ---- 2. package: bin -> lzss -> ota --------------------------------------
# Run from $TOOLS so lzss.py finds its ./lzss.so; pass absolute in/out paths.
say "packaging .ota (lzss + GIGA header)"
( cd "$TOOLS" && "$PY" lzss.py    --encode "$BIN" "$WORK/fw.lzss" )
( cd "$TOOLS" && "$PY" bin2ota.py GIGA     "$WORK/fw.lzss" "$WORK/GigaControlPanel.ota" )
OTA="$WORK/GigaControlPanel.ota"
say "image ready: $(du -h "$OTA" | cut -f1)"

# ---- 3. figure out the address the GIGA reaches jessy at ------------------
if [ -z "$LAN_IP" ]; then
  # source IP of the default route = jessy's LAN address (NOT the tailscale
  # 100.x, which the GIGA can't route to). Override with LAN_IP=... if wrong.
  LAN_IP="$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')"
fi
[ -n "$LAN_IP" ] || { echo "could not auto-detect LAN_IP; set LAN_IP=... and rerun"; exit 1; }
URL="http://$LAN_IP:$HTTP_PORT/GigaControlPanel.ota"

# ---- 4. serve the image on the LAN ---------------------------------------
say "serving on $URL"
python3 -m http.server "$HTTP_PORT" --bind "$LAN_IP" --directory "$WORK" >/dev/null 2>&1 &
HTTP_PID=$!
sleep 1

# ---- 5. watch OTA status, then trigger -----------------------------------
if command -v mosquitto_sub >/dev/null; then
  ( mosquitto_sub -h "$BROKER" -p "$MQTT_PORT" -t "$TOPIC_PREFIX/ota/status" \
      | sed -u 's/^/   GIGA: /' ) &
  SUB_PID=$!
  sleep 1
fi

say "publishing update trigger to $TOPIC_PREFIX/ota/update"
mosquitto_pub -h "$BROKER" -p "$MQTT_PORT" -t "$TOPIC_PREFIX/ota/update" -m "$URL"

say "waiting for the GIGA to download + reboot (Ctrl-C to stop early)…"
# Keep the HTTP server up long enough for the fetch; the board reboots itself.
sleep "${WAIT:-90}"
say "done — if the GIGA rebooted into the new firmware, the update landed."
