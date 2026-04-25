#!/usr/bin/env bash
# Install the udev rule for the volcaniarm ESP32, baking in the serial
# number of the device currently plugged in. Idempotent.
#
#   ./install.sh           # auto-detect ESP, install rule
#   ./install.sh --uninstall   # remove the installed rule

set -euo pipefail

ESP_VID="10c4"
ESP_PID="ea60"
RULE_NAME="99-volcaniarm-esp32.rules"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="${SCRIPT_DIR}/${RULE_NAME}"
TARGET="/etc/udev/rules.d/${RULE_NAME}"
SYMLINK="/dev/volcaniarm"

if [[ "${1:-}" == "--uninstall" ]]; then
  if [[ -f "$TARGET" ]]; then
    echo "Removing $TARGET"
    sudo rm "$TARGET"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "Done. Unplug + replug the ESP (or any USB device) to fully clear $SYMLINK."
  else
    echo "$TARGET not present, nothing to do."
  fi
  exit 0
fi

if [[ ! -f "$TEMPLATE" ]]; then
  echo "ERROR: template not found at $TEMPLATE" >&2
  exit 1
fi

# Find the first /dev/ttyUSB* whose VID:PID matches our ESP.
ESP_TTY=""
for tty in /dev/ttyUSB* /dev/ttyACM*; do
  [[ -e "$tty" ]] || continue
  vid=$(udevadm info --query=property --name="$tty" 2>/dev/null \
        | awk -F= '/^ID_VENDOR_ID=/ {print $2}')
  pid=$(udevadm info --query=property --name="$tty" 2>/dev/null \
        | awk -F= '/^ID_MODEL_ID=/ {print $2}')
  if [[ "$vid" == "$ESP_VID" && "$pid" == "$ESP_PID" ]]; then
    ESP_TTY="$tty"
    break
  fi
done

if [[ -z "$ESP_TTY" ]]; then
  echo "ERROR: no ESP32 (CP210x ${ESP_VID}:${ESP_PID}) found." >&2
  echo "Plug in the ESP and try again. Sanity check: lsusb -d ${ESP_VID}:${ESP_PID}" >&2
  exit 1
fi

ESP_SERIAL=$(udevadm info --query=property --name="$ESP_TTY" \
             | awk -F= '/^ID_SERIAL_SHORT=/ {print $2}')

if [[ -z "$ESP_SERIAL" ]]; then
  echo "ERROR: ESP at $ESP_TTY has no ID_SERIAL_SHORT." >&2
  echo "The CP210x chip on this board may not store a serial -- you'll need" >&2
  echo "to match by VID/PID only. Edit $TEMPLATE manually." >&2
  exit 1
fi

echo "Found ESP32: $ESP_TTY  (serial=$ESP_SERIAL)"

# Generate the rule with the placeholder filled in, write to a temp file,
# then sudo-copy. Comparing to existing target makes the script idempotent.
TMP=$(mktemp)
trap 'rm -f "$TMP"' EXIT
sed "s|<YOUR_ESP_SERIAL>|${ESP_SERIAL}|" "$TEMPLATE" > "$TMP"

if [[ -f "$TARGET" ]] && cmp -s "$TMP" "$TARGET"; then
  echo "Rule already installed and up to date at $TARGET"
else
  echo "Installing rule to $TARGET"
  sudo install -m 644 "$TMP" "$TARGET"
  sudo udevadm control --reload-rules
  sudo udevadm trigger
fi

# Wait briefly for the symlink to appear after the trigger.
for _ in 1 2 3 4 5; do
  [[ -e "$SYMLINK" ]] && break
  sleep 0.2
done

if [[ -L "$SYMLINK" ]]; then
  echo "OK: $SYMLINK -> $(readlink -f "$SYMLINK")"
else
  echo "Rule installed, but $SYMLINK did not appear yet."
  echo "Unplug and replug the ESP to apply the rule."
fi
