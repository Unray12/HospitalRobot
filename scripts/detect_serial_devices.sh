#!/usr/bin/env bash
# Auto-detect serial devices (/dev/ttyUSB*, /dev/ttyACM*) on Raspberry Pi.
#
# Usage:
#   ./scripts/detect_serial_devices.sh                # liệt kê toàn bộ
#   ./scripts/detect_serial_devices.sh --json         # xuất JSON
#   ./scripts/detect_serial_devices.sh --rules        # in udev rules mẫu
#   ./scripts/detect_serial_devices.sh --probe motor  # tìm device khớp role
#
# Roles được phân loại bằng USB VID:PID (xem bảng KNOWN_DEVICES bên dưới).
# Nếu thiết bị chưa có trong bảng, dùng --rules để tạo symlink theo serial.

set -euo pipefail

# role | vid:pid | mô tả
# Sửa lại VID:PID cho khớp phần cứng thực tế của bạn (xem `lsusb`).
KNOWN_DEVICES=(
  "motor|1a86:7523|CH340 USB-Serial (motor driver)"
  "motor|10c4:ea60|CP210x USB-Serial (motor driver)"
  "motor|0403:6001|FTDI USB-Serial (motor driver)"
  "line|2341:0043|Arduino Uno (line sensor)"
  "line|2341:0001|Arduino Uno R1 (line sensor)"
  "line|1a86:7523|CH340 Arduino clone (line sensor)"
  "huskylens|1a86:55d3|HuskyLens CH9102"
  "huskylens|303a:1001|HuskyLens ESP32-S3"
  "camera|2341:8036|Arduino Leonardo (camera AI)"
  "camera|239a:800b|Adafruit ItsyBitsy (camera AI)"
)

list_tty() {
  shopt -s nullglob
  local devs=(/dev/ttyUSB* /dev/ttyACM*)
  shopt -u nullglob
  printf '%s\n' "${devs[@]}"
}

udev_prop() {
  udevadm info -q property -n "$1" 2>/dev/null || true
}

get_prop() {
  # $1 = output of udev_prop, $2 = key
  awk -F= -v k="$2" '$1==k {print $2; exit}' <<<"$1"
}

classify() {
  local vid="$1" pid="$2"
  local entry role epid
  for entry in "${KNOWN_DEVICES[@]}"; do
    role="${entry%%|*}"
    epid="${entry#*|}"; epid="${epid%%|*}"
    if [ "${vid}:${pid}" = "$epid" ]; then
      printf '%s\n' "$role"
      return
    fi
  done
  printf 'unknown\n'
}

mode="table"
probe_role=""
while [ $# -gt 0 ]; do
  case "$1" in
    --json)   mode="json" ;;
    --rules)  mode="rules" ;;
    --probe)  mode="probe"; probe_role="${2:-}"; shift ;;
    -h|--help)
      sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; exit 2 ;;
  esac
  shift
done

if ! command -v udevadm >/dev/null 2>&1; then
  echo "udevadm not found (cần systemd-udev)." >&2
  exit 1
fi

mapfile -t devices < <(list_tty)
if [ "${#devices[@]}" -eq 0 ]; then
  echo "Không tìm thấy /dev/ttyUSB* hoặc /dev/ttyACM*." >&2
  exit 1
fi

emit_table_header() {
  printf '%-15s %-10s %-10s %-20s %-12s %s\n' \
    "DEVICE" "VID" "PID" "SERIAL" "ROLE" "PRODUCT"
  printf '%s\n' "-------------------------------------------------------------------------------------"
}

emit_rule() {
  local role="$1" vid="$2" pid="$3" serial="$4"
  if [ -n "$serial" ]; then
    printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", ATTRS{serial}=="%s", SYMLINK+="hospitalrobot/%s", MODE="0666"\n' \
      "$vid" "$pid" "$serial" "$role"
  else
    printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", SYMLINK+="hospitalrobot/%s", MODE="0666"\n' \
      "$vid" "$pid" "$role"
  fi
}

[ "$mode" = "table" ] && emit_table_header
[ "$mode" = "rules" ] && {
  cat <<'HDR'
# /etc/udev/rules.d/99-hospitalrobot-serial.rules
# Sinh tự động bởi scripts/detect_serial_devices.sh --rules
# Áp dụng:
#   sudo cp 99-hospitalrobot-serial.rules /etc/udev/rules.d/
#   sudo udevadm control --reload-rules && sudo udevadm trigger
# Sau đó dùng các đường dẫn cố định:
#   /dev/hospitalrobot/motor
#   /dev/hospitalrobot/line
#   /dev/hospitalrobot/huskylens
#   /dev/hospitalrobot/camera
HDR
}
[ "$mode" = "json" ] && printf '[\n'

first_json=1
match_dev=""

for dev in "${devices[@]}"; do
  props="$(udev_prop "$dev")"
  vid="$(get_prop "$props" ID_VENDOR_ID)"
  pid="$(get_prop "$props" ID_MODEL_ID)"
  serial="$(get_prop "$props" ID_SERIAL_SHORT)"
  product="$(get_prop "$props" ID_MODEL)"
  role="$(classify "$vid" "$pid")"

  case "$mode" in
    table)
      printf '%-15s %-10s %-10s %-20s %-12s %s\n' \
        "$dev" "${vid:--}" "${pid:--}" "${serial:--}" "$role" "${product:--}"
      ;;
    json)
      [ $first_json -eq 0 ] && printf ',\n'
      first_json=0
      printf '  {"device":"%s","vid":"%s","pid":"%s","serial":"%s","role":"%s","product":"%s"}' \
        "$dev" "$vid" "$pid" "$serial" "$role" "$product"
      ;;
    rules)
      if [ "$role" != "unknown" ] && [ -n "$vid" ] && [ -n "$pid" ]; then
        emit_rule "$role" "$vid" "$pid" "$serial"
      else
        printf '# (bỏ qua %s: vid=%s pid=%s role=%s)\n' "$dev" "${vid:--}" "${pid:--}" "$role"
      fi
      ;;
    probe)
      if [ "$role" = "$probe_role" ]; then
        match_dev="$dev"
        break
      fi
      ;;
  esac
done

case "$mode" in
  json)  printf '\n]\n' ;;
  probe)
    if [ -z "$match_dev" ]; then
      echo "Không tìm thấy device cho role '$probe_role'." >&2
      exit 1
    fi
    printf '%s\n' "$match_dev"
    ;;
esac
