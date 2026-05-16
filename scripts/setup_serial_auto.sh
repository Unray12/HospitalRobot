#!/usr/bin/env bash
# ============================================================================
# Setup all-in-one cho serial auto-detection trên HospitalRobot.
#
# Sau khi chạy 1 lần:
#   - /dev/hospitalrobot/{motor,line,camera,...} luôn trỏ đúng device
#   - Tự refresh khi cắm/rút USB hoặc reboot
#
# Cách dùng:
#   sudo ./scripts/setup_serial_auto.sh           # cài + chạy
#   sudo ./scripts/setup_serial_auto.sh --probe   # chỉ probe (dry-run) để xem
#   sudo ./scripts/setup_serial_auto.sh --uninstall
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

UNIT_FILE="/etc/systemd/system/hospitalrobot-serial.service"
UDEV_TRIGGER="/etc/udev/rules.d/99-hospitalrobot-trigger.rules"
UDEV_OLD_RULES="/etc/udev/rules.d/99-hospitalrobot-serial.rules"
PROBE_SCRIPT="$SCRIPT_DIR/init_serial_symlinks.py"

c_red='\033[31m'; c_grn='\033[32m'; c_ylw='\033[33m'; c_off='\033[0m'
say()  { printf "${c_grn}[setup]${c_off} %s\n" "$*"; }
warn() { printf "${c_ylw}[warn]${c_off}  %s\n" "$*"; }
die()  { printf "${c_red}[err]${c_off}   %s\n" "$*" >&2; exit 1; }

require_root() {
  [ "$EUID" -eq 0 ] || die "Cần sudo. Chạy: sudo $0 $*"
}

uninstall() {
  require_root
  say "Stop + disable service"
  systemctl disable --now hospitalrobot-serial.service 2>/dev/null || true
  rm -f "$UNIT_FILE" "$UDEV_TRIGGER" "$UDEV_OLD_RULES"
  systemctl daemon-reload
  udevadm control --reload-rules || true
  rm -rf /dev/hospitalrobot
  say "Đã gỡ. Xong."
  exit 0
}

probe_only() {
  require_root
  say "Stop ROS nodes (nếu có)"
  pkill -f "ros2 launch|ros2 run" 2>/dev/null || true
  sleep 1
  say "Stop service (tránh tranh port)"
  systemctl stop hospitalrobot-serial.service 2>/dev/null || true
  say "Chạy probe (dry-run, không tạo symlink)"
  echo "---"
  python3 "$PROBE_SCRIPT" --dry-run || true
  echo "---"
  say "Dump 200 byte đầu của mỗi port (giúp tinh chỉnh signature):"
  for d in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$d" ] || continue
    echo
    echo "### $d (@ 115200)"
    stty -F "$d" 115200 raw -echo 2>/dev/null || true
    timeout 2 head -c 200 "$d" 2>/dev/null | cat -v || true
    echo
    echo "### $d (@ 9600)"
    stty -F "$d" 9600 raw -echo 2>/dev/null || true
    timeout 2 head -c 200 "$d" 2>/dev/null | cat -v || true
  done
  exit 0
}

# ---- args ----
case "${1:-}" in
  --uninstall) uninstall ;;
  --probe)     probe_only ;;
  -h|--help)
    sed -n '2,15p' "$0"; exit 0 ;;
esac

require_root

# ---- 1. Dọn rule cũ từ phiên bản trước ----
say "Dọn udev rule cũ (nếu có)"
rm -f "$UDEV_OLD_RULES"

# ---- 2. Kiểm tra pyserial ----
if ! python3 -c "import serial" 2>/dev/null; then
  say "Cài python3-serial"
  apt-get update
  apt-get install -y python3-serial
fi

# ---- 3. Dừng ROS đang chiếm port (nếu có) ----
if pgrep -f "ros2 launch|ros2 run|line_sensor_driver|camera_sensor|huskylens|motor_driver" >/dev/null 2>&1; then
  warn "Đang có ROS node chạy — tạm dừng để probe được port"
  pkill -f "ros2 launch|ros2 run" || true
  sleep 2
fi

# ---- 4. Cài systemd unit ----
say "Cài $UNIT_FILE"
cat > "$UNIT_FILE" <<EOF
[Unit]
Description=HospitalRobot - probe serial devices and create stable symlinks
After=systemd-udev-settle.service
Wants=systemd-udev-settle.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/bin/sleep 2
ExecStart=/usr/bin/python3 ${PROBE_SCRIPT}
ExecStop=/bin/sh -c 'rm -rf /dev/hospitalrobot'
TimeoutStartSec=120

[Install]
WantedBy=multi-user.target
EOF
chmod 644 "$UNIT_FILE"

# ---- 5. Cài udev trigger ----
say "Cài $UDEV_TRIGGER"
cat > "$UDEV_TRIGGER" <<'EOF'
ACTION=="add|remove", SUBSYSTEM=="tty", KERNEL=="ttyUSB*|ttyACM*", \
  RUN+="/bin/systemctl --no-block restart hospitalrobot-serial.service"
EOF
chmod 644 "$UDEV_TRIGGER"

# ---- 6. Reload + start ----
say "Reload systemd + udev"
systemctl daemon-reload
udevadm control --reload-rules

say "Enable + start service"
systemctl enable hospitalrobot-serial.service
if ! systemctl restart hospitalrobot-serial.service; then
  warn "Service báo lỗi — xem log dưới:"
fi

echo
say "Trạng thái service:"
systemctl --no-pager --full status hospitalrobot-serial.service || true

echo
say "Log mới nhất:"
journalctl -u hospitalrobot-serial.service -n 40 --no-pager || true

echo
say "Symlink hiện tại:"
if [ -d /dev/hospitalrobot ]; then
  ls -l /dev/hospitalrobot/
else
  warn "Chưa có /dev/hospitalrobot/ — service chưa probe ra được role nào."
  warn "Chạy: sudo $0 --probe   để xem raw bytes mỗi port và chỉnh PROBES trong init_serial_symlinks.py"
fi

echo
say "Hoàn tất. Có thể launch ROS:"
echo "  source /opt/ros/humble/setup.bash && source install/setup.bash"
echo "  ros2 launch robot bringup.launch.py"
