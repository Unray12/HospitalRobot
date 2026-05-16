#!/usr/bin/env bash
# Cài systemd service tự probe + tạo symlink /dev/hospitalrobot/* khi boot.
# Chạy 1 lần, sau đó symlink luôn tồn tại đúng sau mỗi reboot/cắm lại USB.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
UNIT_SRC="$SCRIPT_DIR/hospitalrobot-serial.service"
UNIT_DST="/etc/systemd/system/hospitalrobot-serial.service"

if [ "$EUID" -ne 0 ]; then
  echo "Cần sudo. Chạy: sudo $0" >&2
  exit 1
fi

# Substitute __REPO_DIR__ và cài.
sed "s|__REPO_DIR__|${REPO_DIR}|g" "$UNIT_SRC" > "$UNIT_DST"
chmod 644 "$UNIT_DST"

# Udev rule: cắm/rút ttyUSB hoặc ttyACM thì restart service để probe lại.
UDEV_RULE="/etc/udev/rules.d/99-hospitalrobot-trigger.rules"
cat > "$UDEV_RULE" <<'EOF'
ACTION=="add", SUBSYSTEM=="tty", KERNEL=="ttyUSB*|ttyACM*", \
  TAG+="systemd", ENV{SYSTEMD_WANTS}="hospitalrobot-serial.service", \
  RUN+="/bin/systemctl --no-block restart hospitalrobot-serial.service"
ACTION=="remove", SUBSYSTEM=="tty", KERNEL=="ttyUSB*|ttyACM*", \
  RUN+="/bin/systemctl --no-block restart hospitalrobot-serial.service"
EOF
chmod 644 "$UDEV_RULE"
udevadm control --reload-rules

systemctl daemon-reload
systemctl enable hospitalrobot-serial.service
systemctl restart hospitalrobot-serial.service

echo
echo "Trạng thái:"
systemctl --no-pager status hospitalrobot-serial.service || true

echo
echo "Symlink sau khi probe:"
ls -l /dev/hospitalrobot/ 2>/dev/null || echo "  (chưa có — xem journalctl -u hospitalrobot-serial -n 60)"
