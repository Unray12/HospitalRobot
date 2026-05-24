#!/usr/bin/env bash
# ============================================================================
# BOOTSTRAP — setup ROS workspace + serial auto-detect, KHÔNG launch.
#
#   Stage 1: setup_raspberry_ros.sh  (apt + rosdep + colcon build)
#   Stage 2: setup_serial_auto.sh    (udev + systemd + symlinks)
#
# Sau khi script kết thúc, terminal trả về prompt — bạn tự mở các terminal
# khác để chạy:
#   ros2 run mqtt_bridge mqtt_bridge
#   ros2 run robot robot
#
# Cách dùng:
#   ./scripts/bootstrap_all.sh                 # build + serial (mặc định)
#   ./scripts/bootstrap_all.sh --skip-build    # bỏ stage 1 (đã build)
#   ./scripts/bootstrap_all.sh --skip-serial   # bỏ stage 2
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

c_grn='\033[1;32m'; c_ylw='\033[1;33m'; c_red='\033[1;31m'; c_off='\033[0m'
hr()    { printf "${c_grn}%s${c_off}\n" "================================================================"; }
stage() { hr; printf "${c_grn}== %s${c_off}\n" "$*"; hr; }
warn()  { printf "${c_ylw}[warn]${c_off} %s\n" "$*"; }
die()   { printf "${c_red}[err]${c_off}  %s\n" "$*" >&2; exit 1; }

do_build=1
do_serial=1

while [ $# -gt 0 ]; do
  case "$1" in
    --skip-build)   do_build=0 ;;
    --skip-serial)  do_serial=0 ;;
    -h|--help)      sed -n '2,16p' "$0"; exit 0 ;;
    *) die "Unknown arg: $1" ;;
  esac
  shift
done

cd "$REPO_DIR"

# ---------------------------------------------------------------------------
# Stage 1 — Build workspace
# ---------------------------------------------------------------------------
if [ $do_build -eq 1 ]; then
  stage "Stage 1/2 — ROS workspace setup (apt + rosdep + colcon build)"
  [ -x ./scripts/setup_raspberry_ros.sh ] || chmod +x ./scripts/setup_raspberry_ros.sh
  ./scripts/setup_raspberry_ros.sh
else
  stage "Stage 1/2 — SKIP build"
fi

# ---------------------------------------------------------------------------
# Stage 2 — Serial auto-detection
# ---------------------------------------------------------------------------
if [ $do_serial -eq 1 ]; then
  stage "Stage 2/2 — Serial auto-detection (udev + systemd)"
  [ -x ./scripts/setup_serial_auto.sh ]    || chmod +x ./scripts/setup_serial_auto.sh
  [ -x ./scripts/init_serial_symlinks.py ] || chmod +x ./scripts/init_serial_symlinks.py
  if [ "$EUID" -eq 0 ]; then
    ./scripts/setup_serial_auto.sh
  else
    sudo ./scripts/setup_serial_auto.sh
  fi

  if [ -d /dev/hospitalrobot ]; then
    echo
    echo "Symlinks sẵn sàng:"
    ls -l /dev/hospitalrobot/
  else
    warn "Chưa có /dev/hospitalrobot/. Debug: sudo ./scripts/setup_serial_auto.sh --probe"
  fi
else
  stage "Stage 2/2 — SKIP serial"
fi

# ---------------------------------------------------------------------------
# Hướng dẫn launch (script kết thúc, không treo)
# ---------------------------------------------------------------------------
hr
printf "${c_grn}Setup xong. Mở terminal khác để chạy:${c_off}\n"
hr
cat <<EOF

  # Source ROS + workspace (làm 1 lần mỗi terminal):
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source $REPO_DIR/install/setup.bash

  # Terminal 1 — MQTT bridge (cần cho keyboard + MQTT control):
  ros2 run mqtt_bridge mqtt_bridge

  # Terminal 2 — Robot bringup (tất cả driver nodes):
  ros2 run robot robot

  # Tùy chọn — kiểm tra:
  ls -l /dev/hospitalrobot/
  ros2 topic echo /huskylens/frame
  ros2 topic pub /pick_robot std_msgs/msg/String "{data: '1'}" --once  # auto mode

EOF
