#!/usr/bin/env bash
# ============================================================================
# BOOTSTRAP ALL — chạy 1 phát từ đầu tới cuối.
#
#   Stage 1: setup_raspberry_ros.sh  (apt + rosdep + colcon build)
#   Stage 2: setup_serial_auto.sh    (udev + systemd + symlinks)
#   Stage 3: launch ROS (optional)
#
# Cách dùng:
#   ./scripts/bootstrap_all.sh                  # cài + build + serial, KHÔNG launch
#   ./scripts/bootstrap_all.sh --launch         # cài + build + serial + ros2 launch
#   ./scripts/bootstrap_all.sh --skip-build     # bỏ stage 1 (đã build trước rồi)
#   ./scripts/bootstrap_all.sh --skip-serial    # bỏ stage 2
#   ./scripts/bootstrap_all.sh --launch-only    # chỉ launch (đã setup xong)
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# Lệnh launch — sửa nếu launch file của bạn khác.
LAUNCH_CMD="${LAUNCH_CMD:-ros2 run robot robot}"

c_grn='\033[1;32m'; c_ylw='\033[1;33m'; c_red='\033[1;31m'; c_off='\033[0m'
hr()    { printf "${c_grn}%s${c_off}\n" "================================================================"; }
stage() { hr; printf "${c_grn}== %s${c_off}\n" "$*"; hr; }
warn()  { printf "${c_ylw}[warn]${c_off} %s\n" "$*"; }
die()   { printf "${c_red}[err]${c_off}  %s\n" "$*" >&2; exit 1; }

do_build=1
do_serial=1
do_launch=0

while [ $# -gt 0 ]; do
  case "$1" in
    --launch)       do_launch=1 ;;
    --launch-only)  do_build=0; do_serial=0; do_launch=1 ;;
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
  stage "Stage 1/3 — ROS workspace setup (apt + rosdep + colcon build)"
  [ -x ./scripts/setup_raspberry_ros.sh ] || chmod +x ./scripts/setup_raspberry_ros.sh
  ./scripts/setup_raspberry_ros.sh
else
  stage "Stage 1/3 — SKIP build"
fi

# ---------------------------------------------------------------------------
# Stage 2 — Serial auto-detection
# ---------------------------------------------------------------------------
if [ $do_serial -eq 1 ]; then
  stage "Stage 2/3 — Serial auto-detection (udev + systemd)"
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
    warn "Chưa có /dev/hospitalrobot/. Chạy để debug: sudo ./scripts/setup_serial_auto.sh --probe"
  fi
else
  stage "Stage 2/3 — SKIP serial"
fi

# ---------------------------------------------------------------------------
# Stage 3 — Launch ROS
# ---------------------------------------------------------------------------
if [ $do_launch -eq 1 ]; then
  stage "Stage 3/3 — Launch ROS: $LAUNCH_CMD"

  set +u
  # shellcheck source=/dev/null
  [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] && source "/opt/ros/${ROS_DISTRO}/setup.bash"
  # shellcheck source=/dev/null
  [ -f "$REPO_DIR/install/setup.bash" ] && source "$REPO_DIR/install/setup.bash"
  set -u

  cd "$REPO_DIR"
  exec $LAUNCH_CMD
else
  stage "Stage 3/3 — SKIP launch"
  echo
  echo "Để launch thủ công:"
  echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
  echo "  source $REPO_DIR/install/setup.bash"
  echo "  $LAUNCH_CMD"
  echo
  echo "Hoặc chạy lại với cờ --launch:"
  echo "  ./scripts/bootstrap_all.sh --launch-only"
fi
