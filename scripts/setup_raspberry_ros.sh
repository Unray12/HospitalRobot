#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_SOURCE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$REPO_SOURCE_DIR}"
REPO_DIR="${REPO_DIR:-$WORKSPACE_DIR/src/HospitalRobot}"
WORKSPACE_SRC_PATH=""
SKIP_APT="${SKIP_APT:-0}"
RUN_TESTS="${RUN_TESTS:-0}"

log() {
  printf '\n[setup] %s\n' "$1"
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing command: $1" >&2
    exit 1
  fi
}

ensure_ubuntu() {
  if [ ! -r /etc/os-release ]; then
    echo "Cannot detect OS. This script expects Ubuntu on Raspberry Pi." >&2
    exit 1
  fi
  . /etc/os-release
  if [ "${ID:-}" != "ubuntu" ]; then
    echo "Unsupported OS: ${ID:-unknown}. Use Ubuntu for ROS2 apt packages." >&2
    exit 1
  fi
}

install_apt_deps() {
  if [ "$SKIP_APT" = "1" ]; then
    log "Skipping apt install because SKIP_APT=1"
    return
  fi

  log "Installing apt dependencies"
  sudo apt update
  sudo apt install -y \
    curl \
    git \
    gnupg \
    lsb-release \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    python3-serial \
    python3-paho-mqtt \
    ros-${ROS_DISTRO}-ament-copyright \
    ros-${ROS_DISTRO}-ament-flake8 \
    ros-${ROS_DISTRO}-ament-pep257 \
    ros-${ROS_DISTRO}-rcl-interfaces \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs
}

ensure_ros_sourced() {
  if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    set +u
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    return
  fi

  echo "ROS distro '${ROS_DISTRO}' not found at /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  echo "Install ROS2 first, or set ROS_DISTRO to an installed distro." >&2
  exit 1
}

init_rosdep() {
  if [ "$SKIP_APT" = "1" ]; then
    return
  fi

  log "Initializing rosdep"
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
  fi
  rosdep update
}

prepare_workspace() {
  log "Preparing workspace: $WORKSPACE_DIR"

  if [ -f "$WORKSPACE_DIR/robot_common/package.xml" ] && [ -f "$WORKSPACE_DIR/robot/package.xml" ]; then
    WORKSPACE_SRC_PATH="$WORKSPACE_DIR"
    cd "$WORKSPACE_DIR"
    log "Using current repository as workspace root"
    return
  fi

  mkdir -p "$WORKSPACE_DIR/src"

  if [ ! -d "$REPO_DIR" ]; then
    log "Copying repository into workspace src from $REPO_SOURCE_DIR"
    mkdir -p "$(dirname "$REPO_DIR")"
    cp -a "$REPO_SOURCE_DIR" "$REPO_DIR"
  fi

  cd "$WORKSPACE_DIR"
  if [ ! -f "src/HospitalRobot/robot_common/package.xml" ]; then
    echo "Workspace layout invalid: expected src/HospitalRobot under $WORKSPACE_DIR" >&2
    exit 1
  fi

  WORKSPACE_SRC_PATH="$WORKSPACE_DIR/src"
}

install_rosdep_packages() {
  log "Installing ROS package dependencies via rosdep"
  rosdep install \
    --from-paths "$WORKSPACE_SRC_PATH" \
    --ignore-src \
    --rosdistro "$ROS_DISTRO" \
    -y
}

build_workspace() {
  log "Building workspace"
  cd "$WORKSPACE_DIR"
  colcon build --symlink-install

  set +u
  # shellcheck source=/dev/null
  source "$WORKSPACE_DIR/install/setup.bash"
  set -u
}

run_tests() {
  if [ "$RUN_TESTS" != "1" ]; then
    log "Skipping tests by default. Set RUN_TESTS=1 to enable."
    return
  fi

  log "Running package tests"
  cd "$WORKSPACE_DIR"
  set +e
  colcon test --packages-select \
    robot_common \
    camera_sensor \
    line_sensors \
    huskylens_sensor \
    line_follower \
    manual_control \
    motor_driver \
    mqtt_bridge \
    robot
  test_rc=$?
  colcon test-result --verbose
  result_rc=$?
  set -e

  if [ "$test_rc" -ne 0 ] || [ "$result_rc" -ne 0 ]; then
    echo "" >&2
    echo "[setup] Tests reported failures. See 'colcon test-result --verbose' output above." >&2
    echo "[setup] Re-run with RUN_TESTS=1 only when you want to debug tests." >&2
    exit 1
  fi
}

write_shell_hint() {
  shell_rc="$HOME/.bashrc"
  begin_marker="# >>> HospitalRobot ROS2 workspace >>>"
  end_marker="# <<< HospitalRobot ROS2 workspace <<<"

  if [ -f "$shell_rc" ]; then
    tmp_rc="$(mktemp)"
    # Dọn 2 dạng block cũ:
    #  1) Block có marker mới  ">>> HospitalRobot ROS2 workspace >>>" ... "<<<"
    #  2) Block legacy bắt đầu  "# HospitalRobot ROS2 workspace" cho đến "set -u"
    #     (phiên bản cũ ghi set +u/set -u → leak nounset vào shell tương tác).
    awk -v b="$begin_marker" -v e="$end_marker" '
      $0==b           {skip=1; next}
      $0==e           {skip=0; next}
      $0=="# HospitalRobot ROS2 workspace" {legacy=1; next}
      legacy && $0=="set -u" {legacy=0; next}
      legacy          {next}
      !skip           {print}
    ' "$shell_rc" > "$tmp_rc"
    mv "$tmp_rc" "$shell_rc"
  fi

  log "Refreshing workspace source block in $shell_rc"
  {
    echo ""
    echo "$begin_marker"
    echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then"
    echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "fi"
    echo "if [ -f $WORKSPACE_DIR/install/setup.bash ]; then"
    echo "  source $WORKSPACE_DIR/install/setup.bash"
    echo "fi"
    echo "$end_marker"
  } >> "$shell_rc"
}

print_next_steps() {
  cat <<EOF

Setup complete.

Use this shell now:
  cd ${WORKSPACE_DIR}
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source ./install/setup.bash

Check packages:
  ros2 pkg list | grep -E "robot|line_sensors|line_follower|motor_driver|manual_control|mqtt_bridge|camera_sensor|huskylens_sensor"

Run system:
  ros2 run mqtt_bridge mqtt_bridge
  ros2 run robot robot

Run individual nodes:
  ros2 run camera_sensor camera_sensor
  ros2 run line_sensors line_sensor_driver
  ros2 run huskylens_sensor huskylens_sensor
  ros2 run line_follower line_follower
  ros2 run manual_control manual_control
  ros2 run motor_driver motor_driver

Useful topics:
  ros2 topic echo /VR_control
  ros2 topic echo /line_sensors/frame
  ros2 topic echo /motor_cmd
  ros2 topic echo /auto_mode

Auto mode publish:
  ros2 topic pub /pick_robot std_msgs/msg/String "{data: '1'}" --once
  ros2 topic pub /pick_robot std_msgs/msg/String "{data: '0'}" --once

MQTT test:
  mosquitto_sub -h 172.28.182.106 -p 1883 -t VR_control -v

Serial ports (cố định qua symlink):
  Motor:       /dev/hospitalrobot/motor
  Line sensor: /dev/hospitalrobot/line
  HuskyLens:   /dev/hospitalrobot/huskylens
  Camera:      /dev/hospitalrobot/camera

Setup serial auto-detect (1 lần, content-probe + systemd):
  sudo ./scripts/setup_serial_auto.sh
  ls -l /dev/hospitalrobot/

Hoặc chạy bootstrap toàn bộ (build + serial + launch):
  ./scripts/bootstrap_all.sh --launch

Useful env overrides:
  ROS_DISTRO=humble ./scripts/setup_raspberry_ros.sh
  WORKSPACE_DIR=../ros2_ws ./scripts/setup_raspberry_ros.sh
  SKIP_APT=1 ./scripts/setup_raspberry_ros.sh
  RUN_TESTS=1 ./scripts/setup_raspberry_ros.sh
  REPO_DIR=./src/HospitalRobot ./scripts/setup_raspberry_ros.sh
EOF
}

main() {
  ensure_ubuntu
  need_cmd sudo
  install_apt_deps
  ensure_ros_sourced
  init_rosdep
  prepare_workspace
  install_rosdep_packages
  build_workspace
  run_tests
  write_shell_hint
  print_next_steps
}

main "$@"
