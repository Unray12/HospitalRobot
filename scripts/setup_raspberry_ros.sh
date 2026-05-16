#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_SOURCE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd -- "$REPO_SOURCE_DIR/.." && pwd)}"
REPO_DIR="${REPO_DIR:-$WORKSPACE_DIR/src/HospitalRobot}"
SKIP_APT="${SKIP_APT:-0}"
SKIP_TESTS="${SKIP_TESTS:-0}"

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
    python3-rosdep \
    python3-serial \
    python3-paho-mqtt \
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
}

install_rosdep_packages() {
  log "Installing ROS package dependencies via rosdep"
  rosdep install \
    --from-paths "$WORKSPACE_DIR/src" \
    --ignore-src \
    --rosdistro "$ROS_DISTRO" \
    -y
}

build_workspace() {
  log "Building workspace"
  cd "$WORKSPACE_DIR"
  colcon build --symlink-install

  # shellcheck source=/dev/null
  source "$WORKSPACE_DIR/install/setup.bash"
}

run_tests() {
  if [ "$SKIP_TESTS" = "1" ]; then
    log "Skipping tests because SKIP_TESTS=1"
    return
  fi

  log "Running package tests"
  cd "$WORKSPACE_DIR"
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
  colcon test-result --verbose
}

write_shell_hint() {
  shell_rc="$HOME/.bashrc"
  workspace_source="source $WORKSPACE_DIR/install/setup.bash"

  if ! grep -Fq "$workspace_source" "$shell_rc" 2>/dev/null; then
    log "Adding workspace source to $shell_rc"
    {
      echo ""
      echo "# HospitalRobot ROS2 workspace"
      echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
      echo "$workspace_source"
    } >> "$shell_rc"
  fi
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

Serial ports to verify:
  Motor: /dev/ttyUSB0
  Line sensor: /dev/ttyACM0
  HuskyLens: /dev/ttyACM1
  Camera: /dev/ttyACM2

Useful env overrides:
  ROS_DISTRO=humble ./scripts/setup_raspberry_ros.sh
  WORKSPACE_DIR=../ros2_ws ./scripts/setup_raspberry_ros.sh
  SKIP_APT=1 ./scripts/setup_raspberry_ros.sh
  SKIP_TESTS=1 ./scripts/setup_raspberry_ros.sh
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
