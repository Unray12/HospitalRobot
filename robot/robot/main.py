#!/usr/bin/env python3
import json
import sys
from importlib import resources

import launch
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


def _load_config():
    try:
        path = resources.files("robot").joinpath("config.json")
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def main():
    config = _load_config()
    nodes_cfg = config.get("bringup", {}).get(
        "nodes",
        ["line_sensor_driver", "line_follower", "motor_driver", "manual_control"],
    )

    node_map = {
        "line_sensor_driver": ("line_sensors", "line_sensor_driver"),
        "line_follower": ("line_follower", "line_follower"),
        "motor_driver": ("motor_driver", "motor_driver"),
        "manual_control": ("manual_control", "manual_control"),
    }

    actions = []
    for name in nodes_cfg:
        if name not in node_map:
            print(f"[robot bringup] Unknown node '{name}' - skipped")
            continue
        pkg, exe = node_map[name]
        actions.append(Node(package=pkg, executable=exe, output="screen"))

    ld = LaunchDescription(actions)

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    raise SystemExit(main())
