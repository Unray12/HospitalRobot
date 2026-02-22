#!/usr/bin/env python3
import sys
import launch
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


def main():
    ld = LaunchDescription(
        [
            Node(package="line_sensors", executable="line_sensor_driver", output="screen"),
            Node(package="line_follower", executable="line_follower", output="screen"),
            Node(package="motor_driver", executable="motor_driver", output="screen"),
            Node(package="manual_control", executable="manual_control", output="screen"),
        ]
    )

    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    raise SystemExit(main())
