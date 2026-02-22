import rclpy
from rclpy.executors import MultiThreadedExecutor

from line_sensors.line_sensor_driver_node import LineSensorDriverNode
from line_follower.line_follower_node import LineFollowerNode
from motor_driver.motor_driver_node import MotorDriverNode
from manual_control.manual_control_node import ManualControlNode


def main(args=None):
    rclpy.init(args=args)

    nodes = [
        LineSensorDriverNode(),
        LineFollowerNode(),
        MotorDriverNode(),
        ManualControlNode(),
    ]

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
