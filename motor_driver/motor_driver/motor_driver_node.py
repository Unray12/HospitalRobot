import json
from importlib import resources

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .motor_controller import MotorController


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver")

        config = self._load_config()
        serial_cfg = config.get("serial", {})
        sub_cfg = config.get("subscribe", {})

        port = serial_cfg.get("port", "/dev/ttyUSB0")
        baudrate = serial_cfg.get("baudrate", 115200)
        timeout = serial_cfg.get("timeout", 0.1)
        cmd_topic = sub_cfg.get("cmd_vel", "/cmd_vel")

        self.motor = MotorController(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            logger=self.get_logger(),
        )

        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)

    def _cmd_cb(self, msg: Twist):
        direction, speed = self._twist_to_command(msg)
        self.motor.move(direction, speed)

    def _twist_to_command(self, msg: Twist):
        eps = 1e-3
        if abs(msg.angular.z) > eps:
            return ("RotateLeft", abs(msg.angular.z)) if msg.angular.z > 0 else ("RotateRight", abs(msg.angular.z))
        if abs(msg.linear.y) > eps:
            return ("Left", abs(msg.linear.y)) if msg.linear.y > 0 else ("Right", abs(msg.linear.y))
        if abs(msg.linear.x) > eps:
            return ("Forward", abs(msg.linear.x)) if msg.linear.x > 0 else ("Backward", abs(msg.linear.x))
        return "Stop", 0

    def destroy_node(self):
        self.motor.close()
        super().destroy_node()

    def _load_config(self):
        try:
            path = resources.files("motor_driver").joinpath("config.json")
            with path.open("r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as exc:
            self.get_logger().warn(f"config.json not loaded, using defaults: {exc}")
            return {}


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
