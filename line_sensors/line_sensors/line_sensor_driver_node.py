import json
from importlib import resources

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

from .line_sensor_reader import LineSensorReader


class LineSensorDriverNode(Node):
    def __init__(self):
        super().__init__("line_sensor_driver")

        config = self._load_config()
        serial_cfg = config.get("serial", {})
        pub_cfg = config.get("publish", {})

        port = serial_cfg.get("port", "/dev/ttyACM0")
        baudrate = serial_cfg.get("baudrate", 115200)
        timeout = serial_cfg.get("timeout", 0.1)
        topic = pub_cfg.get("topic", "/line_sensors/frame")
        rate_hz = pub_cfg.get("rate_hz", 100)

        self.reader = LineSensorReader(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            logger=self.get_logger(),
        )

        self.pub = self.create_publisher(Int16MultiArray, topic, 10)
        period = 1.0 / max(rate_hz, 1)
        self.timer = self.create_timer(period, self._timer_cb)

    def _timer_cb(self):
        frame = self.reader.read_frame()
        if frame is None:
            return

        msg = Int16MultiArray()
        msg.data = [
            int(frame["left_count"]),
            int(frame["mid_count"]),
            int(frame["right_count"]),
            int(frame["left_full"]),
            int(frame["mid_full"]),
            int(frame["right_full"]),
        ]
        self.pub.publish(msg)

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()

    def _load_config(self):
        try:
            path = resources.files("line_sensors").joinpath("config.json")
            with path.open("r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as exc:
            self.get_logger().warn(f"config.json not loaded, using defaults: {exc}")
            return {}


def main(args=None):
    rclpy.init(args=args)
    node = LineSensorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
