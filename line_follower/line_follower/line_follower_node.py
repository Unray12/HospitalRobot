import json
import time
from importlib import resources

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool, String
from std_srvs.srv import SetBool

from .line_follower import LineFollowerFSM


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower")

        config = self._load_config()
        topics_cfg = config.get("topics", {})
        service_cfg = config.get("service", {})

        self.base_speed = int(config.get("base_speed", 6))
        self.autoMode = False
        self._last_frame = None

        self.follower = LineFollowerFSM(
            base_speed=self.base_speed,
            crossing_duration=config.get("crossing_duration", 2.0),
            logger=self.get_logger(),
        )

        cmd_topic = topics_cfg.get("cmd_vel", "/cmd_vel")
        frame_topic = topics_cfg.get("line_frame", "/line_sensors/frame")
        auto_topic = topics_cfg.get("auto_mode", "/auto_mode")
        auto_service = service_cfg.get("set_auto_mode", "/set_auto_mode")

        self.cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self.create_subscription(Int16MultiArray, frame_topic, self._frame_cb, 10)
        self.create_subscription(Bool, auto_topic, self._auto_cb, 10)
        self.create_service(SetBool, auto_service, self._auto_srv_cb)

        self.timer = self.create_timer(0.01, self._timer_cb)

    def _frame_cb(self, msg: Int16MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warn("Line frame invalid length")
            return

        self._last_frame = {
            "left_count": msg.data[0],
            "mid_count": msg.data[1],
            "right_count": msg.data[2],
            "left_full": bool(msg.data[3]),
            "mid_full": bool(msg.data[4]),
            "right_full": bool(msg.data[5]),
        }

    def _auto_cb(self, msg: Bool):
        self._set_auto_mode(msg.data)

    def _auto_srv_cb(self, request, response):
        self._set_auto_mode(request.data)
        response.success = True
        response.message = "Auto mode updated"
        return response

    def _set_auto_mode(self, enabled: bool):
        if enabled and not self.autoMode:
            self.autoMode = True
            self.follower.reset()
            self.get_logger().info("Auto Mode Enabled")
        elif not enabled and self.autoMode:
            self.autoMode = False
            self.follower.stop()
            self._publish_stop()
            self.get_logger().info("Auto Mode Disabled")

    def _timer_cb(self):
        if not self.autoMode:
            return

        result = self.follower.update(self._last_frame, time.time())
        if result is None:
            return

        direction, speed = result
        self.cmd_pub.publish(self._command_to_msg(direction, speed))

    def _command_to_msg(self, direction, speed):
        msg = String()
        speed = int(round(speed))
        if speed < 0:
            speed = 0
        msg.data = f"{direction}:{speed}"
        return msg

    def _publish_stop(self):
        self.cmd_pub.publish(self._command_to_msg("Stop", 0))

    def _load_config(self):
        try:
            path = resources.files("line_follower").joinpath("config.json")
            with path.open("r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as exc:
            self.get_logger().warn(f"config.json not loaded, using defaults: {exc}")
            return {}


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
