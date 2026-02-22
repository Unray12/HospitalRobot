import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool

from .line_sensor_reader import LineSensorReader
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter


class LineSensorDriverNode(Node):
    def __init__(self):
        super().__init__("line_sensor_driver")
        self.log = LogAdapter(self.get_logger(), "line_sensors")

        config = ConfigManager("line_sensors", logger=self.log).load()
        serial_cfg = config.get("serial", {})
        pub_cfg = config.get("publish", {})
        topics_cfg = config.get("topics", {})

        port = serial_cfg.get("port", "/dev/ttyACM0")
        baudrate = serial_cfg.get("baudrate", 115200)
        timeout = serial_cfg.get("timeout", 0.1)
        topic = pub_cfg.get("topic", "/line_sensors/frame")
        rate_hz = pub_cfg.get("rate_hz", 100)
        debug_toggle_topic = topics_cfg.get("debug_toggle", "/debug_logs_toggle")
        self.debug_log_period = float(config.get("debug_log_period", 0.2))
        self.debug_enabled = bool(config.get("debug_enabled_default", False))
        self._last_debug_log_ts = 0.0

        self.reader = LineSensorReader(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            logger=self.log,
        )

        self.pub = self.create_publisher(Int16MultiArray, topic, 10)
        self.create_subscription(Bool, debug_toggle_topic, self._debug_toggle_cb, 10)
        period = 1.0 / max(rate_hz, 1)
        self.timer = self.create_timer(period, self._timer_cb)

    def _debug_toggle_cb(self, msg: Bool):
        self.debug_enabled = bool(msg.data)
        state = "ON" if self.debug_enabled else "OFF"
        self.log.info(f"Line sensor debug log: {state}", event="DEBUG_TOGGLE")

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
        self._log_sensor_debug(frame)

    def _log_sensor_debug(self, frame):
        if not self.debug_enabled:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_debug_log_ts) < self.debug_log_period:
            return
        self._last_debug_log_ts = now
        self.log.info(
            "Line frame",
            event="SENSOR",
            left=frame["left_count"],
            mid=frame["mid_count"],
            right=frame["right_count"],
            left_full=int(frame["left_full"]),
            mid_full=int(frame["mid_full"]),
            right_full=int(frame["right_full"]),
        )

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = LineSensorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
