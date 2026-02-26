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
        filter_cfg = config.get("filter", {})

        port = serial_cfg.get("port", "/dev/ttyACM0")
        baudrate = serial_cfg.get("baudrate", 115200)
        timeout = serial_cfg.get("timeout", 0.1)
        topic = pub_cfg.get("topic", "/line_sensors/frame")
        rate_hz = pub_cfg.get("rate_hz", 100)
        debug_toggle_topic = topics_cfg.get("debug_toggle", "/debug_logs_toggle")
        self.reconnect_period_sec = float(config.get("reconnect_period_sec", 2.0))
        self.fallback_ports = config.get("fallback_ports", ["/dev/ttyACM1", "/dev/ttyACM0"])
        self.scan_prefixes = config.get("scan_prefixes", ["/dev/ttyACM", "/dev/ttyUSB", "COM"])
        self.debug_log_period = float(config.get("debug_log_period", 0.2))
        self.debug_enabled = bool(config.get("debug_enabled_default", False))
        self.zero_hold_sec = float(filter_cfg.get("zero_hold_sec", 0.15))
        self.zero_min_streak = int(max(1, filter_cfg.get("zero_min_streak", 3)))
        self._last_debug_log_ts = 0.0
        self._zero_streak = 0
        self._last_nonzero_frame = None
        self._last_nonzero_ts = 0.0
        self._last_published_frame = None

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
        self.reconnect_timer = self.create_timer(self.reconnect_period_sec, self._reconnect_cb)

    def _debug_toggle_cb(self, msg: Bool):
        self.debug_enabled = bool(msg.data)
        state = "ON" if self.debug_enabled else "OFF"
        self.log.info(f"Line sensor debug log: {state}", event="DEBUG_TOGGLE")

    def _timer_cb(self):
        frame = self.reader.read_frame()
        if frame is None:
            return
        frame = self._filter_frame(frame)

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
        self._last_published_frame = dict(frame)
        self._log_sensor_debug(frame)

    def _reconnect_cb(self):
        if self.reader.is_connected():
            return
        if self.reader.reconnect(
            fallback_ports=self.fallback_ports,
            scan_prefixes=self.scan_prefixes,
        ):
            self.log.info("Line sensor serial reconnected", event="SERIAL")

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

    def _filter_frame(self, frame):
        now = self.get_clock().now().nanoseconds / 1e9
        is_zero = (
            int(frame.get("left_count", 0)) == 0
            and int(frame.get("mid_count", 0)) == 0
            and int(frame.get("right_count", 0)) == 0
            and not bool(frame.get("left_full", False))
            and not bool(frame.get("mid_full", False))
            and not bool(frame.get("right_full", False))
        )

        if not is_zero:
            self._zero_streak = 0
            self._last_nonzero_frame = dict(frame)
            self._last_nonzero_ts = now
            return frame

        self._zero_streak += 1
        if (
            self._last_nonzero_frame is not None
            and (now - self._last_nonzero_ts) < self.zero_hold_sec
        ):
            return dict(self._last_nonzero_frame)
        if self._zero_streak < self.zero_min_streak and self._last_published_frame is not None:
            return dict(self._last_published_frame)
        return frame

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = LineSensorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
