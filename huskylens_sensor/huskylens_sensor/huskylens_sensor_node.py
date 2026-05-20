import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .huskylens_parser import (
    default_frame,
    is_frame_tracking_valid,
    normalize_huskylens_payload,
    to_frame_message,
)
from .huskylens_sensor_reader import HuskyLensSensorReader
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter


class HuskyLensSensorNode(Node):
    def __init__(self):
        super().__init__("huskylens_sensor")
        self.log = LogAdapter(self.get_logger(), "huskylens_sensor")

        config = ConfigManager("huskylens_sensor", logger=self.log).load()
        serial_cfg = config.get("serial", {})
        pub_cfg = config.get("publish", {})

        port = str(serial_cfg.get("port", "/dev/ttyACM2"))
        baudrate = int(serial_cfg.get("baudrate", 115200))
        read_timeout = float(serial_cfg.get("timeout", 0.2))
        topic_frame = str(pub_cfg.get("topic_frame", "/huskylens/frame"))
        topic_valid = str(pub_cfg.get("topic_valid", "/huskylens/valid"))
        rate_hz = float(pub_cfg.get("rate_hz", 20.0))

        self.reconnect_period_sec = float(config.get("reconnect_period_sec", 2.0))
        self.fallback_ports = config.get("fallback_ports", ["/dev/ttyACM2", "/dev/ttyACM1", "/dev/ttyACM0"])
        self.scan_prefixes = config.get("scan_prefixes", ["/dev/ttyACM", "/dev/ttyUSB", "COM"])
        self.parse_log_every = int(max(1, config.get("parse_log_every", 20)))
        self.status_log_period = float(max(0.1, config.get("status_log_period", 2.0)))
        self._parse_error_count = 0
        self._last_valid_state = None
        self._last_status_log_ts = 0.0

        self.reader = HuskyLensSensorReader(
            port=port,
            baudrate=baudrate,
            timeout=read_timeout,
            logger=self.log,
        )

        self.frame_pub = self.create_publisher(String, topic_frame, 10)
        self.valid_pub = self.create_publisher(Bool, topic_valid, 10)
        period = 1.0 / max(rate_hz, 1.0)
        self.timer = self.create_timer(period, self._timer_cb)
        self.reconnect_timer = self.create_timer(self.reconnect_period_sec, self._reconnect_cb)
        self.log.info(
            f"Reading HuskyLens serial from {port} @ {baudrate} -> topics '{topic_frame}', '{topic_valid}'",
            event="BOOT",
        )

    def _timer_cb(self):
        line = self.reader.read_line()
        if line is None:
            return

        frame, err = normalize_huskylens_payload(line)
        # Control frame (boot/info/error) — không publish, không log.
        if frame is None and err == "skip":
            return
        if frame is None:
            self._parse_error_count += 1
            frame = default_frame()
            frame["connected"] = 1 if self.reader.is_connected() else 0
            if self._parse_error_count % self.parse_log_every == 1:
                self.log.warning(f"Dropped malformed HuskyLens frame ({err}): {line}", event="PARSE")
        else:
            self._parse_error_count = 0

        self.frame_pub.publish(String(data=to_frame_message(frame)))
        valid = is_frame_tracking_valid(frame)
        self.valid_pub.publish(Bool(data=valid))
        self._log_status(frame, valid)

    def _reconnect_cb(self):
        if self.reader.is_connected():
            return
        if self.reader.reconnect(
            fallback_ports=self.fallback_ports,
            scan_prefixes=self.scan_prefixes,
        ):
            self.log.info("HuskyLens serial reconnected", event="SERIAL")

    def _log_status(self, frame, valid):
        now = self.get_clock().now().nanoseconds / 1e9
        changed = self._last_valid_state is None or bool(valid) != bool(self._last_valid_state)
        periodic = (now - self._last_status_log_ts) >= self.status_log_period
        if not changed and not periodic:
            return
        self._last_valid_state = bool(valid)
        self._last_status_log_ts = now
        self.log.info(
            "HuskyLens frame status",
            event="STATUS",
            connected=frame.get("connected", 0),
            algorithm_set=frame.get("algorithm_set", 0),
            valid=int(bool(valid)),
            error=frame.get("error", 0),
            y_type=frame.get("y_type", 0),
            line_length_y=frame.get("line_length_y", 0),
            direction=frame.get("direction", 0),
        )

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HuskyLensSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
