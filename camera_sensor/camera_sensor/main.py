#!/usr/bin/env python3
"""ROS2 node parse envelope camera (Device Protocol v1) -> <DEV1,FACE,...> string.

Firmware schema (xem `docs/DEVICE_PROTOCOL.md`):
  {"dev_id":"hrbot_camera","event":"boot","payload":{"fw":"camera","ver":1}}
  {"dev_id":"hrbot_camera","event":"data","payload":{"kind":"face","ids":[1,2]}}
  {"dev_id":"hrbot_camera","event":"data","payload":{"kind":"no_object"}}
  {"dev_id":"hrbot_camera","event":"info","payload":{"msg":"..."}}
  {"dev_id":"hrbot_camera","event":"error","payload":{"code":"..."}}

Downstream (mqtt_bridge, speaker) dùng String "<DEV1,FACE,1>" qua /face/camera, node
này dịch envelope -> string để consumer không phải đổi.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .camera_sensor_reader import CameraSensorReader
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter
from robot_common.device_protocol import parse_envelope, is_silent_error

DEV_ID = "hrbot_camera"
DOWNSTREAM_DEV = "DEV1"


class CameraSensorNode(Node):
    """Publish normalized face/no-object events from the camera serial reader."""
    def __init__(self):
        super().__init__("camera_sensor")
        self.log = LogAdapter(self.get_logger(), "camera_sensor")

        config = ConfigManager("camera_sensor", logger=self.log).load()
        serial_cfg = config.get("serial", {})
        pub_cfg = config.get("publish", {})

        port = str(serial_cfg.get("port", "/dev/ttyACM0"))
        baudrate = int(serial_cfg.get("baudrate", 9600))
        read_timeout = float(serial_cfg.get("timeout", 0.2))
        topic = str(pub_cfg.get("topic", "/face/camera"))
        rate_hz = float(pub_cfg.get("rate_hz", 30.0))

        self.reconnect_period_sec = float(
            config.get("reconnect_period_sec", serial_cfg.get("reconnect_period_sec", 2.0))
        )
        self.fallback_ports = config.get("fallback_ports", ["/dev/ttyACM1", "/dev/ttyACM0"])
        self.scan_prefixes = config.get("scan_prefixes", ["/dev/ttyACM", "/dev/ttyUSB", "COM"])
        self.malformed_log_every = int(max(1, config.get("malformed_log_every", 20)))
        self.status_log_period = float(max(0.1, config.get("status_log_period", 2.0)))
        self._drop_count = 0
        self._last_kind = None
        self._last_ids = None
        self._frames_since_log = 0
        self._last_status_log_ts = 0.0

        self.reader = CameraSensorReader(
            port=port,
            baudrate=baudrate,
            timeout=read_timeout,
            logger=self.log,
        )

        self.pub = self.create_publisher(String, topic, 10)
        period = 1.0 / max(rate_hz, 1.0)
        self.timer = self.create_timer(period, self._timer_cb)
        self.reconnect_timer = self.create_timer(self.reconnect_period_sec, self._reconnect_cb)
        self.log.info(
            f"Reading camera serial from {port} @ {baudrate} -> topic '{topic}'",
            event="BOOT",
        )

    def _timer_cb(self):
        line = self.reader.read_line()
        if line is None:
            return

        normalized = self._normalize_face_payload(line)
        if normalized is None:
            return  # parse_envelope đã chia silent skip vs warning

        self.pub.publish(String(data=normalized))
        self._frames_since_log += 1
        self._log_status(normalized)

    def _log_status(self, normalized):
        # Parse "<DEV1,FACE,1,2>" hoặc "<DEV1,NO_OBJECT>" cho status.
        inner = normalized.strip("<>")
        parts = inner.split(",") if inner else []
        kind = parts[1] if len(parts) >= 2 else "?"
        ids = parts[2:] if len(parts) > 2 else []

        now = self.get_clock().now().nanoseconds / 1e9
        changed = (kind != self._last_kind) or (ids != self._last_ids)
        periodic = (now - self._last_status_log_ts) >= self.status_log_period
        if not changed and not periodic:
            return
        self._last_kind = kind
        self._last_ids = ids
        self._last_status_log_ts = now

        self.log.info(
            "Camera frame status",
            event="STATUS",
            kind=kind,
            ids=",".join(ids) if ids else "-",
            frames=self._frames_since_log,
        )
        self._frames_since_log = 0

    def _reconnect_cb(self):
        if self.reader.is_connected():
            return
        if self.reader.reconnect(
            fallback_ports=self.fallback_ports,
            scan_prefixes=self.scan_prefixes,
        ):
            self.log.info("Camera serial reconnected", event="SERIAL")

    def _normalize_face_payload(self, raw):
        """Parse 1 envelope -> string downstream hoặc None (skip silently)."""
        envelope, err = parse_envelope(raw, expected_dev_id=DEV_ID)
        if envelope is None:
            if is_silent_error(err):
                return None
            self._drop_count += 1
            if self._drop_count % self.malformed_log_every == 1:
                self.log.warning(f"Dropped malformed camera frame ({err}): {raw}", event="FRAME")
            return None
        if not envelope.is_data():
            # boot/info/error — bỏ qua thầm lặng.
            return None

        kind = envelope.payload.get("kind")
        if kind == "face":
            ids_raw = envelope.payload.get("ids") or []
            if not isinstance(ids_raw, list):
                return None
            ids = [int(i) for i in ids_raw if isinstance(i, (int, float))]
            ids_str = ",".join(str(i) for i in ids) if ids else "0"
            return f"<{DOWNSTREAM_DEV},FACE,{ids_str}>"
        if kind == "no_object":
            return f"<{DOWNSTREAM_DEV},NO_OBJECT>"
        return None

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraSensorNode()
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
