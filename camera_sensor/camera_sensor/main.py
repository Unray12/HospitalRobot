#!/usr/bin/env python3
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .camera_sensor_reader import CameraSensorReader
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter


class CameraSensorNode(Node):
    def __init__(self):
        super().__init__("camera_sensor")
        self.log = LogAdapter(self.get_logger(), "camera_sensor")

        config = ConfigManager("camera_sensor", logger=self.log).load()
        serial_cfg = config.get("serial", {})
        pub_cfg = config.get("publish", {})

        port = str(serial_cfg.get("port", "/dev/ttyACM0"))
        baudrate = int(serial_cfg.get("baudrate", 115200))
        read_timeout = float(serial_cfg.get("timeout", 0.2))
        topic = str(pub_cfg.get("topic", "/face/camera"))
        rate_hz = float(pub_cfg.get("rate_hz", 30.0))

        self.reconnect_period_sec = float(
            config.get("reconnect_period_sec", serial_cfg.get("reconnect_period_sec", 2.0))
        )
        self.fallback_ports = config.get("fallback_ports", ["/dev/ttyACM1", "/dev/ttyACM0"])
        self.scan_prefixes = config.get("scan_prefixes", ["/dev/ttyACM", "/dev/ttyUSB", "COM"])
        self.malformed_log_every = int(max(1, config.get("malformed_log_every", 20)))
        self._drop_count = 0

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
            self._drop_count += 1
            if self._drop_count % self.malformed_log_every == 1:
                self.log.warning(f"Dropped malformed camera frame: {line}", event="FRAME")
            return

        self.pub.publish(String(data=normalized))

    def _reconnect_cb(self):
        if self.reader.is_connected():
            return
        if self.reader.reconnect(
            fallback_ports=self.fallback_ports,
            scan_prefixes=self.scan_prefixes,
        ):
            self.log.info("Camera serial reconnected", event="SERIAL")

    def _normalize_face_payload(self, raw):
        text = (raw or "").strip()
        if not text:
            return None

        text = text.upper().replace(" ", "")
        text = text.replace("Ư", "")

        if "<" in text and ">" in text:
            start = text.find("<")
            end = text.find(">", start + 1)
            if end > start:
                text = text[start + 1:end]
        text = text.strip("<>")
        text = re.sub(r"[^A-Z0-9_,]", "", text)
        if not text:
            return None

        parts = [p for p in text.split(",") if p]
        if len(parts) == 2:
            m = re.match(
                r"^(DEV1|DEV|DV1|EV1|D1|V1|DV|EV)(FACE|ACE|NO_OBJECT|NOOBJECT|NO_OBECT|NO_BJECT|O_OBJECT)$",
                parts[0],
            )
            if m:
                parts = [m.group(1), m.group(2), parts[1]]
        if len(parts) < 2:
            return None

        dev_raw = parts[0]
        state_raw = parts[1]
        rest = ",".join(parts[2:]) if len(parts) > 2 else ""

        dev = self._normalize_device(dev_raw)
        state = self._normalize_state(state_raw)
        if dev is None or state is None:
            return None

        if state == "NO_OBJECT":
            return f"<{dev},NO_OBJECT>"

        face_id = self._extract_face_id(rest)
        return f"<{dev},FACE,{face_id}>"

    def _normalize_device(self, token):
        t = (token or "").strip()
        if not t:
            return None
        if t.startswith("DEV") and len(t) > 3 and t[3:].isdigit():
            return t
        if t in {"DEV", "DV1", "EV1", "D1", "V1", "DV", "EV", "DEV1"}:
            return "DEV1"
        if any(ch in t for ch in "DEV") and "1" in t:
            return "DEV1"
        return None

    def _normalize_state(self, token):
        t = (token or "").strip()
        if not t:
            return None
        if "FACE" in t or t == "ACE":
            return "FACE"
        if (
            t in {"NO_OBJECT", "NOOBJECT", "NO_OBECT", "NO_BJECT", "O_OBJECT"}
            or ("NO" in t and "OBJECT" in t)
        ):
            return "NO_OBJECT"
        return None

    def _extract_face_id(self, token):
        if not token:
            return 0
        m = re.search(r"\d+", token)
        if not m:
            return 0
        return int(m.group(0))

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
