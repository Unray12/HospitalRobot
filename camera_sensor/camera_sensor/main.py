#!/usr/bin/env python3
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter


class SerialReader(Node):
    def __init__(self):
        super().__init__('camera_sensor')
        self.log = LogAdapter(self.get_logger(), "camera_sensor")

        config = ConfigManager("camera_sensor", logger=self.log).load()
        serial_cfg = config.get("serial", {})
        pub_cfg = config.get("publish", {})

        self.port = str(serial_cfg.get("port", "/dev/ttyACM0"))
        self.baud = int(serial_cfg.get("baudrate", 115200))
        self.topic = str(pub_cfg.get("topic", "/face/camera"))
        self.read_timeout = float(serial_cfg.get("timeout", 0.2))

        self.pub = self.create_publisher(String, self.topic, 10)
        self._drop_count = 0

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)

        self.ser = None
        self._open_serial()

        self._thread.start()
        self.log.info(
            f"Reading serial from {self.port} @ {self.baud} -> topic '{self.topic}'",
            event="BOOT",
        )

    def _open_serial(self):
        try:
            # exclusive=True giúp tránh 2 process mở cùng lúc (pyserial>=3.5)
            self.ser = serial.Serial(
                port=self.port,
                baudrate=int(self.baud),
                timeout=self.read_timeout,
                exclusive=True
            )
            # Optional: flush
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            # self._log_info(f"Serial Line Sensors connected: {self.port}")
        except Exception as e:
            self.log.error(f"Cannot open serial {self.port}: {e}", event="SERIAL")
            raise

    def _read_loop(self):
        while rclpy.ok() and not self._stop.is_set():
            try:
                if self.ser is None or not self.ser.is_open:
                    time.sleep(0.2)
                    continue

                line = self.ser.readline()  # bytes, ends with \n if present
                if not line:
                    continue

                text = line.decode('utf-8', errors='replace').strip()
                if text == '':
                    continue

                normalized = self._normalize_face_payload(text)
                if normalized is None:
                    self._drop_count += 1
                    if self._drop_count % 20 == 1:
                        self.log.warning(
                            f"Dropped malformed camera frame: {text}",
                            event="FRAME",
                        )
                    continue

                msg = String()
                msg.data = normalized
                self.pub.publish(msg)

            except serial.SerialException as e:
                self.log.error(f"SerialException: {e}", event="SERIAL")
                time.sleep(0.5)
            except Exception as e:
                self.log.error(f"Read error: {e}", event="SERIAL")
                time.sleep(0.2)

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

        score = self._extract_score(rest)
        return f"<{dev},FACE,{score}>"

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

    def _extract_score(self, token):
        if not token:
            return 0
        m = re.search(r"[01]", token)
        if not m:
            return 0
        return int(m.group(0))

    def destroy_node(self):
        self._stop.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
