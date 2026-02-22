import time
import json

try:
    import serial
except ImportError:
    serial = None


class LineSensorReader:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=0.1, logger=None):
        self._logger = logger
        self._buffer = ""
        self.ser = None

        if serial is None:
            self._log_error("pyserial is not installed; line sensor serial is unavailable")
            return

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
            )
            time.sleep(2)
            self._log_info("Serial Line Sensors connected")
        except Exception as exc:
            self._log_error(f"Serial error: {exc}")
            self.ser = None

    def read_frame(self):
        if not (self.ser and self.ser.is_open):
            return None

        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                return None

            self._buffer += data.decode(errors="ignore")
            if "\n" not in self._buffer:
                return None

            line, self._buffer = self._buffer.split("\n", 1)
            line = line.strip()
            if not line:
                return None

            return self.parse_line(line)
        except json.JSONDecodeError:
            self._log_warn("Corrupted JSON skipped")
            return None
        except Exception as exc:
            self._log_error(f"Line Sensors read error: {exc}")
            return None

    def parse_line(self, line):
        payload = json.loads(line)
        return self.parse_payload(payload)

    def parse_payload(self, payload):
        if "LineSensor" not in payload:
            self._log_warn("LineSensor key missing")
            return None

        line_data = payload["LineSensor"]
        if not isinstance(line_data, dict) or not line_data:
            return None

        left = line_data.get("0x25", {})
        middle = line_data.get("0x24", {})
        right = line_data.get("0x23", {})

        left_count = self._count_active(left)
        mid_count = self._count_active(middle)
        right_count = self._count_active(right)

        return {
            "left_count": left_count,
            "mid_count": mid_count,
            "right_count": right_count,
            "left_full": self._is_full_black(left),
            "mid_full": self._is_full_black(middle),
            "right_full": self._is_full_black(right),
        }

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _count_active(self, sensor_dict):
        if not isinstance(sensor_dict, dict):
            return 0
        return sum(sensor_dict.values())

    def _is_full_black(self, sensor_dict):
        if not isinstance(sensor_dict, dict) or not sensor_dict:
            return False
        return all(v == 1 for v in sensor_dict.values())

    def _log_info(self, msg):
        if self._logger:
            self._logger.info(msg)
        else:
            print(msg)

    def _log_warn(self, msg):
        if self._logger:
            self._logger.warning(msg)
        else:
            print(msg)

    def _log_error(self, msg):
        if self._logger:
            self._logger.error(msg)
        else:
            print(msg)
