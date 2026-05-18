"""Serial reader and parser for the line sensor frame stream.

Đọc JSON envelope theo HospitalRobot Device Protocol v1
(xem `docs/DEVICE_PROTOCOL.md`). Frame data:

    {"dev_id":"hrbot_line","event":"data","payload":{"sensors":{"0xNN":{...}}}}
"""

from robot_common.serial_device import SerialDevice
from robot_common.device_protocol import parse_envelope, is_silent_error

DEV_ID = "hrbot_line"


class LineSensorReader(SerialDevice):
    """Read newline-delimited JSON payloads and normalize them into line frames."""

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=0.1, logger=None):
        self._buffer = ""
        super().__init__(port=port, baudrate=baudrate, timeout=timeout, logger=logger)

    def read_frame(self):
        """Return the newest complete frame available from the serial buffer."""
        if not (self.ser and self.ser.is_open):
            return None

        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                return None

            self._buffer += data.decode(errors="ignore")
            if "\n" not in self._buffer:
                return None

            lines = self._buffer.split("\n")
            self._buffer = lines[-1]
            for line in reversed(lines[:-1]):
                candidate = line.strip()
                if not candidate:
                    continue
                return self.parse_line(candidate)
            return None
        except Exception as exc:
            self._log_error(f"Line Sensors read error: {exc}")
            self.close()
            return None

    def parse_line(self, line):
        """Parse 1 JSON envelope line. Trả về frame dict, None, hoặc None (skip thầm lặng)."""
        envelope, err = parse_envelope(line, expected_dev_id=DEV_ID)
        if envelope is None:
            if is_silent_error(err):
                return None
            self._log_warn(f"Line envelope parse fail: {err}")
            return None
        if not envelope.is_data():
            return None
        return self.parse_payload(envelope.payload)

    def parse_payload(self, payload):
        """Convert payload dict thành frame có count + full cho left/mid/right.

        Hỗ trợ 2 shape:
          - mới: {"sensors": {"0xNN": {...}}}
          - cũ:  {"0xNN": {...}}  (dùng cho test trực tiếp parse_payload)
        """
        if not isinstance(payload, dict):
            return None

        sensors = payload.get("sensors")
        if not isinstance(sensors, dict):
            sensors = payload

        if not sensors:
            return None

        left = sensors.get("0x25", {})
        middle = sensors.get("0x24", {})
        right = sensors.get("0x23", {})

        return {
            "left_count":  self._count_active(left),
            "mid_count":   self._count_active(middle),
            "right_count": self._count_active(right),
            "left_full":   self._is_full_black(left),
            "mid_full":    self._is_full_black(middle),
            "right_full":  self._is_full_black(right),
        }

    def _count_active(self, sensor_dict):
        """Count active channels without trusting payload types from the serial device."""
        if not isinstance(sensor_dict, dict):
            return 0
        return sum(1 for value in sensor_dict.values() if self._is_active(value))

    def _is_full_black(self, sensor_dict):
        if not isinstance(sensor_dict, dict) or not sensor_dict:
            return False
        return all(self._is_active(value) for value in sensor_dict.values())

    def _is_active(self, value):
        return value in (True, 1, "1")