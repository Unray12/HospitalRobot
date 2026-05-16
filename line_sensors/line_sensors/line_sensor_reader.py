"""Serial reader and parser for the line sensor frame stream.

Đọc JSON envelope theo HospitalRobot Device Protocol v1
(xem `docs/DEVICE_PROTOCOL.md`). Frame data:

    {"dev_id":"hrbot_line","event":"data","payload":{"sensors":{"0xNN":{...}}}}
"""

import glob
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from robot_common.device_protocol import parse_envelope, is_silent_error

DEV_ID = "hrbot_line"


class LineSensorReader:
    """Read newline-delimited JSON payloads and normalize them into line frames."""

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=0.1, logger=None):
        self._logger = logger
        self._buffer = ""
        self.ser = None
        self.port = str(port)
        self.baudrate = baudrate
        self.timeout = timeout
        self._last_open_error = None

        if serial is None:
            self._log_error("pyserial is not installed; line sensor serial is unavailable")
            return

        self._open_serial(self.port)

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
            # boot/info/error — bỏ qua thầm lặng.
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
            # Backward-compat: payload chính là dict sensors (test legacy).
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

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def is_connected(self):
        return bool(self.ser and self.ser.is_open)

    def reconnect(self, fallback_ports=None, scan_prefixes=None):
        if self.is_connected() or serial is None:
            return self.is_connected()

        candidates = []
        if self.port:
            candidates.append(self.port)
        for item in (fallback_ports or []):
            if item and item not in candidates:
                candidates.append(item)

        # Empty list = intentional "không scan" (vs None = chưa cấu hình, dùng default).
        if scan_prefixes is None:
            scan_prefixes = ["/dev/ttyACM", "/dev/ttyUSB", "COM"]
        if scan_prefixes:
            for detected in self._discover_ports(scan_prefixes):
                if detected not in candidates:
                    candidates.append(detected)

        for candidate in candidates:
            if self._open_serial(candidate):
                self.port = candidate
                return True
        return False

    def _open_serial(self, port):
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
            time.sleep(min(self.timeout, 0.2))
            self._last_open_error = None
            self._log_info(f"Serial Line Sensors connected: {port}")
            return True
        except Exception as exc:
            self.ser = None
            msg = str(exc)
            if msg != self._last_open_error:
                self._log_error(f"Serial error on {port}: {exc}")
                self._last_open_error = msg
            return False

    def _discover_ports(self, prefixes):
        found = []
        if list_ports is not None:
            for port_info in list_ports.comports():
                name = str(getattr(port_info, "device", "") or "")
                if self._matches_prefix(name, prefixes):
                    found.append(name)
        for prefix in prefixes:
            if prefix.startswith("/dev/"):
                found.extend(glob.glob(prefix + "*"))
        dedup = []
        for port in found:
            if port and port not in dedup:
                dedup.append(port)
        return dedup

    def _matches_prefix(self, name, prefixes):
        upper = name.upper()
        for prefix in prefixes:
            if upper.startswith(prefix.upper()):
                return True
        return False

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
