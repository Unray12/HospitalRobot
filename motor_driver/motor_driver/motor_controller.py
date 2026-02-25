import time
import glob

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

# Direction map for mecanum wheels
# Order: [FL, FR, RL, RR]
DIR = {
    "Forward": (-1, -1, -1, -1),
    "Backward": (+1, +1, +1, +1),
    "Right": (-1, +1, -1, +1),
    "Left": (+1, -1, +1, -1),
    "RotateRight": (-1, -1, +1, +1),
    "RotateLeft": (+1, +1, -1, -1),
    "Stop": (0, 0, 0, 0),
}


class MotorController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.1, logger=None):
        self._logger = logger
        self.ser = None
        self.port = str(port)
        self.baudrate = baudrate
        self.timeout = timeout
        self._last_open_error = None

        if serial is None:
            self._log_error("pyserial is not installed; motor serial is unavailable")
            return

        self._open_serial(self.port)

    def move(self, direction, speed: float = 0):
        if direction not in DIR:
            self._log_error("Invalid direction command")
            return None

        wheel_speeds = [s * speed for s in DIR[direction]]
        cmd = ",".join(f"{v}" for v in wheel_speeds) + "\n"
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(f"({cmd})".encode())
            except Exception as exc:
                self._log_error(f"Serial write error: {exc}")
                self.close()
        return wheel_speeds

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

        prefixes = scan_prefixes or ["/dev/ttyUSB", "/dev/ttyACM", "COM"]
        for detected in self._discover_ports(prefixes):
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
            time.sleep(2)
            self._last_open_error = None
            self._log_info(f"Serial CAN connected: {port}")
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
            for p in list_ports.comports():
                name = str(getattr(p, "device", "") or "")
                if self._matches_prefix(name, prefixes):
                    found.append(name)
        for prefix in prefixes:
            if prefix.startswith("/dev/"):
                found.extend(glob.glob(prefix + "*"))
        dedup = []
        for p in found:
            if p and p not in dedup:
                dedup.append(p)
        return dedup

    def _matches_prefix(self, name, prefixes):
        upper = name.upper()
        for prefix in prefixes:
            if upper.startswith(prefix.upper()):
                return True
        return False

    def _log_info(self, msg):
        if self._logger:
            self._logger.info(msg)
        else:
            print(msg)

    def _log_error(self, msg):
        if self._logger:
            self._logger.error(msg)
        else:
            print(msg)
