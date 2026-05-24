"""Base class for serial device readers, eliminating ~300 LOC of copy-paste."""

import glob
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


class SerialDevice:
    def __init__(self, port="", baudrate=115200, timeout=0.1, logger=None,
                 exclusive=False):
        self._logger = logger
        self.ser = None
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self._exclusive = bool(exclusive)
        self._last_open_error = None
        self._lock = threading.Lock()

        if serial is None:
            self._log_error(f"pyserial not installed; {self._device_name()} unavailable")
            return
        self._open_serial(self.port)

    def _device_name(self):
        return self.__class__.__name__

    def close(self):
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None

    def is_connected(self):
        with self._lock:
            return bool(self.ser and self.ser.is_open)

    def reconnect(self, fallback_ports=None, scan_prefixes=None):
        if self.is_connected() or serial is None:
            return self.is_connected()


        # Try candidates in priority order:
        # 1. The original configured port (handles transient disconnects).
        # 2. fallback_ports from config (devices that may have shifted ttyACM#).
        # 3. OS-discovered ports matching scan_prefixes (broad recovery when
        #    udev re-enumeration changes everything).
        candidates = []
        if self.port:
            candidates.append(self.port)
        for item in (fallback_ports or []):
            if item and item not in candidates:
                candidates.append(item)

        if scan_prefixes is None:
            scan_prefixes = ["/dev/ttyUSB", "/dev/ttyACM", "COM"]
        if scan_prefixes:
            for detected in self._discover_ports(scan_prefixes):
                if detected not in candidates:
                    candidates.append(detected)

        for candidate in candidates:
            if self._open_serial(candidate):
                # Remember the working port so the next reconnect tries it first
                # instead of re-scanning.
                self.port = candidate
                return True
        return False

    def _open_serial(self, port):
        with self._lock:
            try:
                kwargs = {
                    "port": port,
                    "baudrate": self.baudrate,
                    "timeout": self.timeout,
                }
                if self._exclusive:
                    # exclusive=True prevents another process from opening the same
                    # port simultaneously. Required for HuskyLens/camera readers
                    # because pyserial on Linux otherwise allows shared access and
                    # the second reader steals bytes mid-frame.
                    try:
                        self.ser = serial.Serial(exclusive=True, **kwargs)
                    except TypeError:
                        # Older pyserial without exclusive= kwarg.
                        self.ser = serial.Serial(**kwargs)
                else:
                    self.ser = serial.Serial(**kwargs)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                # Small grace for USB-CDC enumeration; full timeout would stack up
                # across multiple candidate ports during reconnect.
                time.sleep(0.05)
                self._last_open_error = None
                self._log_info(f"Serial connected: {port}")
                return True
            except Exception as exc:
                self.ser = None
                msg = str(exc)
                # Throttle identical error messages — when a port is permanently
                # missing, the reconnect timer (2s period) would otherwise log the
                # same error every 2s forever.
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