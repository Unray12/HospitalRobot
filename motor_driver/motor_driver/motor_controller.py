import time

try:
    import serial
except ImportError:
    serial = None

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

        if serial is None:
            self._log_error("pyserial is not installed; motor serial is unavailable")
            return

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
            )
            time.sleep(2)
            self._log_info("Serial CAN connected")
        except Exception as exc:
            self._log_error(f"Serial error: {exc}")
            self.ser = None

    def move(self, direction, speed: float = 0):
        if direction not in DIR:
            self._log_error("Invalid direction command")
            return

        wheel_speeds = [s * speed for s in DIR[direction]]
        cmd = ",".join(f"{v}" for v in wheel_speeds) + "\n"
        if self.ser and self.ser.is_open:
            self.ser.write(f"({cmd})".encode())

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

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
