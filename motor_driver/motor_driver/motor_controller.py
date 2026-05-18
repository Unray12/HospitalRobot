"""Serial motor controller wrapper for mecanum wheel commands."""

from robot_common.serial_device import SerialDevice

DIR = {
    "Forward": (-1, -1, -1, -1),
    "Backward": (+1, +1, +1, +1),
    "Right": (-1, +1, -1, +1),
    "Left": (+1, -1, +1, -1),
    "RotateRight": (-1, -1, +1, +1),
    "RotateLeft": (+1, +1, -1, -1),
    "Stop": (0, 0, 0, 0),
}


class MotorController(SerialDevice):
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.1, logger=None):
        super().__init__(port=port, baudrate=baudrate, timeout=timeout, logger=logger)

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