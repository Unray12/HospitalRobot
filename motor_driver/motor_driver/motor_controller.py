"""Serial motor controller wrapper for mecanum wheel commands."""

from robot_common.serial_device import SerialDevice

# Mecanum kinematics: each tuple is (front-left, front-right, rear-left, rear-right)
# direction sign. Multiplied by scalar speed at runtime. Negative sign reflects
# the motor wiring convention used by the on-board MCU firmware.
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

        # MCU parser expects integer wheel speeds, not floats. Rounding here
        # avoids "8.0" / "8.7" tokens that the firmware tokenizer would reject.
        wheel_speeds = [int(round(s * speed)) for s in DIR[direction]]
        with self._lock:
            if not (self.ser and self.ser.is_open):
                self._log_warn(f"Motor serial disconnected, command dropped: {direction}")
                return None
            frame = "(" + ",".join(str(v) for v in wheel_speeds) + ")\n"
            try:
                self.ser.write(frame.encode())
            except Exception as exc:
                self._log_warn(f"Serial write failed, command dropped: {exc}")
                self.ser = None
                return None
        return wheel_speeds