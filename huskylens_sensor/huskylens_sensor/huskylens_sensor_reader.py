from robot_common.serial_device import SerialDevice


class HuskyLensSensorReader(SerialDevice):
    def __init__(self, port="/dev/ttyACM2", baudrate=9600, timeout=0.2, logger=None):
        super().__init__(port=port, baudrate=baudrate, timeout=timeout, logger=logger, exclusive=True)

    def read_line(self):
        if not self.is_connected():
            return None
        try:
            data = self.ser.readline()
            if not data:
                return None
            text = data.decode("utf-8", errors="replace").strip()
            if not text:
                return None
            return text
        except Exception as exc:
            self._log_error(f"HuskyLens serial read error: {exc}")
            self.close()
            return None