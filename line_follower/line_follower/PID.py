class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, setpoint=0.0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.min_output, self.max_output = output_limits

        self._integral = 0.0
        self._last_error = None
        self._last_time = None

    def reset(self):
        self._integral = 0.0
        self._last_error = None
        self._last_time = None

    def update(self, measurement, current_time):
        error = self.setpoint - measurement

        if self._last_time is None:
            dt = 0.0
        else:
            dt = current_time - self._last_time

        # Proportional
        p = self.kp * error

        # Integral
        if dt > 0.0:
            self._integral += error * dt
        i = self.ki * self._integral

        # Derivative
        if self._last_error is None or dt == 0.0:
            d = 0.0
        else:
            d = self.kd * (error - self._last_error) / dt

        output = p + i + d

        # Clamp output
        if self.min_output is not None:
            output = max(self.min_output, output)
        if self.max_output is not None:
            output = min(self.max_output, output)

        self._last_error = error
        self._last_time = current_time

        return output
