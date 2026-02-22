from .PID import PID


class LineFollowerFSM:
    STATE_FOLLOWING = 0
    STATE_CROSSING = 1
    STATE_TURN_LEFT = 2
    STATE_TURN_RIGHT = 3
    STATE_STOPPED = 4

    def __init__(
        self,
        base_speed=6.0,
        crossing_duration=2.0,
        pid_kp=6.0,
        pid_ki=0.0,
        pid_kd=0.0,
        pid_deadband=0.15,
        min_turn_speed=2.0,
        max_turn_speed=None,
        logger=None,
    ):
        self._logger = logger
        self.base_speed = base_speed
        self.crossing_duration = crossing_duration
        self.pid_deadband = pid_deadband
        self.min_turn_speed = min_turn_speed
        self.max_turn_speed = max_turn_speed if max_turn_speed is not None else base_speed

        self.pid = PID(
            kp=pid_kp,
            ki=pid_ki,
            kd=pid_kd,
            setpoint=0.0,
            output_limits=(-self.max_turn_speed, self.max_turn_speed),
        )

        self.state = self.STATE_FOLLOWING
        self.crossing_start_time = None

    def reset(self):
        self.state = self.STATE_FOLLOWING
        self.crossing_start_time = None
        self.pid.reset()

    def stop(self):
        self.state = self.STATE_STOPPED
        self.crossing_start_time = None
        self.pid.reset()

    def update(self, frame, now):
        if self.state == self.STATE_STOPPED:
            return "Stop", 0
        if self.state == self.STATE_CROSSING:
            return self._handle_crossing(now)

        if frame is None:
            return None

        left_count = frame["left_count"]
        mid_count = frame["mid_count"]
        right_count = frame["right_count"]
        left_full = frame["left_full"]
        mid_full = frame["mid_full"]
        right_full = frame["right_full"]

        total_black = left_count + mid_count + right_count

        # Cross detected
        if left_full and mid_full and right_full:
            self.state = self.STATE_CROSSING
            self.crossing_start_time = now
            self._log_info("===> CROSS detected: move forward 2s")
            return "Forward", self.base_speed

        # Lost line -> stop
        if total_black == 0:
            self.stop()
            self._log_info("===> LOST LINE: STOP")
            return "Stop", 0

        position = self._compute_line_position(left_count, right_count, total_black)
        output = self.pid.update(position, now)

        if abs(output) < self.pid_deadband:
            self.state = self.STATE_FOLLOWING
            return "Forward", self.base_speed

        speed = self._clamp(abs(output), self.min_turn_speed, self.max_turn_speed)
        if output > 0:
            self.state = self.STATE_TURN_LEFT
            return "RotateLeft", speed
        self.state = self.STATE_TURN_RIGHT
        return "RotateRight", speed

    def _handle_crossing(self, now):
        if self.crossing_start_time is None:
            self.crossing_start_time = now

        elapsed = now - self.crossing_start_time
        if elapsed < self.crossing_duration:
            return "Forward", self.base_speed

        self.stop()
        self._log_info("===> CROSS done: STOP")
        return "Stop", 0

    def _compute_line_position(self, left_count, right_count, total_black):
        # Positive position => line is to the right.
        return (right_count - left_count) / total_black

    def _clamp(self, value, min_value, max_value):
        if value < min_value:
            return min_value
        if value > max_value:
            return max_value
        return value

    def _log_info(self, msg):
        if self._logger:
            self._logger.info(msg)
        else:
            print(msg)
