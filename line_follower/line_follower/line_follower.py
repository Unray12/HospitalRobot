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
        logger=None,
    ):
        self._logger = logger
        self.base_speed = int(base_speed)
        self.crossing_duration = crossing_duration

        self.state = self.STATE_FOLLOWING
        self.crossing_start_time = None

    def reset(self):
        self.state = self.STATE_FOLLOWING
        self.crossing_start_time = None

    def stop(self):
        self.state = self.STATE_STOPPED
        self.crossing_start_time = None

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

        # Follow logic (no PID)
        if mid_full and left_count <= 1 and right_count <= 1:
            self.state = self.STATE_FOLLOWING
            return "Forward", self.base_speed

        if left_count > right_count:
            self.state = self.STATE_TURN_LEFT
            return "RotateLeft", self.base_speed

        if right_count > left_count:
            self.state = self.STATE_TURN_RIGHT
            return "RotateRight", self.base_speed

        if mid_count == 0:
            if left_count > 0:
                self.state = self.STATE_TURN_LEFT
                return "RotateLeft", self.base_speed
            if right_count > 0:
                self.state = self.STATE_TURN_RIGHT
                return "RotateRight", self.base_speed

        self.state = self.STATE_FOLLOWING
        return "Forward", self.base_speed

    def _handle_crossing(self, now):
        if self.crossing_start_time is None:
            self.crossing_start_time = now

        elapsed = now - self.crossing_start_time
        if elapsed < self.crossing_duration:
            return "Forward", self.base_speed

        self.stop()
        self._log_info("===> CROSS done: STOP")
        return "Stop", 0

    def _log_info(self, msg):
        if self._logger:
            self._logger.info(msg)
        else:
            print(msg)
