class LineFollowerFSM:
    STATE_FOLLOWING = 0
    STATE_CROSSING = 1
    STATE_TURN_LEFT = 2
    STATE_TURN_RIGHT = 3
    STATE_BACKWARD = 4
    STATE_PLAN = 5
    STATE_STOPPED = 6

    def __init__(
        self,
        base_speed=6.0,
        turn_speed_left=None,
        turn_speed_right=None,
        crossing_duration=2.0,
        cross_plan=None,
        plan_end_state="stop",
        logger=None,
    ):
        self._logger = logger
        self.base_speed = int(base_speed)
        self.turn_speed_left = int(turn_speed_left) if turn_speed_left is not None else int(base_speed)
        self.turn_speed_right = int(turn_speed_right) if turn_speed_right is not None else int(base_speed)
        self.crossing_duration = crossing_duration
        self.cross_plan = cross_plan or []
        self.plan_end_state = plan_end_state

        self.state = self.STATE_FOLLOWING
        self.crossing_start_time = None
        self._plan_index = 0
        self._plan_step_start = None

    def reset(self):
        self.state = self.STATE_FOLLOWING
        self.crossing_start_time = None
        self._plan_index = 0
        self._plan_step_start = None

    def stop(self):
        self.state = self.STATE_STOPPED
        self.crossing_start_time = None
        self._plan_index = 0
        self._plan_step_start = None

    def set_plan(self, steps, end_state=None):
        self.cross_plan = steps or []
        if end_state:
            self.plan_end_state = end_state
        self._plan_index = 0
        self._plan_step_start = None

    def update(self, frame, now):
        if self.state == self.STATE_STOPPED:
            return "Stop", 0
        if self.state == self.STATE_CROSSING:
            return self._handle_crossing(now)
        if self.state == self.STATE_PLAN:
            return self._handle_plan(now)

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
            if self.cross_plan:
                self.state = self.STATE_PLAN
                self._plan_index = 0
                self._plan_step_start = None
                self._log_info("===> CROSS detected: start plan")
                return self._handle_plan(now)
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
            return "RotateLeft", self.turn_speed_left

        if right_count > left_count:
            self.state = self.STATE_TURN_RIGHT
            return "RotateRight", self.turn_speed_right

        if mid_count == 0:
            if left_count > 0:
                self.state = self.STATE_TURN_LEFT
                return "RotateLeft", self.turn_speed_left
            if right_count > 0:
                self.state = self.STATE_TURN_RIGHT
                return "RotateRight", self.turn_speed_right

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

    def _handle_plan(self, now):
        if not self.cross_plan:
            self.stop()
            return "Stop", 0

        if self._plan_index >= len(self.cross_plan):
            if self.plan_end_state == "follow":
                self.state = self.STATE_FOLLOWING
                return "Forward", self.base_speed
            self.stop()
            return "Stop", 0

        step = self.cross_plan[self._plan_index]
        action = step.get("action", "Stop")
        duration = step.get("duration", 0)
        speed = int(step.get("speed", self.base_speed))

        if self._plan_step_start is None:
            self._plan_step_start = now

        elapsed = now - self._plan_step_start
        if duration and elapsed >= duration:
            self._plan_index += 1
            self._plan_step_start = None
            return self._handle_plan(now)

        # Apply action
        if action == "Forward":
            return "Forward", speed
        if action == "Backward":
            return "Backward", speed
        if action == "RotateLeft":
            return "RotateLeft", speed
        if action == "RotateRight":
            return "RotateRight", speed
        if action == "Left":
            return "Left", speed
        if action == "Right":
            return "Right", speed
        return "Stop", 0

    def _log_info(self, msg):
        if self._logger:
            self._logger.info(msg)
        else:
            print(msg)
