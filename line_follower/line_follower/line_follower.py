import time

class LineFollowerFSM:
    STATE_FOLLOWING = 0
    STATE_CROSSING = 1
    STATE_TURN_LEFT = 2
    STATE_TURN_RIGHT = 3
    STATE_BACKWARD = 4
    STATE_PLAN = 5
    STATE_CROSS_PRE = 6
    STATE_STOPPED = 7

    def __init__(
        self,
        base_speed=6.0,
        turn_speed_left=None,
        turn_speed_right=None,
        crossing_duration=2.0,
        cross_plan=None,
        plan_end_state="stop",
        cross_pre_forward_speed=8,
        cross_pre_forward_duration=2.0,
        cross_pre_stop_duration=1.0,
        rotate_min_duration=0.5,
        rotate_line_mid_min_count=1,
        rotate_line_side_max_count=2,
        rotate_early_stop_on_side=True,
        rotate_line_side_min_count=1,
        plan_lost_line_hold_sec=0.8,
        plan_lost_line_warn_period=0.5,
        logger=None,
    ):
        self._logger = logger
        self.base_speed = int(base_speed)
        self.turn_speed_left = int(turn_speed_left) if turn_speed_left is not None else int(base_speed)
        self.turn_speed_right = int(turn_speed_right) if turn_speed_right is not None else int(base_speed)
        self.crossing_duration = crossing_duration
        self.cross_plan = cross_plan or []
        self.plan_end_state = plan_end_state
        self.cross_pre_forward_speed = int(cross_pre_forward_speed)
        self.cross_pre_forward_duration = cross_pre_forward_duration
        self.cross_pre_stop_duration = cross_pre_stop_duration
        self.rotate_min_duration = rotate_min_duration
        self.rotate_line_mid_min_count = int(max(1, rotate_line_mid_min_count))
        self.rotate_line_side_max_count = int(max(0, rotate_line_side_max_count))
        self.rotate_early_stop_on_side = bool(rotate_early_stop_on_side)
        self.rotate_line_side_min_count = int(max(1, rotate_line_side_min_count))
        self.plan_lost_line_hold_sec = float(max(0.0, plan_lost_line_hold_sec))
        self.plan_lost_line_warn_period = float(max(0.1, plan_lost_line_warn_period))

        self.state = self.STATE_FOLLOWING
        self.crossing_start_time = None
        self._plan_index = 0
        self._plan_step_start = None
        self._plan_action_until = None
        self._plan_action = None
        self._plan_action_speed = None
        self._plan_action_until_line = False
        self._plan_action_min_until = None
        self._plan_action_timeout = None
        self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
        self._plan_continue_immediate = False
        self._plan_labels = self._rebuild_plan_labels(self.cross_plan)
        self._cross_active = False
        self._plan_new_step = True
        self._cross_pre_phase = 0
        self._cross_pre_until = None
        self._requested_autoline = None
        self._requested_step_messages = None
        self._plan_start_requested = False
        self._autoline_mode = False
        self._plan_lost_line_since = None
        self._last_plan_lost_line_warn_ts = 0.0

    def reset(self):
        self.state = self.STATE_FOLLOWING
        self.crossing_start_time = None
        self._plan_index = 0
        self._plan_step_start = None
        self._plan_action_until = None
        self._plan_action = None
        self._plan_action_speed = None
        self._plan_action_until_line = False
        self._plan_action_min_until = None
        self._plan_action_timeout = None
        self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
        self._plan_continue_immediate = False
        self._cross_active = False
        self._plan_new_step = True
        self._cross_pre_phase = 0
        self._cross_pre_until = None
        self._requested_autoline = None
        self._requested_step_messages = None
        self._plan_start_requested = False
        self._autoline_mode = False
        self._plan_lost_line_since = None
        self._last_plan_lost_line_warn_ts = 0.0

    def stop(self):
        # Preserve current plan progress when stopping so status does not jump to step 1/N.
        self._enter_stopped(reset_plan_progress=False)

    def _enter_stopped(self, reset_plan_progress=True):
        self.state = self.STATE_STOPPED
        self.crossing_start_time = None
        if reset_plan_progress:
            self._plan_index = 0
        self._plan_step_start = None
        self._plan_action_until = None
        self._plan_action = None
        self._plan_action_speed = None
        self._plan_action_until_line = False
        self._plan_action_min_until = None
        self._plan_action_timeout = None
        self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
        self._plan_continue_immediate = False
        self._cross_active = False
        self._plan_new_step = True
        self._cross_pre_phase = 0
        self._cross_pre_until = None
        self._requested_autoline = None
        self._requested_step_messages = None
        self._plan_start_requested = False
        self._autoline_mode = False
        self._plan_lost_line_since = None
        self._last_plan_lost_line_warn_ts = 0.0

    def set_plan(self, steps, end_state=None):
        self.cross_plan = steps or []
        if end_state:
            self.plan_end_state = end_state
        self._plan_labels = self._rebuild_plan_labels(self.cross_plan)
        self._plan_index = 0
        self._plan_step_start = None
        self._plan_action_until = None
        self._plan_action = None
        self._plan_action_speed = None
        self._plan_action_until_line = False
        self._plan_action_min_until = None
        self._plan_action_timeout = None
        self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
        self._plan_continue_immediate = False
        self._plan_new_step = True
        self._cross_active = False
        self.state = self.STATE_FOLLOWING
        self._cross_pre_phase = 0
        self._cross_pre_until = None
        self._requested_autoline = None
        self._requested_step_messages = None
        self._plan_start_requested = False
        self._autoline_mode = False
        self._plan_lost_line_since = None
        self._last_plan_lost_line_warn_ts = 0.0

    def clear_plan(self):
        self.cross_plan = []
        self.plan_end_state = "stop"
        self._plan_labels = {}
        self._plan_index = 0
        self._plan_step_start = None
        self._plan_action_until = None
        self._plan_action = None
        self._plan_action_speed = None
        self._plan_action_until_line = False
        self._plan_action_min_until = None
        self._plan_action_timeout = None
        self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
        self._plan_continue_immediate = False
        self._cross_active = False
        self.state = self.STATE_FOLLOWING
        self._cross_pre_phase = 0
        self._cross_pre_until = None
        self._requested_autoline = None
        self._requested_step_messages = None
        self._plan_start_requested = False
        self._autoline_mode = False
        self._plan_lost_line_since = None
        self._last_plan_lost_line_warn_ts = 0.0

    def update(self, frame, now):
        if self.state == self.STATE_STOPPED:
            return "Stop", 0
        if self.state == self.STATE_CROSSING:
            return self._handle_crossing(frame, now)
        if self.state == self.STATE_PLAN:
            return self._run_plan_action(frame, now)
        if self.state == self.STATE_CROSS_PRE:
            return self._handle_cross_pre(now)
        if self._plan_start_requested and self.cross_plan:
            self._plan_start_requested = False
            self._cross_active = False
            if self._should_skip_cross_pre():
                self._log_info("===> PLAN trigger: start without cross (skip cross_pre)")
                return self._start_plan_action(now)
            self.state = self.STATE_CROSS_PRE
            self._cross_pre_phase = 0
            self._cross_pre_until = now + self.cross_pre_forward_duration
            self._log_info("===> PLAN trigger: start without cross")
            return "Forward", self.cross_pre_forward_speed

        if frame is None:
            return None

        left_count = frame["left_count"]
        mid_count = frame["mid_count"]
        right_count = frame["right_count"]
        left_full = frame["left_full"]
        mid_full = frame["mid_full"]
        right_full = frame["right_full"]

        total_black = left_count + mid_count + right_count
        cross_detected = left_full and mid_full and right_full
        cross_event = cross_detected and not self._cross_active
        self._cross_active = cross_detected
        if total_black > 0:
            self._plan_lost_line_since = None

        # Cross detected
        if cross_detected:
            if self.cross_plan:
                if cross_event:
                    if self._should_skip_cross_pre():
                        return self._start_plan_action(now)
                    self.state = self.STATE_CROSS_PRE
                    self._cross_pre_phase = 0
                    self._cross_pre_until = now + self.cross_pre_forward_duration
                    return "Forward", self.cross_pre_forward_speed
                return self._follow_line(frame)
            self.state = self.STATE_CROSSING
            self.crossing_start_time = now
            self._log_info("===> CROSS detected: move forward 2s")
            return "Forward", self.base_speed

        # Lost line -> stop
        if total_black == 0:
            if self._has_pending_plan():
                self._hold_stop_for_plan_lost_line(now)
                self.state = self.STATE_FOLLOWING
                return "Stop", 0
            self.stop()
            self._log_info("===> LOST LINE: STOP")
            return "Stop", 0

        return self._follow_line(frame)

    def _handle_crossing(self, frame, now):
        if self.crossing_start_time is None:
            self.crossing_start_time = now

        elapsed = now - self.crossing_start_time
        if elapsed < self.crossing_duration:
            return "Forward", self.base_speed

        # After crossing duration: if line detected -> follow, else stop
        if frame is not None:
            total_black = frame["left_count"] + frame["mid_count"] + frame["right_count"]
            if total_black > 0:
                self.state = self.STATE_FOLLOWING
                return self._follow_line(frame)

        self.stop()
        self._log_info("===> CROSS done: STOP (no line)")
        return "Stop", 0

    def _handle_cross_pre(self, now):
        if self._cross_pre_until is None:
            self._cross_pre_phase = 0
            self._cross_pre_until = now + self.cross_pre_forward_duration
            return "Forward", self.cross_pre_forward_speed

        if self._cross_pre_phase == 0:
            if now < self._cross_pre_until:
                return "Forward", self.cross_pre_forward_speed
            self._cross_pre_phase = 1
            self._cross_pre_until = now + self.cross_pre_stop_duration
            return "Stop", 0

        if self._cross_pre_phase == 1:
            if now < self._cross_pre_until:
                return "Stop", 0
            # After stop, execute plan action
            self.state = self.STATE_FOLLOWING
            self._cross_pre_phase = 0
            self._cross_pre_until = None
            return self._start_plan_action(now)

    def _start_plan_action(self, now):
        if not self.cross_plan:
            return self._follow_default()

        jump_guard = 0
        while jump_guard < max(4, len(self.cross_plan) * 2):
            jump_guard += 1
            if self._plan_index >= len(self.cross_plan):
                if self.plan_end_state == "follow":
                    return self._follow_default()
                # Keep plan progress at end so status remains completed (not step 1/N).
                self._enter_stopped(reset_plan_progress=False)
                return "Stop", 0

            step = self.cross_plan[self._plan_index]
            self._plan_index += 1
            self._plan_new_step = False
            if not isinstance(step, dict):
                self._log_warn(f"Invalid plan step type at index {self._plan_index - 1}: {type(step).__name__}")
                continue

            action = self._normalize_action(step.get("action", "Stop"))
            speed = self._to_int(step.get("speed"), self.base_speed, "speed", step)
            duration = self._to_float(step.get("duration"), 0.0, "duration", step)
            until = str(step.get("until", "") or "").strip().lower()
            timeout = self._to_float(step.get("timeout"), None, "timeout", step)
            continue_immediately = self._to_bool(
                step.get("continue_immediately", step.get("no_wait_cross")),
                False,
            )

            if action in ("LABEL",):
                continue

            if action == "GOTO":
                target = self._resolve_goto_target(step.get("target"))
                if target is None:
                    self._log_warn(f"Invalid Goto target: {step.get('target')}")
                    continue
                self._log_info(f"[PLAN] Goto -> step {target + 1}")
                self._plan_index = target
                continue

            if action == "AUTOLINE":
                enabled = self._to_bool(
                    step.get("enabled", step.get("value", step.get("autoline"))),
                    True,
                )
                self._requested_autoline = enabled
                self._queue_step_messages(step)
                self._plan_continue_immediate = continue_immediately
                self._log_plan_step(self._plan_index, step, f"autoline-{enabled}")
                # After AutoLine step, return to FOLLOWING and wait next cross event
                # before executing the next plan step.
                return self._follow_default()

            if action in ("AUTO",):
                return self._follow_default()

            if action in ("WAIT", "STOP"):
                if duration and duration > 0:
                    self.state = self.STATE_PLAN
                    self._plan_action = "Stop"
                    self._plan_action_speed = 0
                    self._queue_step_messages(step)
                    self._plan_continue_immediate = continue_immediately
                    self._plan_action_until = now + duration
                    self._plan_action_until_line = False
                    self._plan_action_min_until = None
                    self._plan_action_timeout = None
                    self._log_plan_step(self._plan_index, step, "timed-stop")
                    return "Stop", 0
                if action == "WAIT":
                    duration = 0.3
                    self.state = self.STATE_PLAN
                    self._plan_action = "Stop"
                    self._plan_action_speed = 0
                    self._queue_step_messages(step)
                    self._plan_continue_immediate = continue_immediately
                    self._plan_action_until = now + duration
                    self._plan_action_until_line = False
                    self._plan_action_min_until = None
                    self._plan_action_timeout = None
                    self._log_plan_step(self._plan_index, step, "wait-default")
                    return "Stop", 0
                if self.plan_end_state == "follow":
                    self._log_info("[PLAN] Stop step reached, returning to follow")
                    return self._follow_default()
                self._enter_stopped(reset_plan_progress=False)
                self._log_info("[PLAN] Stop step reached, robot stopped")
                return "Stop", 0

            if action == "FOLLOW":
                if duration <= 0:
                    duration = 0.6
                self.state = self.STATE_PLAN
                self._plan_action = "AutoFollow"
                self._plan_action_speed = self.base_speed
                self._queue_step_messages(step)
                self._plan_continue_immediate = continue_immediately
                self._plan_action_until = now + duration
                self._plan_action_until_line = False
                self._plan_action_min_until = None
                self._plan_action_timeout = None
                self._log_plan_step(self._plan_index, step, "follow")
                return "Forward", self.base_speed

            if action in ("ROTATELEFT", "ROTATERIGHT"):
                move_action = "RotateLeft" if action == "ROTATELEFT" else "RotateRight"
                strict_line = self._to_bool(
                    step.get("strict_line", step.get("center_only")),
                    False,
                )
                min_duration = self._to_float(
                    step.get("min_duration"),
                    self.rotate_min_duration,
                    "min_duration",
                    step,
                )
                self.state = self.STATE_PLAN
                self._plan_action = move_action
                self._plan_action_speed = speed
                self._queue_step_messages(step)
                self._plan_continue_immediate = continue_immediately
                self._plan_action_until = None
                self._plan_action_min_until = now + max(0.0, float(min_duration))
                self._plan_action_timeout = now + timeout if timeout and timeout > 0 else None
                self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side) and (not strict_line)
                if duration > 0:
                    self._plan_action_until = now + duration
                    self._plan_action_until_line = False
                    self._log_plan_step(self._plan_index, step, "rotate-duration")
                    return move_action, speed
                self._plan_action_until_line = (until == "line") or (until == "")
                self._log_plan_step(self._plan_index, step, "rotate-until-line")
                return move_action, speed

            if action in ("FORWARD", "BACKWARD", "LEFT", "RIGHT"):
                if duration <= 0:
                    duration = 0.5
                move_action = action.capitalize()
                self.state = self.STATE_PLAN
                self._plan_action = move_action
                self._plan_action_speed = speed
                self._queue_step_messages(step)
                self._plan_continue_immediate = continue_immediately
                self._plan_action_until = now + duration
                self._plan_action_until_line = False
                self._plan_action_min_until = None
                self._plan_action_timeout = None
                self._log_plan_step(self._plan_index, step, "move")
                return move_action, speed

            self._log_warn(f"Unknown plan action skipped: {step.get('action')}")
            continue

        self._log_warn("Plan jump loop exceeded safety guard")
        self.stop()
        return "Stop", 0

    def _run_plan_action(self, frame, now):
        if self._plan_action is None:
            self.state = self.STATE_FOLLOWING
            return self._follow_default()

        if self._plan_action == "AutoFollow":
            if self._plan_action_until is not None and now < self._plan_action_until:
                if frame is None:
                    return "Forward", self.base_speed
                return self._follow_line(frame)
            self._plan_action = None
            self._plan_action_speed = None
            self._plan_action_until = None
            self._plan_action_until_line = False
            self._plan_action_min_until = None
            self._plan_action_timeout = None
            self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
            return self._after_plan_action(now)

        if self._plan_action_until_line:
            if self._plan_action_timeout is not None and now >= self._plan_action_timeout:
                self._log_warn(f"Plan rotate timeout reached: {self._plan_action}")
                self._plan_action = None
                self._plan_action_speed = None
                self._plan_action_until_line = False
                self._plan_action_min_until = None
                self._plan_action_timeout = None
                return self._after_plan_action(now)
            if self._plan_action_min_until is not None and now < self._plan_action_min_until:
                return self._plan_action, int(self._plan_action_speed)
            if frame is not None and self._is_line_reacquired(
                frame,
                self._plan_action,
                allow_side_stop=self._plan_rotate_allow_side_stop,
            ):
                self._plan_action = None
                self._plan_action_speed = None
                self._plan_action_until_line = False
                self._plan_action_min_until = None
                self._plan_action_timeout = None
                self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
                return self._after_plan_action(now)
            return self._plan_action, int(self._plan_action_speed)

        if self._plan_action_until is not None and now < self._plan_action_until:
            return self._plan_action, int(self._plan_action_speed)

        # action done
        self._plan_action = None
        self._plan_action_speed = None
        self._plan_action_until = None
        self._plan_action_until_line = False
        self._plan_action_min_until = None
        self._plan_action_timeout = None
        self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
        return self._after_plan_action(now)

    def _follow_default(self):
        self.state = self.STATE_FOLLOWING
        return "Forward", self.base_speed

    def _after_plan_action(self, now):
        if self._plan_index >= len(self.cross_plan):
            self._log_info(f"[PLAN] Completed all steps (end_state={self.plan_end_state})")
            if self.plan_end_state == "follow":
                self.state = self.STATE_FOLLOWING
                return self._follow_default()
            self._enter_stopped(reset_plan_progress=False)
            return "Stop", 0
        # Allow immediate transition for action steps that request no-wait-cross.
        # Keep existing behavior where next AutoLine step executes immediately.
        if self._plan_continue_immediate or self._next_action_is_autoline():
            self._plan_continue_immediate = False
            return self._start_plan_action(now)
        self._plan_continue_immediate = False
        self.state = self.STATE_FOLLOWING
        return self._follow_default()

    def _is_centered(self, frame):
        left_count = frame["left_count"]
        mid_count = frame["mid_count"]
        right_count = frame["right_count"]
        return (
            mid_count >= self.rotate_line_mid_min_count
            and left_count <= self.rotate_line_side_max_count
            and right_count <= self.rotate_line_side_max_count
        )

    def _is_line_reacquired(self, frame, rotate_action, allow_side_stop=None):
        if self._is_centered(frame):
            return True
        allow_side = self.rotate_early_stop_on_side if allow_side_stop is None else bool(allow_side_stop)
        if not allow_side:
            return False
        if rotate_action == "RotateLeft":
            return frame["left_count"] >= self.rotate_line_side_min_count
        if rotate_action == "RotateRight":
            return frame["right_count"] >= self.rotate_line_side_min_count
        return False

    def _follow_line(self, frame):
        left_count = frame["left_count"]
        mid_count = frame["mid_count"]
        right_count = frame["right_count"]
        left_full = frame["left_full"]
        mid_full = frame["mid_full"]
        right_full = frame["right_full"]

        total_black = left_count + mid_count + right_count
        if total_black > 0:
            self._plan_lost_line_since = None
        if total_black == 0:
            if self._has_pending_plan():
                self._hold_stop_for_plan_lost_line(now=None)
                self.state = self.STATE_FOLLOWING
                return "Stop", 0
            self.stop()
            self._log_info("===> LOST LINE: STOP")
            return "Stop", 0

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

    def _normalize_action(self, action):
        text = str(action or "").strip().upper()
        alias = {
            "TURNLEFT": "ROTATELEFT",
            "TURNRIGHT": "ROTATERIGHT",
            "ROTATE_LEFT": "ROTATELEFT",
            "ROTATE_RIGHT": "ROTATERIGHT",
            "PAUSE": "WAIT",
            "SLEEP": "WAIT",
            "GO": "FORWARD",
            "STRAIGHT": "FORWARD",
            "AUTO_LINE": "AUTOLINE",
            "AUTOLINE_ON": "AUTOLINE",
            "AUTOLINE_OFF": "AUTOLINE",
        }
        return alias.get(text, text)

    def consume_requested_autoline(self):
        value = self._requested_autoline
        self._requested_autoline = None
        return value

    def consume_requested_step_messages(self):
        value = self._requested_step_messages
        self._requested_step_messages = None
        return value

    def request_plan_start(self):
        if not self.cross_plan:
            return False
        self._plan_start_requested = True
        return True

    def set_autoline_mode(self, enabled):
        self._autoline_mode = bool(enabled)

    def _should_skip_cross_pre(self):
        """Skip pre-forward phase when next actionable step is rotate for immediate turn."""
        if self._autoline_mode:
            return False
        idx = self._plan_index
        guard = 0
        while idx < len(self.cross_plan) and guard < len(self.cross_plan):
            guard += 1
            step = self.cross_plan[idx]
            idx += 1
            if not isinstance(step, dict):
                continue
            action = self._normalize_action(step.get("action", "Stop"))
            if action in ("LABEL", "GOTO"):
                continue
            return action in ("ROTATELEFT", "ROTATERIGHT")
        return False

    def _next_action_is_autoline(self):
        idx = self._plan_index
        guard = 0
        while idx < len(self.cross_plan) and guard < len(self.cross_plan):
            guard += 1
            step = self.cross_plan[idx]
            idx += 1
            if not isinstance(step, dict):
                continue
            action = self._normalize_action(step.get("action", "Stop"))
            if action in ("LABEL",):
                continue
            if action == "GOTO":
                target = self._resolve_goto_target(step.get("target"))
                if target is None:
                    continue
                idx = target
                continue
            return action == "AUTOLINE"
        return False

    def _has_pending_plan(self):
        return bool(self.cross_plan) and self._plan_index < len(self.cross_plan)

    def _hold_stop_for_plan_lost_line(self, now=None):
        ts = time.time() if now is None else float(now)
        if self._plan_lost_line_since is None:
            self._plan_lost_line_since = ts
        elapsed = ts - self._plan_lost_line_since
        if elapsed < self.plan_lost_line_hold_sec:
            return
        if (ts - self._last_plan_lost_line_warn_ts) >= self.plan_lost_line_warn_period:
            self._log_warn(
                "LOST LINE during active plan; hold STOP and wait line reacquire "
                f"(elapsed={elapsed:.2f}s)"
            )
            self._last_plan_lost_line_warn_ts = ts

    def _resolve_goto_target(self, target):
        if isinstance(target, int):
            if 0 <= target < len(self.cross_plan):
                return target
            return None
        if isinstance(target, str):
            name = target.strip()
            if not name:
                return None
            if name.isdigit():
                idx = int(name)
                if 0 <= idx < len(self.cross_plan):
                    return idx
            return self._plan_labels.get(name)
        return None

    def _rebuild_plan_labels(self, steps):
        labels = {}
        for idx, step in enumerate(steps or []):
            if not isinstance(step, dict):
                continue
            name = step.get("label")
            if isinstance(name, str):
                key = name.strip()
                if key:
                    labels[key] = idx
        return labels

    def get_plan_status(self):
        total = len(self.cross_plan)
        next_step = self._plan_index + 1 if self._plan_index < total else total
        return {
            "has_plan": total > 0,
            "state": self._state_name(self.state),
            "total_steps": total,
            "next_step": next_step,
            "current_action": self._plan_action,
            "end_state": self.plan_end_state,
            "plan_done": self.is_plan_done(),
        }

    def is_plan_done(self):
        return self._plan_index >= len(self.cross_plan) and self._plan_action is None

    def _state_name(self, state):
        names = {
            self.STATE_FOLLOWING: "FOLLOWING",
            self.STATE_CROSSING: "CROSSING",
            self.STATE_TURN_LEFT: "TURN_LEFT",
            self.STATE_TURN_RIGHT: "TURN_RIGHT",
            self.STATE_BACKWARD: "BACKWARD",
            self.STATE_PLAN: "PLAN",
            self.STATE_CROSS_PRE: "CROSS_PRE",
            self.STATE_STOPPED: "STOPPED",
        }
        return names.get(state, f"UNKNOWN({state})")

    def _log_plan_step(self, step_index_1_based, step, mode):
        action = step.get("action", "Stop")
        speed = step.get("speed", self.base_speed)
        duration = step.get("duration")
        until = step.get("until")
        timeout = step.get("timeout")
        total = len(self.cross_plan)
        self._log_info(
            f"[PLAN] Step {step_index_1_based}/{total}: action={action}, mode={mode}, "
            f"speed={speed}, duration={duration}, until={until}, timeout={timeout}"
        )

    def _to_int(self, raw, default, field_name, step):
        if raw is None:
            return int(default)
        try:
            return int(raw)
        except (TypeError, ValueError):
            self._log_warn(
                f"Invalid {field_name}='{raw}' in step action={step.get('action')}; use default={default}"
            )
            return int(default)

    def _to_float(self, raw, default, field_name, step):
        if raw is None:
            return default
        try:
            return float(raw)
        except (TypeError, ValueError):
            self._log_warn(
                f"Invalid {field_name}='{raw}' in step action={step.get('action')}; use default={default}"
            )
            return default

    def _to_bool(self, raw, default):
        if raw is None:
            return bool(default)
        if isinstance(raw, bool):
            return raw
        if isinstance(raw, (int, float)):
            return bool(raw)
        text = str(raw).strip().lower()
        if text in {"1", "true", "yes", "on", "enable", "enabled"}:
            return True
        if text in {"0", "false", "no", "off", "disable", "disabled"}:
            return False
        return bool(default)

    def _queue_step_messages(self, step):
        raw = step.get("messages")
        if not isinstance(raw, list) or not raw:
            return
        self._requested_step_messages = raw
