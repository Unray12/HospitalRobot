"""Finite-state controller for line following, plan execution, and HuskyLens guidance."""

import time

from robot_common.logging_utils import to_bool
from .line_follow_strategies import HybridStrategy, HuskyLensStrategy, LineSensorStrategy

# HuskyLens y_type classification codes -- must stay in sync with the firmware
# `huskylens_sensor/device_code/main.py` (Y_TYPE_* constants).
Y_TYPE_NO_LINE             = 0
Y_TYPE_BOTTOM_TO_MID       = 1
Y_TYPE_MID_TO_TOP          = 2
Y_TYPE_BOTTOM_TO_TOP       = 3
Y_TYPE_BOTTOM_TO_MID_SHORT = 5
Y_TYPE_MID_TO_TOP_SHORT    = 6

# Any of these is treated as "robot is near a cross / line ends just under the
# camera" and arms the cross_pre trigger. Adding a code here also makes the
# rotate-until-y_type plan step exit correctly when the new code reappears.
_Y_TYPE_CROSS_TRIGGER  = (Y_TYPE_BOTTOM_TO_MID, Y_TYPE_BOTTOM_TO_MID_SHORT)
_Y_TYPE_LINE_ACQUIRED  = (Y_TYPE_MID_TO_TOP, Y_TYPE_MID_TO_TOP_SHORT, Y_TYPE_BOTTOM_TO_TOP)


class LineFollowerFSM:
    """Coordinate tracking strategies, cross handling, and plan-driven actions."""

    # ── FSM state constants ────────────────────────────────────────────────────
    STATE_FOLLOWING = 0
    STATE_CROSSING = 1
    STATE_TURN_LEFT = 2
    STATE_TURN_RIGHT = 3
    STATE_BACKWARD = 4
    STATE_PLAN = 5
    STATE_CROSS_PRE = 6
    STATE_STOPPED = 7

    # ── Default durations (seconds) ───────────────────────────────────────────
    # Used as fallback when no duration is specified in a plan step.
    # WAIT default: brief pause before continuing (sub-second so it feels snappy)
    _DEFAULT_WAIT_DURATION = 0.3
    # FOLLOW default: minimum forward-follow time before plan advances
    _DEFAULT_FOLLOW_DURATION = 0.6
    # MOVE (Forward/Backward/Left/Right) default: half-second nudge
    _DEFAULT_MOVE_DURATION = 0.5

    # ── Jump-guard constants ───────────────────────────────────────────────────
    # Safety cap on Goto/Label hops to prevent infinite plan loops.
    # Max iterations = max(_JUMP_GUARD_MIN, plan_length * _JUMP_GUARD_MULTIPLIER)
    _JUMP_GUARD_MIN = 4
    _JUMP_GUARD_MULTIPLIER = 2

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
        tracking_strategy="line_sensor",
        tracking_strict_mode=False,
        tracking_log_invalid_period=1.0,
        tracking_allow_line_sensor_fallback=True,
        huskylens_lateral_deadband=10.0,
        huskylens_heading_deadband=3.0,
        huskylens_y_type_rotate_timeout=5.0,
        huskylens_short_line_speed_factor=0.5,
        huskylens_lateral_hysteresis=6.0,
        huskylens_heading_hysteresis=3.0,
        huskylens_command_hold_sec=0.20,
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
        self.tracking_strategy_name = str(tracking_strategy or "line_sensor").strip().lower()
        self.tracking_strict_mode = bool(tracking_strict_mode)
        self.tracking_log_invalid_period = float(max(0.1, tracking_log_invalid_period))
        self.tracking_allow_line_sensor_fallback = bool(tracking_allow_line_sensor_fallback)
        self.huskylens_lateral_deadband = float(max(0.0, huskylens_lateral_deadband))
        self.huskylens_heading_deadband = float(max(0.0, huskylens_heading_deadband))
        self.huskylens_y_type_rotate_timeout = float(max(0.0, huskylens_y_type_rotate_timeout))
        # Multiplier applied to base/turn speed when y_type is a *_SHORT variant
        # (line is short -> robot is close to a cross or to the end of the line).
        # 1.0 = no slow-down, 0.0 = stop. Default 0.5 = half speed.
        self.huskylens_short_line_speed_factor = float(
            min(1.0, max(0.0, huskylens_short_line_speed_factor))
        )
        # Hysteresis added to deadband to break free of a turn: once turning,
        # the value must drop *inside* deadband by at least the hysteresis
        # amount before Forward kicks back in. Prevents flicker on noisy frames.
        self.huskylens_lateral_hysteresis = float(max(0.0, huskylens_lateral_hysteresis))
        self.huskylens_heading_hysteresis = float(max(0.0, huskylens_heading_hysteresis))
        # Minimum time to hold a non-Forward command before switching to another
        # non-Forward command. Smoothes high-rate command flicker at 100 Hz.
        self.huskylens_command_hold_sec = float(max(0.0, huskylens_command_hold_sec))
        self._last_huskylens_action = None
        self._last_huskylens_action_ts = 0.0
        self._tracking_context = {}
        self._last_tracking_invalid_log_ts = 0.0

        self._line_sensor_strategy = LineSensorStrategy(self._compute_line_sensor_command)
        self._huskylens_strategy = HuskyLensStrategy(self._compute_huskylens_command)
        self._hybrid_strategy = HybridStrategy(self._huskylens_strategy, self._line_sensor_strategy)
        self._tracking_strategy = self._resolve_tracking_strategy(self.tracking_strategy_name)

        self._reset_state(state=self.STATE_FOLLOWING, reset_plan_index=True)
        self._plan_labels = self._rebuild_plan_labels(self.cross_plan)

    def _reset_state(self, state=STATE_FOLLOWING, reset_plan_index=True, clear_plan_fields=False):
        self.state = state
        self.crossing_start_time = None
        if reset_plan_index:
            self._plan_index = 0
        self._plan_step_start = None
        self._plan_action_until = None
        self._plan_action = None
        self._plan_action_speed = None
        self._plan_action_until_line = False
        self._plan_action_until_y_type = False
        self._plan_action_min_until = None
        self._plan_action_timeout = None
        self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
        self._plan_rotate_strict_center = False
        self._plan_continue_immediate = False
        self._plan_force_complete = False
        self._cross_active = False
        self._y_type_cross_active = False
        self._plan_new_step = True
        self._cross_pre_phase = 0
        self._cross_pre_until = None
        self._requested_autoline = None
        self._requested_step_messages = None
        self._plan_start_requested = False
        self._autoline_mode = False
        self._plan_lost_line_since = None
        self._last_plan_lost_line_warn_ts = 0.0
        if clear_plan_fields:
            self.cross_plan = []
            self.plan_end_state = "stop"
            self._plan_labels = {}

    def _clear_plan_action_state(self):
        self._plan_action = None
        self._plan_action_speed = None
        self._plan_action_until = None
        self._plan_action_until_line = False
        self._plan_action_until_y_type = False
        self._plan_action_min_until = None
        self._plan_action_timeout = None
        self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side)
        self._plan_rotate_strict_center = False

    def reset(self):
        self._reset_state(state=self.STATE_FOLLOWING, reset_plan_index=True)

    def stop(self):
        self._enter_stopped(reset_plan_progress=False)

    def _enter_stopped(self, reset_plan_progress=True):
        self._reset_state(state=self.STATE_STOPPED, reset_plan_index=reset_plan_progress)

    def set_plan(self, steps, end_state=None):
        self.cross_plan = steps or []
        if end_state:
            self.plan_end_state = end_state
        self._plan_labels = self._rebuild_plan_labels(self.cross_plan)
        self._reset_state(state=self.STATE_FOLLOWING, reset_plan_index=True)

    def clear_plan(self):
        self._reset_state(state=self.STATE_FOLLOWING, reset_plan_index=True, clear_plan_fields=True)

    def update(self, frame, now):
        if self.state == self.STATE_STOPPED:
            return "Stop", 0
        if self.state == self.STATE_CROSSING:
            return self._handle_crossing(frame, now)
        if self.state == self.STATE_PLAN:
            return self._run_plan_action(frame, now)
        if self.state == self.STATE_CROSS_PRE:
            return self._handle_cross_pre(now)
        # Plan trigger via /plan_select with start_without_cross=true. Skips
        # waiting for a physical line cross — useful when robot is hand-placed
        # at a starting position with no nearby crossing.
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

        # HuskyLens y_type-based crossing trigger: fires before line-sensor logic so
        # huskylens-mode plans can advance without 3-sensor cross detection.
        # Always runs the pre-forward phase since the user explicitly requested the
        # robot drive forward a bit when y_type==BOTTOM_TO_MID before rotating.
        if (
            self.tracking_strategy_name == "huskylens"
            and self.cross_plan
            and self._next_rotate_uses_y_type()
        ):
            y_type = self._huskylens_y_type()
            if y_type is not None:
                # Edge-triggered: only fire on the BOTTOM_TO_MID(_SHORT) transition,
                # not on every frame that stays in that band. _y_type_cross_active
                # latches True until y_type leaves the trigger band.
                if y_type in _Y_TYPE_CROSS_TRIGGER and not self._y_type_cross_active:
                    self._y_type_cross_active = True
                    self.state = self.STATE_CROSS_PRE
                    self._cross_pre_phase = 0
                    self._cross_pre_until = now + self.cross_pre_forward_duration
                    self._log_info(
                        f"===> PLAN trigger (y_type={y_type}): enter cross_pre"
                    )
                    return "Forward", self.cross_pre_forward_speed
                if y_type not in _Y_TYPE_CROSS_TRIGGER:
                    self._y_type_cross_active = False

        if frame is None:
            # HuskyLens/hybrid mode may still provide valid tracking even without line sensor frame.
            if self._should_bypass_line_sensor_gate():
                return self._follow_line(frame)
            return None

        left_count = frame["left_count"]
        mid_count = frame["mid_count"]
        right_count = frame["right_count"]
        left_full = frame["left_full"]
        mid_full = frame["mid_full"]
        right_full = frame["right_full"]

        total_black = left_count + mid_count + right_count
        # Cross is detected only when ALL 3 zones are fully black. The
        # cross_event flag distinguishes the rising edge from a sustained
        # cross (e.g. robot sitting on top of the crossing) — we only act
        # once per crossing.
        cross_detected = left_full and mid_full and right_full
        cross_event = cross_detected and not self._cross_active
        self._cross_active = cross_detected
        if total_black > 0:
            self._plan_lost_line_since = None

        # In HuskyLens-first tracking, ignore line-sensor cross/lost-line gate while HuskyLens is usable.
        if self._should_bypass_line_sensor_gate():
            return self._follow_line(frame)

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
            self._log_info(f"===> CROSS detected: move forward {self.crossing_duration}s")
            return "Forward", self.base_speed

        # Lost line -> stop
        if total_black == 0:
            if self._has_pending_plan():
                # Don't reset plan_index here; operator may want to resume
                # after re-aligning the robot. _hold_stop_for_plan_lost_line
                # logs a periodic warning so the operator notices.
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
        # Two-phase pause-then-act executed before any plan action triggered
        # by a cross. Phase 0: drive forward to fully cross the intersection
        # so the rotate axis sits on the crossing center. Phase 1: brief stop
        # to bleed off momentum before rotating.
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

        # Defensive fallback: if _cross_pre_phase somehow lands outside {0, 1}
        # (e.g. partial reset on plan reload), recover by re-arming phase 0
        # rather than returning None — None would leave the FSM stuck in
        # STATE_CROSS_PRE forever because the timer callback skips publish.
        self._log_warn(f"cross_pre invalid phase={self._cross_pre_phase}; reset")
        self._cross_pre_phase = 0
        self._cross_pre_until = now + self.cross_pre_forward_duration
        return "Forward", self.cross_pre_forward_speed

    def _start_plan_action(self, now):
        if not self.cross_plan:
            return self._follow_default()

        # Loop guard: Goto/Label steps don't consume real time, so a plan that
        # only contains Goto/Label could loop forever. Cap iterations at 2x
        # the plan length to break runaway loops (real plans should never need
        # this many no-op hops in a row).
        jump_guard = 0
        while jump_guard < max(self._JUMP_GUARD_MIN, len(self.cross_plan) * self._JUMP_GUARD_MULTIPLIER):
            jump_guard += 1
            if self._plan_index >= len(self.cross_plan):
                if self.plan_end_state == "follow":
                    return self._follow_default()
                self._enter_stopped(reset_plan_progress=False)
                return "Stop", 0

            step = self.cross_plan[self._plan_index]
            self._plan_index += 1
            self._plan_new_step = False
            if not isinstance(step, dict):
                self._log_warn(f"Invalid plan step type at index {self._plan_index - 1}: {type(step).__name__}")
                continue

            action = self._normalize_action(step.get("action", "Stop"))
            if action in ("LABEL",):
                # Label is a marker for Goto targets, not an executable step.
                continue
            if action == "GOTO":
                target = self._resolve_goto_target(step.get("target"))
                if target is None:
                    self._log_warn(f"Invalid Goto target: {step.get('target')}")
                    continue
                self._log_info(f"[PLAN] Goto -> step {target + 1}")
                self._plan_index = target
                continue

            speed = self._to_int(step.get("speed"), self.base_speed, "speed", step)
            duration = self._to_float(step.get("duration"), 0.0, "duration", step)
            until = str(step.get("until", "") or "").strip().lower()
            timeout = self._to_float(step.get("timeout"), None, "timeout", step)
            continue_immediately = self._to_bool(
                step.get("continue_immediately", step.get("no_wait_cross")), False,
            )
            end_after_step = self._to_bool(step.get("end_state"), False)

            result = self._dispatch_plan_action(
                action, step, now, speed, duration, until, timeout,
                continue_immediately, end_after_step,
            )
            if result is not None:
                return result

            self._log_warn(f"Unknown plan action skipped: {step.get('action')}")
            continue

        self._log_warn("Plan jump loop exceeded safety guard")
        self.stop()
        return "Stop", 0

    def _dispatch_plan_action(
        self, action, step, now, speed, duration, until, timeout,
        continue_immediately, end_after_step,
    ):
        if action == "AUTOLINE":
            return self._exec_autoline(step, continue_immediately, end_after_step)
        if action in ("AUTO",):
            return self._follow_default()
        if action in ("WAIT", "STOP"):
            return self._exec_wait_stop(action, step, now, duration, continue_immediately, end_after_step)
        if action == "FOLLOW":
            return self._exec_follow(step, now, duration, continue_immediately, end_after_step)
        if action in ("ROTATELEFT", "ROTATERIGHT"):
            return self._exec_rotate(action, step, now, speed, duration, until, timeout, continue_immediately, end_after_step)
        if action in ("FORWARD", "BACKWARD", "LEFT", "RIGHT"):
            return self._exec_move(action, step, now, speed, duration, continue_immediately, end_after_step)
        return None

    def _exec_autoline(self, step, continue_immediately, end_after_step):
        enabled = self._to_bool(
            step.get("enabled", step.get("value", step.get("autoline"))), True,
        )
        self._requested_autoline = enabled
        self._queue_step_messages(step)
        self._plan_continue_immediate = continue_immediately
        self._plan_force_complete = end_after_step
        self._log_plan_step(self._plan_index, step, f"autoline-{enabled}")
        return self._follow_default()

    def _exec_wait_stop(self, action, step, now, duration, continue_immediately, end_after_step):
        if duration and duration > 0:
            self.state = self.STATE_PLAN
            self._plan_action = "Stop"
            self._plan_action_speed = 0
            self._queue_step_messages(step)
            self._plan_continue_immediate = continue_immediately
            self._plan_force_complete = end_after_step
            self._plan_action_until = now + duration
            self._log_plan_step(self._plan_index, step, "timed-stop")
            return "Stop", 0
        if action == "WAIT":
            duration = self._DEFAULT_WAIT_DURATION
            self.state = self.STATE_PLAN
            self._plan_action = "Stop"
            self._plan_action_speed = 0
            self._queue_step_messages(step)
            self._plan_continue_immediate = continue_immediately
            self._plan_force_complete = end_after_step
            self._plan_action_until = now + duration
            self._log_plan_step(self._plan_index, step, "wait-default")
            return "Stop", 0
        if self.plan_end_state == "follow":
            self._log_info("[PLAN] Stop step reached, returning to follow")
            return self._follow_default()
        self._enter_stopped(reset_plan_progress=False)
        self._log_info("[PLAN] Stop step reached, robot stopped")
        return "Stop", 0

    def _exec_follow(self, step, now, duration, continue_immediately, end_after_step):
        if duration <= 0:
            duration = self._DEFAULT_FOLLOW_DURATION
        self.state = self.STATE_PLAN
        self._plan_action = "AutoFollow"
        self._plan_action_speed = self.base_speed
        self._queue_step_messages(step)
        self._plan_continue_immediate = continue_immediately
        self._plan_force_complete = end_after_step
        self._plan_action_until = now + duration
        self._log_plan_step(self._plan_index, step, "follow")
        return "Forward", self.base_speed

    def _exec_rotate(self, action, step, now, speed, duration, until, timeout, continue_immediately, end_after_step):
        move_action = "RotateLeft" if action == "ROTATELEFT" else "RotateRight"
        strict_line = self._to_bool(
            step.get("strict_line", step.get("center_only")), False,
        )
        min_duration = self._to_float(
            step.get("min_duration"), self.rotate_min_duration, "min_duration", step,
        )
        self.state = self.STATE_PLAN
        self._plan_action = move_action
        self._plan_action_speed = speed
        self._queue_step_messages(step)
        self._plan_continue_immediate = continue_immediately
        self._plan_force_complete = end_after_step
        self._plan_action_until = None
        self._plan_action_min_until = now + max(0.0, float(min_duration))
        self._plan_action_timeout = now + timeout if timeout and timeout > 0 else None
        self._plan_rotate_allow_side_stop = bool(self.rotate_early_stop_on_side) and (not strict_line)
        self._plan_rotate_strict_center = bool(strict_line)
        if duration > 0:
            self._plan_action_until = now + duration
            self._plan_action_until_line = False
            self._plan_action_until_y_type = False
            self._log_plan_step(self._plan_index, step, "rotate-duration")
            return move_action, speed
        if until == "y_type":
            self._plan_action_until_line = False
            self._plan_action_until_y_type = True
            # Reset edge tracker so the *next* BOTTOM_TO_MID after rotation can retrigger.
            self._y_type_cross_active = False
            # Apply a default safety timeout if step doesn't supply one.
            if self._plan_action_timeout is None and self.huskylens_y_type_rotate_timeout > 0:
                self._plan_action_timeout = now + self.huskylens_y_type_rotate_timeout
            self._log_plan_step(self._plan_index, step, "rotate-until-y_type")
            return move_action, speed
        self._plan_action_until_line = (until == "line") or (until == "")
        self._plan_action_until_y_type = False
        self._log_plan_step(self._plan_index, step, "rotate-until-line")
        return move_action, speed

    def _exec_move(self, action, step, now, speed, duration, continue_immediately, end_after_step):
        if duration <= 0:
            duration = self._DEFAULT_MOVE_DURATION
        move_action = action.capitalize()
        self.state = self.STATE_PLAN
        self._plan_action = move_action
        self._plan_action_speed = speed
        self._queue_step_messages(step)
        self._plan_continue_immediate = continue_immediately
        self._plan_force_complete = end_after_step
        self._plan_action_until = now + duration
        self._log_plan_step(self._plan_index, step, "move")
        return move_action, speed

    def _run_plan_action(self, frame, now):
        if self._plan_action is None:
            self.state = self.STATE_FOLLOWING
            return self._follow_default()

        if self._plan_action == "AutoFollow":
            if self._plan_action_until is not None and now < self._plan_action_until:
                if frame is None:
                    return "Forward", self.base_speed
                return self._follow_line(frame)
            self._clear_plan_action_state()
            return self._after_plan_action(now)

        if self._plan_action_until_line:
            if self._plan_action_timeout is not None and now >= self._plan_action_timeout:
                self._log_warn(f"Plan rotate timeout reached: {self._plan_action}")
                self._clear_plan_action_state()
                self._plan_action_until_line = False
                self._plan_rotate_strict_center = False
                return self._after_plan_action(now)
            if self._plan_action_min_until is not None and now < self._plan_action_min_until:
                return self._plan_action, int(self._plan_action_speed)
            if frame is not None and self._is_line_reacquired(
                frame, self._plan_action,
                allow_side_stop=self._plan_rotate_allow_side_stop,
                strict_center=self._plan_rotate_strict_center,
            ):
                self._clear_plan_action_state()
                return self._after_plan_action(now)
            # In huskylens/hybrid tracking, the camera can detect a line ahead
            # before the 3-zone line sensor (robot rotating over no-line floor).
            # Accept HuskyLens line-acquired y_type as an exit signal too.
            if self.tracking_strategy_name in ("huskylens", "hybrid"):
                y_type = self._huskylens_y_type()
                if y_type is not None and y_type in _Y_TYPE_LINE_ACQUIRED:
                    self._log_info(
                        f"===> until:line satisfied by HuskyLens y_type={y_type}"
                    )
                    self._clear_plan_action_state()
                    return self._after_plan_action(now)
            return self._plan_action, int(self._plan_action_speed)

        if self._plan_action_until_y_type:
            if self._plan_action_timeout is not None and now >= self._plan_action_timeout:
                self._log_warn(f"Plan rotate y_type timeout reached: {self._plan_action}")
                self._clear_plan_action_state()
                return self._after_plan_action(now)
            if self._plan_action_min_until is not None and now < self._plan_action_min_until:
                return self._plan_action, int(self._plan_action_speed)
            if self._huskylens_y_type() in _Y_TYPE_LINE_ACQUIRED:
                self._clear_plan_action_state()
                return self._after_plan_action(now)
            return self._plan_action, int(self._plan_action_speed)

        if self._plan_action_until is not None and now < self._plan_action_until:
            return self._plan_action, int(self._plan_action_speed)

        self._clear_plan_action_state()
        return self._after_plan_action(now)

    def _follow_default(self):
        self.state = self.STATE_FOLLOWING
        return "Forward", self.base_speed

    def _after_plan_action(self, now):
        if self._plan_force_complete:
            self._plan_index = len(self.cross_plan)
            self._plan_force_complete = False
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

    def _is_line_reacquired(self, frame, rotate_action, allow_side_stop=None, strict_center=False):
        if strict_center:
            return (
                frame["mid_count"] >= self.rotate_line_mid_min_count
                and frame["left_count"] == 0
                and frame["right_count"] == 0
            )
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
        context = dict(self._tracking_context or {})
        if isinstance(frame, dict) and context.get("line_sensor_frame") is None:
            context["line_sensor_frame"] = frame
            context["line_sensor_stale"] = False

        command = self._tracking_strategy.compute(context)
        if command is None:
            if self.tracking_strict_mode:
                now = time.time()
                if (now - self._last_tracking_invalid_log_ts) >= self.tracking_log_invalid_period:
                    self._log_warn(
                        f"strict_mode stop: strategy={self.tracking_strategy_name} no valid input",
                        event="TRACKING",
                    )
                    self._last_tracking_invalid_log_ts = now
                self.state = self.STATE_STOPPED
                return "Stop", 0
            if not self.tracking_allow_line_sensor_fallback:
                now = time.time()
                if (now - self._last_tracking_invalid_log_ts) >= self.tracking_log_invalid_period:
                    self._log_warn(
                        (
                            f"strategy={self.tracking_strategy_name} invalid input with "
                            "line_sensor fallback disabled -> stop"
                        ),
                        event="TRACKING",
                    )
                    self._last_tracking_invalid_log_ts = now
                self.state = self.STATE_STOPPED
                return "Stop", 0
            now = time.time()
            if (now - self._last_tracking_invalid_log_ts) >= self.tracking_log_invalid_period:
                self._log_warn(
                    f"strategy={self.tracking_strategy_name} invalid input -> fallback line_sensor",
                    event="FALLBACK",
                )
                self._last_tracking_invalid_log_ts = now
            return self._compute_line_sensor_command(
                {
                    "line_sensor_frame": frame,
                    "line_sensor_stale": False,
                }
            )
        return command

    def _compute_line_sensor_command(self, frame_context):
        frame = (frame_context or {}).get("line_sensor_frame")
        if not isinstance(frame, dict):
            return None
        if bool((frame_context or {}).get("line_sensor_stale", False)):
            return None

        left_count = int(frame.get("left_count", 0) or 0)
        mid_count = int(frame.get("mid_count", 0) or 0)
        right_count = int(frame.get("right_count", 0) or 0)
        left_full = bool(frame.get("left_full", False))
        mid_full = bool(frame.get("mid_full", False))
        right_full = bool(frame.get("right_full", False))

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

    def _compute_huskylens_command(self, frame_context):
        """Compute a command from the normalized HuskyLens frame.

        Lateral correction comes from ``tail_offset_x`` (sign of the tail offset),
        heading correction from ``angle_deg``. Lateral takes priority over heading;
        within each axis the deadband from config decides when to start steering.

        Three smoothing layers stack to fight noisy-frame oscillation:
          1. Deadband on each axis (lateral/heading)
          2. Hysteresis — once turning, the value must fall *inside* deadband by
             ``hysteresis`` extra before Forward releases.
          3. Command hold — once a non-Forward command is issued, do not switch
             to a *different* non-Forward command for ``command_hold_sec`` seconds.
        """
        huskylens = (frame_context or {}).get("huskylens_frame")
        if not isinstance(huskylens, dict):
            return None
        if bool((frame_context or {}).get("huskylens_stale", False)):
            return None

        connected = bool(huskylens.get("connected", 0))
        algorithm_set = bool(huskylens.get("algorithm_set", 0))
        valid = bool(huskylens.get("valid", 0))
        if not (connected and algorithm_set and valid):
            return None

        try:
            angle_deg = float(huskylens.get("angle_deg"))
            tail_offset_x = float(huskylens.get("tail_offset_x"))
        except Exception:
            return None

        try:
            y_type = int(huskylens.get("y_type", 0))
        except Exception:
            y_type = 0
        is_short = y_type in (Y_TYPE_BOTTOM_TO_MID_SHORT, Y_TYPE_MID_TO_TOP_SHORT)
        scale = self.huskylens_short_line_speed_factor if is_short else 1.0

        def _scaled(speed):
            return max(0, int(round(int(speed) * scale)))

        lat_db = self.huskylens_lateral_deadband
        hd_db = self.huskylens_heading_deadband
        lat_hyst = self.huskylens_lateral_hysteresis
        hd_hyst = self.huskylens_heading_hysteresis

        prev = self._last_huskylens_action
        # When already turning a given direction, require the offset/angle to
        # drop INSIDE deadband by `hysteresis` extra before releasing. This
        # widens the "stay turning" band on the active side without widening
        # the entry threshold.
        lat_release_left  = lat_db - lat_hyst   # tail_offset_x must drop below this to stop strafing Left
        lat_release_right = -(lat_db - lat_hyst)  # tail_offset_x must rise above this to stop strafing Right
        hd_release_left   = hd_db - hd_hyst
        hd_release_right  = -(hd_db - hd_hyst)

        # Decide raw command (lateral first, then heading, else forward).
        if prev == "Left" and tail_offset_x > lat_release_left:
            raw = "Left"
        elif prev == "Right" and tail_offset_x < lat_release_right:
            raw = "Right"
        elif tail_offset_x > lat_db:
            raw = "Left"
        elif tail_offset_x < -lat_db:
            raw = "Right"
        elif prev == "RotateLeft" and angle_deg > hd_release_left:
            raw = "RotateLeft"
        elif prev == "RotateRight" and angle_deg < hd_release_right:
            raw = "RotateRight"
        elif angle_deg > hd_db:
            raw = "RotateLeft"
        elif angle_deg < -hd_db:
            raw = "RotateRight"
        else:
            raw = "Forward"

        # Command hold: once a non-Forward command is active, block switches to
        # a *different* non-Forward command for command_hold_sec. Switching to
        # Forward is always allowed (we want to settle on the line, not flick
        # between turn directions).
        now = time.time()
        hold = self.huskylens_command_hold_sec
        if (
            prev is not None
            and prev != "Forward"
            and raw != "Forward"
            and raw != prev
            and (now - self._last_huskylens_action_ts) < hold
        ):
            raw = prev

        # Latch + state mapping.
        if raw != prev:
            self._last_huskylens_action_ts = now
        self._last_huskylens_action = raw

        if raw == "Left":
            self.state = self.STATE_TURN_LEFT
            return "Left", _scaled(self.turn_speed_left)
        if raw == "Right":
            self.state = self.STATE_TURN_RIGHT
            return "Right", _scaled(self.turn_speed_right)
        if raw == "RotateLeft":
            self.state = self.STATE_TURN_LEFT
            return "RotateLeft", _scaled(self.turn_speed_left)
        if raw == "RotateRight":
            self.state = self.STATE_TURN_RIGHT
            return "RotateRight", _scaled(self.turn_speed_right)
        self.state = self.STATE_FOLLOWING
        return "Forward", _scaled(self.base_speed)

    def _log_info(self, msg, event="INFO"):
        if self._logger:
            self._logger.info(msg, event=event)
        else:
            print(msg)

    def _log_warn(self, msg, event="WARN"):
        if self._logger:
            self._logger.warning(msg, event=event)
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

    def set_tracking_context(self, frame_context):
        self._tracking_context = frame_context if isinstance(frame_context, dict) else {}

    def _resolve_tracking_strategy(self, strategy_name):
        normalized = str(strategy_name or "line_sensor").strip().lower()
        if normalized == "huskylens":
            return self._huskylens_strategy
        if normalized == "hybrid":
            return self._hybrid_strategy
        if normalized != "line_sensor":
            self._log_warn(f"unknown strategy '{strategy_name}', fallback to line_sensor", event="STRATEGY")
            self.tracking_strategy_name = "line_sensor"
        return self._line_sensor_strategy

    def _should_bypass_line_sensor_gate(self):
        if self.tracking_strategy_name == "line_sensor":
            return False
        ctx = self._tracking_context if isinstance(self._tracking_context, dict) else {}
        huskylens = ctx.get("huskylens_frame")
        if not isinstance(huskylens, dict):
            return False
        if bool(ctx.get("huskylens_stale", False)):
            return False
        return (
            bool(huskylens.get("connected", 0))
            and bool(huskylens.get("algorithm_set", 0))
            and bool(huskylens.get("valid", 0))
        )

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

    def _next_rotate_uses_y_type(self):
        """Return True when the next actionable plan step is a rotate-until-line/y_type.

        Both `until: line` and `until: y_type` qualify: in huskylens/hybrid mode
        the `until: line` branch also exits on HuskyLens line-acquired y_type
        (see _plan_action_until_line handler), so both step kinds need the
        upstream cross_pre trigger to fire when y_type signals an approaching
        cross.
        """
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
            if action not in ("ROTATELEFT", "ROTATERIGHT"):
                return False
            until = str(step.get("until", "") or "").strip().lower()
            return until in ("y_type", "line", "")
        return False

    def _huskylens_y_type(self):
        ctx = self._tracking_context if isinstance(self._tracking_context, dict) else {}
        if bool(ctx.get("huskylens_stale", False)):
            return None
        huskylens = ctx.get("huskylens_frame")
        if not isinstance(huskylens, dict):
            return None
        if not (
            bool(huskylens.get("connected", 0))
            and bool(huskylens.get("algorithm_set", 0))
            and bool(huskylens.get("valid", 0))
        ):
            return None
        try:
            return int(huskylens.get("y_type", 0))
        except Exception:
            return None

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
        min_duration = step.get("min_duration")
        until = step.get("until")
        timeout = step.get("timeout")
        total = len(self.cross_plan)
        self._log_info(
            f"[PLAN] Step {step_index_1_based}/{total}: action={action}, mode={mode}, "
            f"speed={speed}, duration={duration}, min_duration={min_duration}, "
            f"until={until}, timeout={timeout}"
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
        return to_bool(raw, default)

    def _queue_step_messages(self, step):
        raw = step.get("messages")
        if not isinstance(raw, list) or not raw:
            return
        self._requested_step_messages = raw
