import math


def compute_tail_offset_and_angle(frame, image_center_x=160):
    if not isinstance(frame, dict):
        return None

    required = ("x_tail", "y_tail", "x_head", "y_head", "connected", "algorithm_set")
    for name in required:
        if name not in frame:
            return None

    try:
        connected = int(frame.get("connected", 0))
        algorithm_set = int(frame.get("algorithm_set", 0))
        x_tail = int(frame.get("x_tail", -1))
        y_tail = int(frame.get("y_tail", -1))
        x_head = int(frame.get("x_head", -1))
        y_head = int(frame.get("y_head", -1))
    except Exception:
        return None

    if connected != 1 or algorithm_set != 1:
        return None
    if x_tail < 0 or y_tail < 0 or x_head < 0 or y_head < 0:
        return None

    tail_offset_x = x_tail - int(image_center_x)
    dx = x_head - x_tail
    dy_up = y_tail - y_head
    if dy_up <= 0:
        return None

    angle_deg = math.degrees(math.atan2(dx, dy_up))
    return {
        "tail_offset_x": float(tail_offset_x),
        "angle_deg": float(angle_deg),
        "x_tail": x_tail,
        "y_tail": y_tail,
        "x_head": x_head,
        "y_head": y_head,
    }


class HuskyLensRawController:
    def __init__(self, config=None):
        cfg = config or {}
        self.image_center_x = int(cfg.get("image_center_x", 160))
        self.speed_forward = int(cfg.get("speed_forward", 8))
        self.speed_rotate = int(cfg.get("speed_rotate", 6))
        self.speed_lateral = int(cfg.get("speed_lateral", 6))
        self.alpha_tail = float(min(1.0, max(0.01, cfg.get("alpha_tail", 0.35))))
        self.alpha_angle = float(min(1.0, max(0.01, cfg.get("alpha_angle", 0.25))))
        self.tail_deadband_px = float(max(0.0, cfg.get("tail_deadband_px", 8.0)))
        self.angle_deadband_deg = float(max(0.1, cfg.get("angle_deadband_deg", 1.0)))
        self.tail_hysteresis_px = float(max(0.0, cfg.get("tail_hysteresis_px", 2.0)))
        self.angle_hysteresis_deg = float(max(0.0, cfg.get("angle_hysteresis_deg", 0.5)))
        self.invalid_timeout_sec = float(max(0.0, cfg.get("invalid_timeout_sec", 0.4)))
        self.min_command_hold_ms = int(max(0, cfg.get("min_command_hold_ms", 250)))
        self.debug_log_period = float(max(0.1, cfg.get("debug_log_period", 0.5)))

        self._tail_filt = 0.0
        self._angle_filt = 0.0
        self._has_filter_state = False
        self._last_valid_ts = None
        self._last_cmd = ("Stop", 0)
        self._last_cmd_ts = 0.0
        self._mode = "forward"
        self._last_debug_ts = 0.0
        self._last_debug = {}

    def compute(self, frame_context, now):
        raw_frame = (frame_context or {}).get("huskylens_frame")
        stale = bool((frame_context or {}).get("huskylens_stale", False))
        metrics = None if stale else compute_tail_offset_and_angle(raw_frame, self.image_center_x)

        if metrics is None:
            if self._last_valid_ts is None:
                self._update_debug(now, raw_frame, valid=False, command=("Stop", 0))
                return "Stop", 0
            if (now - self._last_valid_ts) > self.invalid_timeout_sec:
                cmd = self._apply_hold(("Stop", 0), now)
                self._mode = "invalid"
                self._update_debug(now, raw_frame, valid=False, command=cmd)
                return cmd
            # Hold last known command for short invalid gap.
            self._update_debug(now, raw_frame, valid=False, command=self._last_cmd)
            return self._last_cmd

        self._last_valid_ts = now
        self._apply_ema(metrics["tail_offset_x"], metrics["angle_deg"])
        desired = self._decide_command()
        command = self._apply_hold(desired, now)
        self._update_debug(now, raw_frame, valid=True, command=command, metrics=metrics)
        return command

    def _apply_ema(self, tail_offset_x, angle_deg):
        if not self._has_filter_state:
            self._tail_filt = float(tail_offset_x)
            self._angle_filt = float(angle_deg)
            self._has_filter_state = True
            return
        self._tail_filt = (1.0 - self.alpha_tail) * self._tail_filt + self.alpha_tail * float(tail_offset_x)
        self._angle_filt = (1.0 - self.alpha_angle) * self._angle_filt + self.alpha_angle * float(angle_deg)

    def _decide_command(self):
        angle_abs = abs(self._angle_filt)
        tail_abs = abs(self._tail_filt)

        angle_th = self.angle_deadband_deg
        tail_th = self.tail_deadband_px
        if self._mode == "angle":
            angle_th = max(0.0, self.angle_deadband_deg - self.angle_hysteresis_deg)
        if self._mode == "tail":
            tail_th = max(0.0, self.tail_deadband_px - self.tail_hysteresis_px)

        if angle_abs > angle_th:
            self._mode = "angle"
            if self._angle_filt < 0:
                return "RotateLeft", self.speed_rotate
            return "RotateRight", self.speed_rotate

        if tail_abs > tail_th:
            self._mode = "tail"
            if self._tail_filt < 0:
                return "Left", self.speed_lateral
            return "Right", self.speed_lateral

        self._mode = "forward"
        return "Forward", self.speed_forward

    def _apply_hold(self, desired_cmd, now):
        if desired_cmd == self._last_cmd:
            return self._last_cmd
        elapsed_ms = (now - self._last_cmd_ts) * 1000.0
        if elapsed_ms < float(self.min_command_hold_ms):
            return self._last_cmd
        self._last_cmd = desired_cmd
        self._last_cmd_ts = now
        return desired_cmd

    def _update_debug(self, now, raw_frame, valid, command, metrics=None):
        self._last_debug = {
            "valid": bool(valid),
            "mode": self._mode,
            "command": command[0],
            "speed": int(command[1]),
            "tail_offset_x_raw": None if metrics is None else round(float(metrics["tail_offset_x"]), 3),
            "angle_deg_raw": None if metrics is None else round(float(metrics["angle_deg"]), 3),
            "tail_offset_x_filt": round(float(self._tail_filt), 3),
            "angle_deg_filt": round(float(self._angle_filt), 3),
            "raw_frame": raw_frame,
            "log_due": (now - self._last_debug_ts) >= self.debug_log_period,
        }
        if self._last_debug["log_due"]:
            self._last_debug_ts = now

    def consume_debug(self):
        data = dict(self._last_debug or {})
        if data.get("log_due"):
            data["log_due"] = False
            self._last_debug = dict(data)
            return data
        return None
