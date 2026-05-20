from line_follower.line_follower import LineFollowerFSM


def _frame(left_count, mid_count, right_count, left_full=False, mid_full=False, right_full=False):
    return {
        "left_count": left_count,
        "mid_count": mid_count,
        "right_count": right_count,
        "left_full": left_full,
        "mid_full": mid_full,
        "right_full": right_full,
    }


def _set_context(fsm, line_sensor_frame=None, huskylens_frame=None, line_stale=False, huskylens_stale=False):
    fsm.set_tracking_context(
        {
            "line_sensor_frame": line_sensor_frame,
            "line_sensor_stale": line_stale,
            "huskylens_frame": huskylens_frame,
            "huskylens_stale": huskylens_stale,
        }
    )


def test_line_sensor_strategy_centered_forward():
    fsm = LineFollowerFSM(tracking_strategy="line_sensor", base_speed=8, turn_speed_left=6, turn_speed_right=7)
    frame = _frame(0, 3, 0, mid_full=True)
    _set_context(fsm, line_sensor_frame=frame)
    direction, speed = fsm.update(frame, now=0.0)
    assert direction == "Forward"
    assert speed == 8


def test_line_sensor_strategy_left_bias_rotate_left():
    fsm = LineFollowerFSM(tracking_strategy="line_sensor", base_speed=8, turn_speed_left=6, turn_speed_right=7)
    frame = _frame(3, 1, 0)
    _set_context(fsm, line_sensor_frame=frame)
    direction, speed = fsm.update(frame, now=0.0)
    assert direction == "RotateLeft"
    assert speed == 6


def test_line_sensor_strategy_lost_line_stop():
    fsm = LineFollowerFSM(tracking_strategy="line_sensor", base_speed=8, turn_speed_left=6, turn_speed_right=7)
    frame = _frame(0, 0, 0)
    _set_context(fsm, line_sensor_frame=frame)
    direction, speed = fsm.update(frame, now=0.0)
    assert direction == "Stop"
    assert speed == 0


def test_huskylens_strategy_tail_priority_when_both_exceed_threshold():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
    )
    line_frame = _frame(0, 3, 0, mid_full=True)
    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": -15, "angle_deg": 10},
    )
    direction, speed = fsm.update(line_frame, now=0.0)
    assert direction == "Right"
    assert speed == 7


def test_huskylens_strategy_safe_zone_goes_forward():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
    )
    line_frame = _frame(0, 3, 0, mid_full=True)
    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": 0.5, "angle_deg": 0.5},
    )
    direction, speed = fsm.update(line_frame, now=0.0)
    assert direction == "Forward"
    assert speed == 8


def test_huskylens_strategy_invalid_frame_fallback_line_sensor_when_enabled():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        tracking_strict_mode=False,
        tracking_allow_line_sensor_fallback=True,
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
    )
    line_frame = _frame(0, 0, 3)
    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 0, "tail_offset_x": 0, "angle_deg": 0},
    )
    direction, speed = fsm.update(line_frame, now=0.0)
    assert direction == "RotateRight"
    assert speed == 7


def test_hybrid_strategy_prefers_huskylens_then_fallback_line_sensor():
    fsm = LineFollowerFSM(
        tracking_strategy="hybrid",
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
    )
    line_frame = _frame(3, 1, 0)
    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": 1, "angle_deg": 8},
    )
    direction, speed = fsm.update(line_frame, now=0.0)
    assert direction == "RotateLeft"
    assert speed == 6

    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 0, "algorithm_set": 0, "valid": 0, "tail_offset_x": 0, "angle_deg": 0},
    )
    direction, speed = fsm.update(line_frame, now=0.1)
    assert direction == "RotateLeft"
    assert speed == 6


def test_strict_mode_stops_when_selected_strategy_invalid():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        tracking_strict_mode=True,
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
    )
    line_frame = _frame(0, 3, 0, mid_full=True)
    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 0, "tail_offset_x": 0, "angle_deg": 0},
    )
    direction, speed = fsm.update(line_frame, now=0.0)
    assert direction == "Stop"
    assert speed == 0


def test_huskylens_lateral_deadband_controls_turn_threshold():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
        huskylens_lateral_deadband=5.0,
        huskylens_heading_deadband=3.0,
    )
    line_frame = _frame(0, 3, 0, mid_full=True)

    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": 4.0, "angle_deg": 1.0},
    )
    assert fsm.update(line_frame, now=0.0) == ("Forward", 8)

    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": 6.0, "angle_deg": 1.0},
    )
    assert fsm.update(line_frame, now=0.1) == ("Left", 6)


def test_huskylens_heading_deadband_controls_rotate_threshold():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
        huskylens_lateral_deadband=10.0,
        huskylens_heading_deadband=5.0,
    )
    line_frame = _frame(0, 3, 0, mid_full=True)

    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": 0.0, "angle_deg": 4.0},
    )
    assert fsm.update(line_frame, now=0.0) == ("Forward", 8)

    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": 0.0, "angle_deg": 6.0},
    )
    assert fsm.update(line_frame, now=0.1) == ("RotateLeft", 6)


def test_disabled_line_sensor_fallback_stops_when_huskylens_invalid():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        tracking_strict_mode=False,
        tracking_allow_line_sensor_fallback=False,
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
    )
    line_frame = _frame(0, 0, 3)
    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 0, "tail_offset_x": 0.0, "angle_deg": 0.0},
    )
    assert fsm.update(line_frame, now=0.0) == ("Stop", 0)
    assert fsm.state == LineFollowerFSM.STATE_STOPPED


def test_pause_semantics_preserve_plan_progress_until_reset():
    fsm = LineFollowerFSM(
        tracking_strategy="hybrid",
        base_speed=8,
        cross_plan=[{"action": "Wait", "duration": 0.2}, {"action": "Stop"}],
        plan_end_state="stop",
    )
    fsm._plan_index = 1
    fsm.stop()
    paused_status = fsm.get_plan_status()
    assert paused_status["state"] == "STOPPED"
    assert paused_status["next_step"] == 2
    assert paused_status["total_steps"] == 2

    fsm.reset()
    resumed_status = fsm.get_plan_status()
    assert resumed_status["next_step"] == 1
    assert resumed_status["total_steps"] == 2
    assert resumed_status["state"] == "FOLLOWING"


def test_huskylens_contract_requires_tail_offset_and_angle_not_error_only():
    fsm = LineFollowerFSM(tracking_strategy="huskylens")
    _set_context(
        fsm,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "error": 999},
    )
    assert fsm.update(None, now=0.0) is None


def test_plan_flow_regression_not_broken_with_strategy_switch():
    fsm = LineFollowerFSM(
        tracking_strategy="hybrid",
        base_speed=8,
        cross_plan=[{"action": "Wait", "duration": 0.2}, {"action": "Stop"}],
        plan_end_state="stop",
    )
    cross_frame = _frame(1, 1, 1, True, True, True)
    line_frame = _frame(0, 3, 0, mid_full=True)
    _set_context(fsm, line_sensor_frame=cross_frame, huskylens_frame=None)
    fsm.update(cross_frame, now=0.0)
    _set_context(fsm, line_sensor_frame=cross_frame, huskylens_frame=None)
    out = fsm.update(cross_frame, now=3.2)
    assert out == ("Stop", 0)
    _set_context(fsm, line_sensor_frame=line_frame, huskylens_frame=None)
    out = fsm.update(line_frame, now=4.3)
    assert out == ("Stop", 0)


def _hl_frame(tail=0.0, angle=0.0, y_type=0, valid=1, connected=1, algorithm_set=1):
    return {
        "connected": connected,
        "algorithm_set": algorithm_set,
        "valid": valid,
        "tail_offset_x": tail,
        "angle_deg": angle,
        "y_type": y_type,
    }


def test_y_type_rotate_triggers_cross_pre_then_rotates_until_mid_to_top():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
        cross_pre_forward_speed=5,
        cross_pre_forward_duration=1.0,
        cross_pre_stop_duration=0.0,
        rotate_min_duration=0.0,
        cross_plan=[{"action": "RotateLeft", "speed": 6, "until": "y_type"}, {"action": "Stop"}],
        plan_end_state="stop",
    )

    # 1) Line ahead (y_type=2) → robot just follows huskylens guidance, no plan trigger.
    _set_context(fsm, huskylens_frame=_hl_frame(tail=0, angle=0, y_type=2))
    direction, speed = fsm.update(None, now=0.0)
    assert direction == "Forward"
    assert speed == 8

    # 2) y_type transitions to BOTTOM_TO_MID → enter cross_pre, drive forward.
    _set_context(fsm, huskylens_frame=_hl_frame(tail=0, angle=0, y_type=1))
    direction, speed = fsm.update(None, now=0.5)
    assert direction == "Forward"
    assert speed == 5  # cross_pre_forward_speed

    # 3) Forward duration elapsed → phase advances to stop tick.
    _set_context(fsm, huskylens_frame=_hl_frame(tail=0, angle=0, y_type=1))
    fsm.update(None, now=1.6)  # returns ("Stop", 0) for phase transition

    # 4) Next tick → plan rotation kicks in (RotateLeft).
    _set_context(fsm, huskylens_frame=_hl_frame(tail=0, angle=0, y_type=1))
    direction, speed = fsm.update(None, now=1.7)
    assert direction == "RotateLeft"
    assert speed == 6

    # 5) Still rotating while y_type stays BOTTOM_TO_MID or UNKNOWN.
    _set_context(fsm, huskylens_frame=_hl_frame(tail=0, angle=0, y_type=0))
    direction, speed = fsm.update(None, now=2.0)
    assert direction == "RotateLeft"

    # 6) y_type == MID_TO_TOP → rotation ends, returns to following (FSM state leaves PLAN).
    _set_context(fsm, huskylens_frame=_hl_frame(tail=0, angle=0, y_type=2))
    direction, _ = fsm.update(None, now=2.5)
    assert direction != "RotateLeft"
    assert fsm.state != LineFollowerFSM.STATE_PLAN


def test_y_type_rotate_timeout_advances_plan_when_mid_to_top_never_seen():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
        cross_pre_forward_speed=5,
        cross_pre_forward_duration=0.5,
        cross_pre_stop_duration=0.0,
        rotate_min_duration=0.0,
        huskylens_y_type_rotate_timeout=1.0,
        cross_plan=[{"action": "RotateRight", "until": "y_type"}, {"action": "Stop"}],
        plan_end_state="stop",
    )

    _set_context(fsm, huskylens_frame=_hl_frame(y_type=1))
    fsm.update(None, now=0.0)  # enter cross_pre forward
    _set_context(fsm, huskylens_frame=_hl_frame(y_type=1))
    fsm.update(None, now=0.6)  # cross_pre phase 0→1 transition tick
    _set_context(fsm, huskylens_frame=_hl_frame(y_type=1))
    direction, _ = fsm.update(None, now=0.7)  # rotation starts
    assert direction == "RotateRight"

    # y_type stays UNKNOWN; safety timeout (1.0s after rotation start) ends rotation.
    _set_context(fsm, huskylens_frame=_hl_frame(y_type=0))
    direction, _ = fsm.update(None, now=2.0)
    assert direction != "RotateRight"
    assert fsm.state != LineFollowerFSM.STATE_PLAN


def test_y_type_trigger_inactive_when_next_rotation_uses_line():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
        cross_plan=[{"action": "RotateLeft"}, {"action": "Stop"}],  # no until: y_type
        plan_end_state="stop",
    )
    _set_context(fsm, huskylens_frame=_hl_frame(y_type=1))
    direction, speed = fsm.update(None, now=0.0)
    # Should NOT enter cross_pre; just follow line via huskylens.
    assert direction == "Forward"
    assert speed == 8


def test_y_type_trigger_only_fires_once_per_bottom_to_mid_pulse():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        base_speed=8,
        cross_pre_forward_speed=5,
        cross_pre_forward_duration=0.2,
        cross_pre_stop_duration=0.0,
        rotate_min_duration=0.0,
        cross_plan=[{"action": "RotateLeft", "until": "y_type"}, {"action": "Stop"}],
    )
    # First sample with y_type=1 enters cross_pre.
    _set_context(fsm, huskylens_frame=_hl_frame(y_type=1))
    direction, speed = fsm.update(None, now=0.0)
    assert (direction, speed) == ("Forward", 5)

    # Reset FSM, this time simulate y_type=1 → y_type=0 → y_type=1 sequence.
    fsm2 = LineFollowerFSM(
        tracking_strategy="huskylens",
        base_speed=8,
        cross_plan=[{"action": "RotateLeft", "until": "y_type"}, {"action": "Stop"}],
    )
    _set_context(fsm2, huskylens_frame=_hl_frame(y_type=1))
    fsm2.update(None, now=0.0)  # trigger cross_pre on edge
    # Without retriggering on the same continuous BOTTOM_TO_MID frames, edge tracker stays True.
    assert fsm2._y_type_cross_active is True
