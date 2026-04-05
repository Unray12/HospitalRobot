from line_follower.line_follower.line_follower import LineFollowerFSM


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
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": -6, "angle_deg": 10},
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
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": 2.5, "angle_deg": 2.9},
    )
    direction, speed = fsm.update(line_frame, now=0.0)
    assert direction == "Forward"
    assert speed == 8


def test_huskylens_strategy_invalid_frame_fallback_line_sensor_when_not_strict():
    fsm = LineFollowerFSM(
        tracking_strategy="huskylens",
        tracking_strict_mode=False,
        base_speed=8,
        turn_speed_left=6,
        turn_speed_right=7,
    )
    line_frame = _frame(0, 0, 3)
    _set_context(
        fsm,
        line_sensor_frame=line_frame,
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1},
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
    direction, speed = fsm.update(_frame(0, 3, 0, mid_full=True), now=0.0)
    assert direction == "Stop"
    assert speed == 0


def test_huskylens_strategy_tail_offset_used_when_angle_in_safe_zone():
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
        huskylens_frame={"connected": 1, "algorithm_set": 1, "valid": 1, "tail_offset_x": 4.2, "angle_deg": 1.0},
    )
    direction, speed = fsm.update(line_frame, now=0.0)
    assert direction == "Left"
    assert speed == 6


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
