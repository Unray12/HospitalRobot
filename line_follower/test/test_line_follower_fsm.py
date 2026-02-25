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


def test_follow_centered_line_goes_forward():
    fsm = LineFollowerFSM(base_speed=8, turn_speed_left=6, turn_speed_right=7)
    direction, speed = fsm.update(_frame(0, 3, 0, mid_full=True), now=0.0)
    assert direction == "Forward"
    assert speed == 8


def test_left_bias_rotates_left():
    fsm = LineFollowerFSM(base_speed=8, turn_speed_left=6, turn_speed_right=7)
    direction, speed = fsm.update(_frame(3, 1, 0), now=0.0)
    assert direction == "RotateLeft"
    assert speed == 6


def test_lost_line_stops():
    fsm = LineFollowerFSM(base_speed=8)
    direction, speed = fsm.update(_frame(0, 0, 0), now=0.0)
    assert direction == "Stop"
    assert speed == 0


def test_crossing_without_plan_returns_to_follow():
    fsm = LineFollowerFSM(base_speed=8, crossing_duration=0.5)
    first = fsm.update(_frame(1, 1, 1, left_full=True, mid_full=True, right_full=True), now=1.0)
    assert first == ("Forward", 8)
    second = fsm.update(_frame(0, 3, 0, mid_full=True), now=1.6)
    assert second == ("Forward", 8)


def test_plan_wait_then_follow():
    fsm = LineFollowerFSM(
        base_speed=8,
        cross_plan=[
            {"action": "Wait", "duration": 0.2},
            {"action": "Follow", "duration": 0.3},
        ],
        plan_end_state="follow",
    )
    # Enter cross pre
    fsm.update(_frame(1, 1, 1, True, True, True), now=0.0)
    # Finish cross pre forward + stop phases
    out = fsm.update(_frame(1, 1, 1, True, True, True), now=3.2)
    assert out == ("Stop", 0)
    out = fsm.update(_frame(0, 3, 0, mid_full=True), now=4.3)
    assert out == ("Stop", 0)


def test_rotate_until_line_timeout_moves_next_step():
    fsm = LineFollowerFSM(
        base_speed=8,
        rotate_min_duration=0.1,
        cross_plan=[
            {"action": "RotateRight", "speed": 6, "until": "line", "timeout": 0.3},
            {"action": "Stop"},
        ],
        plan_end_state="stop",
    )
    fsm.update(_frame(1, 1, 1, True, True, True), now=0.0)
    fsm.update(_frame(1, 1, 1, True, True, True), now=3.2)
    out = fsm.update(_frame(0, 1, 3, False, False, False), now=4.3)
    assert out == ("RotateRight", 6)
    out = fsm.update(_frame(0, 1, 3, False, False, False), now=4.7)
    assert out == ("Forward", 8)


def test_plan_goto_label():
    fsm = LineFollowerFSM(
        base_speed=8,
        cross_plan=[
            {"label": "entry", "action": "Wait", "duration": 0.1},
            {"action": "Goto", "target": "finish"},
            {"action": "RotateLeft", "speed": 6, "duration": 0.2},
            {"label": "finish", "action": "Stop"},
        ],
        plan_end_state="stop",
    )
    fsm.update(_frame(1, 1, 1, True, True, True), now=0.0)
    fsm.update(_frame(1, 1, 1, True, True, True), now=3.2)
    out = fsm.update(_frame(0, 2, 0, False, True, False), now=4.3)
    assert out == ("Stop", 0)
