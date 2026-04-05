from line_follower.line_follower.huskylens_raw_controller import (
    HuskyLensRawController,
    compute_tail_offset_and_angle,
)


def _ctx(frame, stale=False):
    return {"huskylens_frame": frame, "huskylens_stale": stale}


def test_compute_tail_offset_and_angle_basic():
    frame = {
        "connected": 1,
        "algorithm_set": 1,
        "x_tail": 170,
        "y_tail": 220,
        "x_head": 170,
        "y_head": 200,
    }
    out = compute_tail_offset_and_angle(frame, image_center_x=160)
    assert out is not None
    assert out["tail_offset_x"] == 10.0
    assert abs(out["angle_deg"]) < 1e-6


def test_compute_tail_offset_and_angle_invalid_when_not_connected():
    frame = {
        "connected": 0,
        "algorithm_set": 1,
        "x_tail": 170,
        "y_tail": 220,
        "x_head": 170,
        "y_head": 200,
    }
    assert compute_tail_offset_and_angle(frame, image_center_x=160) is None


def test_control_prioritizes_angle_before_lateral():
    c = HuskyLensRawController(
        {
            "alpha_tail": 1.0,
            "alpha_angle": 1.0,
            "tail_deadband_px": 3.0,
            "angle_deadband_deg": 1.0,
            "min_command_hold_ms": 0,
        }
    )
    frame = {
        "connected": 1,
        "algorithm_set": 1,
        "x_tail": 180,
        "y_tail": 220,
        "x_head": 200,
        "y_head": 200,
    }
    cmd = c.compute(_ctx(frame), now=1.0)
    assert cmd[0] == "RotateRight"


def test_control_lateral_when_angle_is_within_deadband():
    c = HuskyLensRawController(
        {
            "alpha_tail": 1.0,
            "alpha_angle": 1.0,
            "tail_deadband_px": 3.0,
            "angle_deadband_deg": 1.0,
            "min_command_hold_ms": 0,
        }
    )
    frame = {
        "connected": 1,
        "algorithm_set": 1,
        "x_tail": 145,
        "y_tail": 220,
        "x_head": 145,
        "y_head": 200,
    }
    cmd = c.compute(_ctx(frame), now=1.0)
    assert cmd[0] == "Left"


def test_control_forward_when_angle_and_tail_in_deadband():
    c = HuskyLensRawController(
        {
            "alpha_tail": 1.0,
            "alpha_angle": 1.0,
            "tail_deadband_px": 5.0,
            "angle_deadband_deg": 1.0,
            "min_command_hold_ms": 0,
        }
    )
    frame = {
        "connected": 1,
        "algorithm_set": 1,
        "x_tail": 160,
        "y_tail": 220,
        "x_head": 160,
        "y_head": 200,
    }
    cmd = c.compute(_ctx(frame), now=1.0)
    assert cmd[0] == "Forward"


def test_invalid_timeout_forces_stop():
    c = HuskyLensRawController(
        {
            "alpha_tail": 1.0,
            "alpha_angle": 1.0,
            "invalid_timeout_sec": 0.3,
            "min_command_hold_ms": 0,
        }
    )
    valid_frame = {
        "connected": 1,
        "algorithm_set": 1,
        "x_tail": 160,
        "y_tail": 220,
        "x_head": 160,
        "y_head": 200,
    }
    assert c.compute(_ctx(valid_frame), now=1.0)[0] == "Forward"
    # short invalid gap -> hold previous command
    assert c.compute(_ctx(None), now=1.1)[0] == "Forward"
    # timeout exceeded -> stop
    assert c.compute(_ctx(None), now=1.5)[0] == "Stop"


def test_min_command_hold_prevents_rapid_flip():
    c = HuskyLensRawController(
        {
            "alpha_tail": 1.0,
            "alpha_angle": 1.0,
            "tail_deadband_px": 3.0,
            "angle_deadband_deg": 1.0,
            "min_command_hold_ms": 300,
        }
    )
    frame_right = {
        "connected": 1,
        "algorithm_set": 1,
        "x_tail": 180,
        "y_tail": 220,
        "x_head": 180,
        "y_head": 200,
    }
    frame_left = {
        "connected": 1,
        "algorithm_set": 1,
        "x_tail": 140,
        "y_tail": 220,
        "x_head": 140,
        "y_head": 200,
    }
    first = c.compute(_ctx(frame_right), now=1.0)
    assert first[0] == "Right"
    # not enough hold time -> keep previous command
    second = c.compute(_ctx(frame_left), now=1.1)
    assert second[0] == "Right"
    # enough hold time -> accept new command
    third = c.compute(_ctx(frame_left), now=1.5)
    assert third[0] == "Left"
