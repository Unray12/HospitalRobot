import json

from huskylens_sensor.huskylens_parser import (
    default_frame,
    is_frame_tracking_valid,
    normalize_huskylens_payload,
    to_frame_message,
)


def _env(event, payload):
    return json.dumps({"dev_id": "hrbot_huskylens", "event": event, "payload": payload})


def _data_payload(**overrides):
    base = {
        "connected": 1,
        "algorithm_set": 1,
        "valid": 1,
        "tail_offset_x": -72,
        "angle_deg": 4.12,
        "y_type": 1,
        "line_length_y": 84,
        "direction": 0,
    }
    base.update(overrides)
    return base


# ----- envelope parsing -----

def test_envelope_data_parses_ok():
    raw = _env("data", _data_payload())
    frame, err = normalize_huskylens_payload(raw)
    assert err is None
    assert frame["connected"] == 1
    assert frame["valid"] == 1
    assert frame["tail_offset_x"] == -72.0
    assert frame["angle_deg"] == 4.12
    assert frame["error"] == -57
    assert is_frame_tracking_valid(frame) is True


def test_envelope_boot_banner_is_skip():
    frame, err = normalize_huskylens_payload(_env("boot", {"fw": "huskylens", "ver": 1}))
    assert frame is None
    assert err == "skip"


def test_envelope_info_event_is_skip():
    frame, err = normalize_huskylens_payload(_env("info", {"msg": "ok"}))
    assert frame is None
    assert err == "skip"


def test_envelope_wrong_dev_id_is_skip():
    raw = json.dumps({"dev_id": "hrbot_line", "event": "data", "payload": _data_payload()})
    frame, err = normalize_huskylens_payload(raw)
    assert frame is None
    assert err == "skip"


def test_envelope_missing_required_field():
    raw = _env("data", {"connected": 1, "algorithm_set": 1})
    frame, err = normalize_huskylens_payload(raw)
    assert frame is None
    assert "missing_fields" in err


def test_envelope_malformed_json():
    frame, err = normalize_huskylens_payload("{bad json}")
    assert frame is None
    assert "invalid_json" in err


def test_envelope_with_error_field_accepted():
    raw = _env("data", {
        "connected": 1, "algorithm_set": 1, "valid": 1,
        "error": -57, "y_type": 1, "line_length_y": 222, "direction": -4,
    })
    frame, err = normalize_huskylens_payload(raw)
    assert err is None
    assert frame["error"] == -57


# ----- legacy direct-dict support (cho test cũ + firmware cũ) -----

def test_legacy_huskylens_sensor_wrapper_still_parses():
    raw = {"HuskylenSensor": _data_payload()}
    frame, err = normalize_huskylens_payload(raw)
    assert err is None
    assert frame["valid"] == 1


# ----- helpers -----

def test_default_frame_and_message_format():
    frame = default_frame()
    msg = to_frame_message(frame)
    assert '"HuskylenSensor"' in msg
    assert '"valid":0' in msg
