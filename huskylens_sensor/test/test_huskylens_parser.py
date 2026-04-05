from huskylens_sensor.huskylens_sensor.huskylens_parser import (
    default_frame,
    is_frame_tracking_valid,
    normalize_huskylens_payload,
    to_frame_message,
)


def test_normalize_huskylens_payload_valid_data():
    raw = '{"HuskylenSensor":{"connected":1,"algorithm_set":1,"valid":1,"error":-12,"y_type":1,"line_length_y":84,"direction":0}}'
    frame, err = normalize_huskylens_payload(raw)
    assert err is None
    assert frame["connected"] == 1
    assert frame["algorithm_set"] == 1
    assert frame["valid"] == 1
    assert frame["error"] == -12
    assert frame["line_length_y"] == 84
    assert is_frame_tracking_valid(frame) is True


def test_normalize_huskylens_payload_missing_field():
    raw = '{"HuskylenSensor":{"connected":1,"algorithm_set":1}}'
    frame, err = normalize_huskylens_payload(raw)
    assert frame is None
    assert "missing_fields" in err


def test_normalize_huskylens_payload_malformed_json():
    frame, err = normalize_huskylens_payload("{bad json}")
    assert frame is None
    assert "invalid_json" in err


def test_default_frame_and_message_format():
    frame = default_frame()
    message = to_frame_message(frame)
    assert '"HuskylenSensor"' in message
    assert '"valid":0' in message
