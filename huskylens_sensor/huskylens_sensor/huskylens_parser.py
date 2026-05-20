"""HuskyLens payload parser dùng HospitalRobot Device Protocol v1."""

import json

from robot_common.device_protocol import (
    parse_envelope,
    is_silent_error,
    EVENT_DATA,
)

DEV_ID = "hrbot_huskylens"

REQUIRED_BASE_FIELDS = (
    "connected",
    "algorithm_set",
    "valid",
)


def normalize_huskylens_payload(raw):
    """
    Parse 1 envelope line từ HuskyLens firmware -> sensor frame dict.

    Returns (frame_dict, error_text):
      - (frame, None)   : parsed OK
      - (None, "skip")  : control/banner frame, bỏ qua thầm lặng
      - (None, "<err>") : parse failure, caller nên log warning
    """
    # Cho phép caller truyền dict trực tiếp (cho test legacy không có envelope).
    if isinstance(raw, dict):
        sensor = raw.get("HuskylenSensor") or raw.get("HuskyLensSensor")
        if not isinstance(sensor, dict):
            return None, "missing_huskylens_sensor_object"
        return _build_frame(sensor)

    envelope, err = parse_envelope(raw, expected_dev_id=DEV_ID)
    if envelope is None:
        if is_silent_error(err):
            return None, "skip"
        return None, f"invalid_json: {err}" if err == "not_json" else err
    if envelope.event != EVENT_DATA:
        return None, "skip"

    sensor = envelope.payload
    # Backward-compat: nếu firmware cũ vẫn nhét {"HuskylenSensor":{...}} trong payload.
    inner = sensor.get("HuskylenSensor") or sensor.get("HuskyLensSensor")
    if isinstance(inner, dict):
        sensor = inner
    if not isinstance(sensor, dict):
        return None, "missing_huskylens_sensor_object"
    return _build_frame(sensor)


def _build_frame(sensor):

    missing = [name for name in REQUIRED_BASE_FIELDS if name not in sensor]
    if missing:
        return None, f"missing_fields: {','.join(missing)}"

    try:
        frame = {
            "connected": _to_flag(sensor.get("connected")),
            "algorithm_set": _to_flag(sensor.get("algorithm_set")),
            "valid": _to_flag(sensor.get("valid")),
            "y_type": int(sensor.get("y_type", 0)),
            "line_length_y": max(0, int(sensor.get("line_length_y", 0))),
            "direction": int(sensor.get("direction", 0)),
        }

        if "tail_offset_x" in sensor:
            frame["tail_offset_x"] = float(sensor.get("tail_offset_x"))
        if "angle_deg" in sensor:
            frame["angle_deg"] = float(sensor.get("angle_deg"))
        if "error" in sensor:
            frame["error"] = int(sensor.get("error"))
    except Exception as exc:
        return None, f"invalid_field_type: {exc}"

    if ("tail_offset_x" not in frame or "angle_deg" not in frame) and ("error" not in frame):
        return None, "missing_fields: tail_offset_x_angle_deg_or_error"

    return frame, None


def is_frame_tracking_valid(frame):
    if not isinstance(frame, dict):
        return False
    return bool(frame.get("connected")) and bool(frame.get("algorithm_set")) and bool(frame.get("valid"))


def to_frame_message(frame):
    if frame is None:
        frame = default_frame()
    return json.dumps({"HuskylenSensor": frame}, ensure_ascii=False, separators=(",", ":"))


def default_frame():
    return {
        "connected": 0,
        "algorithm_set": 0,
        "valid": 0,
        "tail_offset_x": 0.0,
        "angle_deg": 0.0,
        "y_type": 0,
        "line_length_y": 0,
        "direction": 0,
    }


def _to_flag(value):
    try:
        return 1 if int(value) != 0 else 0
    except Exception:
        return 1 if bool(value) else 0
