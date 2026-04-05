import json


REQUIRED_BASE_FIELDS = (
    "connected",
    "algorithm_set",
    "valid",
)


def normalize_huskylens_payload(raw):
    """
    Normalize raw HuskyLens JSON payload into a strict schema.
    Returns (frame_dict, error_text). error_text is None on success.
    """
    payload = raw
    if isinstance(raw, str):
        try:
            payload = json.loads(raw)
        except Exception as exc:
            return None, f"invalid_json: {exc}"

    if not isinstance(payload, dict):
        return None, "payload_not_object"

    sensor = payload.get("HuskylenSensor")
    if sensor is None:
        sensor = payload.get("HuskyLensSensor")
    if not isinstance(sensor, dict):
        return None, "missing_huskylens_sensor_object"

    missing = [name for name in REQUIRED_BASE_FIELDS if name not in sensor]
    if missing:
        return None, f"missing_fields: {','.join(missing)}"

    try:
        if "error" in sensor:
            error = int(sensor.get("error"))
        elif "tail_offset_x" in sensor and "angle_deg" in sensor:
            tail_offset_x = float(sensor.get("tail_offset_x"))
            angle_deg = float(sensor.get("angle_deg"))
            error = int(round(0.8 * tail_offset_x + 0.2 * angle_deg))
        else:
            return None, "missing_fields: error_or_tail_offset_x_angle_deg"

        frame = {
            "connected": _to_flag(sensor.get("connected")),
            "algorithm_set": _to_flag(sensor.get("algorithm_set")),
            "valid": _to_flag(sensor.get("valid")),
            "error": int(error),
            "y_type": int(sensor.get("y_type", 0)),
            "line_length_y": max(0, int(sensor.get("line_length_y", 0))),
            "direction": int(sensor.get("direction", 0)),
        }
    except Exception as exc:
        return None, f"invalid_field_type: {exc}"

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
        "error": 0,
        "y_type": 0,
        "line_length_y": 0,
        "direction": 0,
    }


def _to_flag(value):
    try:
        return 1 if int(value) != 0 else 0
    except Exception:
        return 1 if bool(value) else 0
