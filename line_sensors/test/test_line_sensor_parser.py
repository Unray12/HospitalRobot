import json

from line_sensors.line_sensor_reader import LineSensorReader


class _FakeSerial:
    def __init__(self, payload):
        self.payload = payload
        self.is_open = True
        self.in_waiting = len(payload)

    def read(self, _):
        data = self.payload
        self.payload = b""
        self.in_waiting = 0
        return data


def _new_reader():
    reader = LineSensorReader.__new__(LineSensorReader)
    reader._logger = None
    reader._buffer = ""
    reader.ser = None
    return reader


def _env(event, payload):
    return json.dumps({"dev_id": "hrbot_line", "event": event, "payload": payload})


# ----- parse_payload (legacy direct dict) -----

def test_parse_payload_complete_data():
    reader = _new_reader()
    payload = {
        "sensors": {
            "0x25": {"s1": 1, "s2": 1},
            "0x24": {"s1": 1, "s2": 0},
            "0x23": {"s1": 0, "s2": 0},
        }
    }
    frame = reader.parse_payload(payload)
    assert frame["left_count"] == 2
    assert frame["mid_count"] == 1
    assert frame["right_count"] == 0
    assert frame["left_full"] is True
    assert frame["mid_full"] is False
    assert frame["right_full"] is False


def test_parse_payload_legacy_flat_dict_still_works():
    reader = _new_reader()
    # Shape cũ không có "sensors" wrapper
    payload = {
        "0x25": {"s1": 1, "s2": 1},
        "0x24": {"s1": 1, "s2": 0},
        "0x23": {"s1": 0, "s2": 0},
    }
    frame = reader.parse_payload(payload)
    assert frame is not None
    assert frame["left_count"] == 2


def test_parse_payload_empty_sensors_returns_none():
    reader = _new_reader()
    assert reader.parse_payload({"sensors": {}}) is None


def test_parse_payload_non_dict_returns_none():
    reader = _new_reader()
    assert reader.parse_payload("nope") is None


# ----- parse_line (envelope) -----

def test_parse_line_data_envelope():
    reader = _new_reader()
    raw = _env("data", {"sensors": {
        "0x25": {"s1": 1},
        "0x24": {"s1": 0},
        "0x23": {"s1": 1},
    }})
    frame = reader.parse_line(raw)
    assert frame is not None
    assert frame["left_count"] == 1
    assert frame["right_count"] == 1


def test_parse_line_boot_banner_skipped_silently():
    reader = _new_reader()
    assert reader.parse_line(_env("boot", {"fw": "line", "ver": 1})) is None


def test_parse_line_wrong_dev_id_skipped_silently():
    reader = _new_reader()
    raw = json.dumps({"dev_id": "hrbot_camera", "event": "data", "payload": {}})
    assert reader.parse_line(raw) is None


def test_parse_line_garbage_returns_none():
    reader = _new_reader()
    assert reader.parse_line("garbage") is None


# ----- read_frame integration -----

def test_read_frame_skips_corrupted_json():
    reader = _new_reader()
    reader.ser = _FakeSerial(b"{bad json}\n")
    assert reader.read_frame() is None


def test_read_frame_prefers_latest_complete_envelope():
    reader = _new_reader()
    a = _env("data", {"sensors": {"0x25": {"s1": 1}, "0x24": {"s1": 0}, "0x23": {"s1": 0}}})
    b = _env("data", {"sensors": {"0x25": {"s1": 0}, "0x24": {"s1": 1}, "0x23": {"s1": 1}}})
    reader.ser = _FakeSerial((a + "\n" + b + "\n").encode())
    frame = reader.read_frame()
    assert frame["left_count"] == 0
    assert frame["mid_count"] == 1
    assert frame["right_count"] == 1


def test_count_active_tolerates_mixed_types():
    reader = _new_reader()
    assert reader._count_active({"s1": 1, "s2": "1", "s3": True, "s4": 0, "s5": "bad"}) == 3


def test_is_full_black_rejects_invalid_active_values():
    reader = _new_reader()
    assert reader._is_full_black({"s1": 2, "s2": 1}) is False
