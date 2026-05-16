from line_sensors.line_sensors.line_sensor_reader import LineSensorReader


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


def test_parse_payload_complete_data():
    reader = _new_reader()
    payload = {
        "LineSensor": {
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


def test_parse_payload_missing_key_returns_none():
    reader = _new_reader()
    assert reader.parse_payload({"foo": {}}) is None


def test_parse_payload_empty_sensor_dict_not_full_black():
    reader = _new_reader()
    payload = {"LineSensor": {"0x25": {}, "0x24": {}, "0x23": {}}}
    frame = reader.parse_payload(payload)
    assert frame["left_full"] is False
    assert frame["mid_full"] is False
    assert frame["right_full"] is False


def test_read_frame_skips_corrupted_json():
    reader = _new_reader()
    reader.ser = _FakeSerial(b"{bad json}\n")
    assert reader.read_frame() is None


def test_count_active_tolerates_mixed_types():
    reader = _new_reader()
    assert reader._count_active({"s1": 1, "s2": "1", "s3": True, "s4": 0, "s5": "bad"}) == 3


def test_is_full_black_accepts_supported_active_types():
    reader = _new_reader()
    assert reader._is_full_black({"s1": 1, "s2": "1", "s3": True}) is True


def test_is_full_black_rejects_invalid_active_values():
    reader = _new_reader()
    assert reader._count_active({"s1": 2, "s2": -1, "s3": "0", "s4": 1}) == 1
    assert reader._is_full_black({"s1": 2, "s2": 1}) is False


def test_read_frame_prefers_latest_complete_line_in_burst():
    reader = _new_reader()
    reader.ser = _FakeSerial(
        b'{"LineSensor":{"0x25":{"s1":1},"0x24":{"s1":0},"0x23":{"s1":0}}}\n'
        b'{"LineSensor":{"0x25":{"s1":0},"0x24":{"s1":1},"0x23":{"s1":1}}}\n'
    )
    frame = reader.read_frame()
    assert frame["left_count"] == 0
    assert frame["mid_count"] == 1
    assert frame["right_count"] == 1


def test_parse_payload_non_numeric_values_do_not_crash():
    reader = _new_reader()
    payload = {
        "LineSensor": {
            "0x25": {"s1": "bad", "s2": 1},
            "0x24": {"s1": None},
            "0x23": {"s1": True},
        }
    }
    frame = reader.parse_payload(payload)
    assert frame["left_count"] == 1
    assert frame["mid_count"] == 0
    assert frame["right_count"] == 1


def test_read_frame_preserves_partial_tail_in_buffer():
    reader = _new_reader()
    reader.ser = _FakeSerial(
        b'{"LineSensor":{"0x25":{"s1":1},"0x24":{"s1":0},"0x23":{"s1":0}}}\npartial'
    )
    frame = reader.read_frame()
    assert frame["left_count"] == 1
    assert reader._buffer == "partial"
