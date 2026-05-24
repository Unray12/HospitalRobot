import json

from robot_common.device_protocol import (
    Envelope,
    EVENT_BOOT,
    EVENT_DATA,
    EVENT_ERROR,
    EVENT_INFO,
    is_silent_error,
    parse_envelope,
)


def _env(dev_id, event, payload=None):
    return json.dumps({"dev_id": dev_id, "event": event, "payload": payload or {}})


# ---- valid ----

def test_parse_data_envelope():
    env, err = parse_envelope(_env("hrbot_line", "data", {"x": 1}))
    assert err is None
    assert env.dev_id == "hrbot_line"
    assert env.event == EVENT_DATA
    assert env.payload == {"x": 1}
    assert env.is_data() is True
    assert env.is_control() is False


def test_parse_boot_envelope():
    env, err = parse_envelope(_env("hrbot_camera", "boot", {"fw": "camera", "ver": 1}))
    assert err is None
    assert env.event == EVENT_BOOT
    assert env.is_data() is False
    assert env.is_control() is True


def test_parse_info_and_error_envelopes():
    for evt in (EVENT_INFO, EVENT_ERROR):
        env, err = parse_envelope(_env("hrbot_camera", evt, {"k": "v"}))
        assert err is None
        assert env.event == evt
        assert env.is_control() is True


def test_role_property():
    env, _ = parse_envelope(_env("hrbot_huskylens", "data"))
    assert env.role == "huskylens"


def test_ts_field_optional():
    raw = json.dumps({"dev_id": "hrbot_line", "event": "data", "payload": {}, "ts": 12345})
    env, err = parse_envelope(raw)
    assert err is None
    assert env.ts == 12345


def test_payload_optional_defaults_to_empty_dict():
    raw = json.dumps({"dev_id": "hrbot_line", "event": "data"})
    env, err = parse_envelope(raw)
    assert err is None
    assert env.payload == {}


def test_expected_dev_id_match():
    env, err = parse_envelope(_env("hrbot_line", "data"), expected_dev_id="hrbot_line")
    assert err is None
    assert env.dev_id == "hrbot_line"


def test_expected_dev_id_mismatch():
    env, err = parse_envelope(_env("hrbot_camera", "data"), expected_dev_id="hrbot_line")
    assert env is None
    assert err == "wrong_dev_id"


# ---- invalid ----

def test_empty_input():
    for raw in (None, "", "   "):
        env, err = parse_envelope(raw)
        assert env is None
        assert err == "empty"


def test_non_json():
    env, err = parse_envelope("not json")
    assert env is None
    assert err == "not_json"


def test_malformed_json():
    env, err = parse_envelope('{"dev_id":')
    assert env is None
    assert err == "not_json"


def test_json_array_not_object():
    env, err = parse_envelope("[1,2,3]")
    assert env is None
    assert err == "not_json"


def test_missing_dev_id():
    env, err = parse_envelope('{"event":"data","payload":{}}')
    assert env is None
    assert err == "missing_dev_id"


def test_missing_event():
    env, err = parse_envelope('{"dev_id":"hrbot_line","payload":{}}')
    assert env is None
    assert err == "missing_event"


def test_unknown_event():
    env, err = parse_envelope('{"dev_id":"hrbot_line","event":"weird","payload":{}}')
    assert env is None
    assert err == "unknown_event"


def test_invalid_payload_type():
    env, err = parse_envelope('{"dev_id":"hrbot_line","event":"data","payload":"oops"}')
    assert env is None
    assert err == "invalid_payload"


# ---- silent error filter ----

def test_is_silent_error():
    assert is_silent_error("empty") is True
    assert is_silent_error("wrong_dev_id") is True
    assert is_silent_error("not_json") is False
    assert is_silent_error("missing_dev_id") is False
    assert is_silent_error(None) is False


# ---- accepts dict directly (tiện cho unit test) ----

def test_accepts_dict_input():
    env, err = parse_envelope({"dev_id": "hrbot_line", "event": "data", "payload": {"a": 1}})
    assert err is None
    assert env.payload == {"a": 1}


# ---- Envelope construction ----

def test_envelope_repr_does_not_crash():
    env = Envelope("hrbot_line", "data", {}, ts=None)
    assert "hrbot_line" in repr(env)
