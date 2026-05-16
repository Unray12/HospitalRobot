import json

import pytest

pytest.importorskip("rclpy")

from camera_sensor.camera_sensor.main import CameraSensorNode


class _NodeStub:
    log = None
    malformed_log_every = 1
    _drop_count = 0
    _normalize_face_payload = CameraSensorNode._normalize_face_payload


def _env(event, payload):
    return json.dumps({"dev_id": "hrbot_camera", "event": event, "payload": payload})


def test_face_single_id():
    n = _NodeStub()
    assert n._normalize_face_payload(_env("data", {"kind": "face", "ids": [1]})) == "<DEV1,FACE,1>"


def test_face_multiple_ids():
    n = _NodeStub()
    raw = _env("data", {"kind": "face", "ids": [1, 2, 3]})
    assert n._normalize_face_payload(raw) == "<DEV1,FACE,1,2,3>"


def test_face_empty_ids_defaults_zero():
    n = _NodeStub()
    raw = _env("data", {"kind": "face", "ids": []})
    assert n._normalize_face_payload(raw) == "<DEV1,FACE,0>"


def test_no_object():
    n = _NodeStub()
    assert n._normalize_face_payload(_env("data", {"kind": "no_object"})) == "<DEV1,NO_OBJECT>"


def test_boot_banner_ignored_silently():
    n = _NodeStub()
    assert n._normalize_face_payload(_env("boot", {"fw": "camera", "ver": 1})) is None
    assert n._drop_count == 0


def test_info_event_ignored_silently():
    n = _NodeStub()
    assert n._normalize_face_payload(_env("info", {"msg": "ok"})) is None
    assert n._drop_count == 0


def test_error_event_ignored_silently():
    n = _NodeStub()
    assert n._normalize_face_payload(_env("error", {"code": "x"})) is None
    assert n._drop_count == 0


def test_wrong_dev_id_silently_ignored():
    n = _NodeStub()
    raw = json.dumps({"dev_id": "hrbot_line", "event": "data", "payload": {"kind": "face", "ids": [1]}})
    assert n._normalize_face_payload(raw) is None


def test_legacy_text_format_rejected():
    n = _NodeStub()
    assert n._normalize_face_payload("<DEV1,FACE,1>") is None


def test_unknown_kind_ignored():
    n = _NodeStub()
    raw = _env("data", {"kind": "mystery"})
    assert n._normalize_face_payload(raw) is None
