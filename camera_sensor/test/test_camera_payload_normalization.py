import pytest

pytest.importorskip("rclpy")

from camera_sensor.camera_sensor.main import CameraSensorNode


class _NodeStub:
    _normalize_face_payload = CameraSensorNode._normalize_face_payload
    _normalize_device = CameraSensorNode._normalize_device
    _normalize_state = CameraSensorNode._normalize_state
    _extract_face_ids = CameraSensorNode._extract_face_ids


def test_normalize_face_payload_single_id():
    node = _NodeStub()
    assert node._normalize_face_payload("<DEV1,FACE,1>") == "<DEV1,FACE,1>"


def test_normalize_face_payload_multiple_ids():
    node = _NodeStub()
    assert node._normalize_face_payload("<DEV1,FACE,1;2;3>") == "<DEV1,FACE,1,2,3>"


def test_normalize_face_payload_multiple_separators():
    node = _NodeStub()
    assert node._normalize_face_payload("<DEV1,FACE,1|2/3>") == "<DEV1,FACE,1,2,3>"


def test_normalize_face_payload_no_object():
    node = _NodeStub()
    assert node._normalize_face_payload("<DEV1,NO_OBJECT>") == "<DEV1,NO_OBJECT>"


def test_normalize_face_payload_missing_digits_defaults_zero():
    node = _NodeStub()
    assert node._normalize_face_payload("<DEV1,FACE,UNKNOWN>") == "<DEV1,FACE,0>"
