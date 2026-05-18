import pytest

pytest.importorskip("rclpy")

from mqtt_bridge.MQTTBridgeROS import MQTTBridgeNode


class _NodeStub:
    _extract_face_ids = MQTTBridgeNode._extract_face_ids


def test_extract_face_ids_supports_multi_value_payload():
    node = _NodeStub()
    assert node._extract_face_ids("<DEV1,FACE,1,2,3>") == [1, 2, 3]


def test_extract_face_ids_supports_mixed_separators():
    node = _NodeStub()
    assert node._extract_face_ids("<DEV1,FACE,1;2/3|4>") == [1, 2, 3, 4]


def test_extract_face_ids_returns_empty_for_non_face_payload():
    node = _NodeStub()
    assert node._extract_face_ids("<DEV1,NO_OBJECT>") == []
