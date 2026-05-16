import pytest

pytest.importorskip("launch")
pytest.importorskip("launch_ros")

from robot.robot.main import _node_map


def test_node_map_contains_all_default_bringup_nodes():
    node_map = _node_map()

    assert node_map["line_sensor_driver"] == ("line_sensors", "line_sensor_driver")
    assert node_map["line_follower"] == ("line_follower", "line_follower")
    assert node_map["motor_driver"] == ("motor_driver", "motor_driver")
    assert node_map["manual_control"] == ("manual_control", "manual_control")
    assert node_map["camera_sensor"] == ("camera_sensor", "camera_sensor")
    assert node_map["huskylens_sensor"] == ("huskylens_sensor", "huskylens_sensor")


def test_node_map_excludes_unknown_aliases():
    node_map = _node_map()
    assert "unknown_node" not in node_map
