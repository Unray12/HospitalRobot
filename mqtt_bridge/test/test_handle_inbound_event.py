"""Tests for ``MQTTBridgeNode._handle_inbound_event`` using a stand-in node.

These exercises ROS-bridge routing logic without booting rclpy/paho. We construct
a SimpleNamespace, bind the unbound methods, and stub the ROS publishers + MQTT
client so each branch can be asserted against captured calls.
"""

"""Tests for ``MQTTBridgeNode._handle_inbound_event`` using a stand-in node.

These exercises ROS-bridge routing logic without booting rclpy/paho. We construct
a SimpleNamespace, bind the unbound methods, and stub the ROS publishers + MQTT
client so each branch can be asserted against captured calls.
"""

import sys
import types

import pytest

pytest.importorskip("rclpy", reason="rclpy not installed (e.g. Windows dev)")
pytest.importorskip("paho.mqtt.client", reason="paho-mqtt not installed")

from mqtt_bridge.MQTTBridgeROS import MQTTBridgeNode  # noqa: E402
from mqtt_bridge.plan_resolver import PlanCommandResolver  # noqa: E402


class _FakePub:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeMqttClient:
    def __init__(self):
        self.published = []

    def publish(self, topic, payload):
        self.published.append((topic, payload))


class _FakeLog:
    def __init__(self):
        self.entries = []

    def info(self, msg, event=None, **kw):
        self.entries.append(("info", event, msg))

    def warning(self, msg, event=None, **kw):
        self.entries.append(("warn", event, msg))

    def error(self, msg, event=None, **kw):
        self.entries.append(("err", event, msg))


class _ConnEvt:
    def __init__(self, value):
        self._v = value

    def is_set(self):
        return self._v


def _make_node(connected=True):
    n = types.SimpleNamespace()
    n.ros_pub = _FakePub()
    n.ros_pick_pub = _FakePub()
    n.ros_plan_pub = _FakePub()
    n.ros_debug_pub = _FakePub()
    n.client = _FakeMqttClient()
    n.log = _FakeLog()
    n.topic_vr = "VR_control"
    n.topic_pick = "pick_robot"
    n.topic_plan = "plan_select"
    n._mqtt_connected = _ConnEvt(connected)
    n._debug_logs_enabled = False
    n._recent_local_pub = {}
    n._echo_suppress_window_sec = 0.35
    n._log_control = {
        "bridge_traffic_period": 1.0,
        "bridge_out_enabled": False,
        "bridge_in_enabled": False,
    }
    n._log_throttler = types.SimpleNamespace(
        should_emit_throttled=lambda *a, **kw: False,
        should_emit_on_change=lambda *a, **kw: False,
    )
    n._plan_resolver = PlanCommandResolver(
        plan_key_map={"1": "a19"},
        room_plan_map={"101": "a19"},
        known_plans={"a19"},
    )

    for name in (
        "_handle_inbound_event",
        "_resolve_plan_command",
        "_mark_local_publish",
        "_prune_local_pub",
        "_is_recent_local_echo",
        "_log_throttled",
    ):
        setattr(n, name, getattr(MQTTBridgeNode, name).__get__(n))
    return n


def test_keyboard_vr_publishes_to_both_ros_and_mqtt():
    n = _make_node()
    n._handle_inbound_event("__keyboard_vr__", "Forward,5")
    assert len(n.ros_pub.published) == 1
    assert n.ros_pub.published[0].data == "Forward,5"
    assert n.client.published == [("VR_control", "Forward,5")]
    assert ("VR_control", "Forward,5") in n._recent_local_pub


def test_keyboard_pick_publishes_to_both_ros_and_mqtt():
    n = _make_node()
    n._handle_inbound_event("__keyboard_pick__", "pick")
    assert len(n.ros_pick_pub.published) == 1
    assert n.client.published == [("pick_robot", "pick")]


def test_keyboard_debug_toggles_state():
    n = _make_node()
    assert n._debug_logs_enabled is False
    n._handle_inbound_event("__keyboard_debug__", "")
    assert n._debug_logs_enabled is True
    assert n.ros_debug_pub.published[0].data is True
    n._handle_inbound_event("__keyboard_debug__", "")
    assert n._debug_logs_enabled is False


def test_keyboard_plan_routes_when_connected():
    n = _make_node(connected=True)
    n._handle_inbound_event("__keyboard_plan__", "a19|1")
    assert n.ros_plan_pub.published[0].data == "a19"
    assert n.client.published == [("plan_select", "1")]


def test_keyboard_plan_falls_back_to_ros_only_when_disconnected():
    n = _make_node(connected=False)
    n._handle_inbound_event("__keyboard_plan__", "a19|1")
    assert n.ros_plan_pub.published[0].data == "a19"
    assert n.client.published == []
    assert any("BRIDGE_FALLBACK" == e[1] for e in n.log.entries)


def test_keyboard_plan_malformed_payload_logs_and_skips():
    n = _make_node()
    n._handle_inbound_event("__keyboard_plan__", "missing_pipe")
    assert n.ros_plan_pub.published == []
    assert n.client.published == []
    assert any("BRIDGE_BAD_PAYLOAD" == e[1] for e in n.log.entries)


def test_inbound_mqtt_vr_publishes_to_ros():
    n = _make_node()
    n._handle_inbound_event("VR_control", "Forward,5")
    assert n.ros_pub.published[0].data == "Forward,5"
    # Should NOT round-trip back to MQTT.
    assert n.client.published == []


def test_inbound_mqtt_plan_resolves_key_to_plan_name():
    n = _make_node()
    n._handle_inbound_event("plan_select", "1")
    assert n.ros_plan_pub.published[0].data == "a19"


def test_inbound_mqtt_plan_rejects_unknown_room():
    n = _make_node()
    n._handle_inbound_event("plan_select", "room:999")
    assert n.ros_plan_pub.published == []
    assert any("PLAN_MQTT" == e[1] for e in n.log.entries)


def test_inbound_echo_is_suppressed():
    n = _make_node()
    n._mark_local_publish("VR_control", "Forward,5")
    n._handle_inbound_event("VR_control", "Forward,5")
    # echo suppressed -> ROS publisher not invoked
    assert n.ros_pub.published == []
