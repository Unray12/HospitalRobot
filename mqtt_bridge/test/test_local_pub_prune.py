"""Tests for MQTTBridgeNode._recent_local_pub pruning behavior.

These tests exercise the prune logic without bringing up rclpy/paho. They only
need the unbound methods, so we construct a minimal stand-in. The test module
is skipped on environments without rclpy installed (e.g. Windows dev).
"""

import time
import types

import pytest

pytest.importorskip("rclpy", reason="rclpy not installed (e.g. Windows dev)")
pytest.importorskip("paho.mqtt.client", reason="paho-mqtt not installed")


def _bridge_with_prune_methods(suppress_window=0.35):
    """Build a stand-in carrying the methods under test."""
    from mqtt_bridge.MQTTBridgeROS import MQTTBridgeNode

    obj = types.SimpleNamespace()
    obj._recent_local_pub = {}
    obj._echo_suppress_window_sec = suppress_window
    obj._mark_local_publish = MQTTBridgeNode._mark_local_publish.__get__(obj)
    obj._prune_local_pub = MQTTBridgeNode._prune_local_pub.__get__(obj)
    obj._is_recent_local_echo = MQTTBridgeNode._is_recent_local_echo.__get__(obj)
    return obj


def test_mark_then_check_echo_is_recent():
    bridge = _bridge_with_prune_methods()
    bridge._mark_local_publish("topic", "payload")
    assert bridge._is_recent_local_echo("topic", "payload") is True


def test_unrelated_topic_is_not_echo():
    bridge = _bridge_with_prune_methods()
    bridge._mark_local_publish("topic", "payload")
    assert bridge._is_recent_local_echo("other", "payload") is False
    assert bridge._is_recent_local_echo("topic", "different") is False


def test_prune_removes_entries_older_than_window():
    bridge = _bridge_with_prune_methods(suppress_window=0.1)
    bridge._recent_local_pub[("a", "1")] = time.time() - 1.0
    bridge._recent_local_pub[("b", "2")] = time.time()
    bridge._prune_local_pub()
    assert ("a", "1") not in bridge._recent_local_pub
    assert ("b", "2") in bridge._recent_local_pub


def test_mark_triggers_prune_when_over_threshold():
    bridge = _bridge_with_prune_methods(suppress_window=0.1)
    stale_ts = time.time() - 10.0
    for i in range(100):
        bridge._recent_local_pub[(f"stale_topic_{i}", str(i))] = stale_ts
    bridge._mark_local_publish("fresh", "payload")
    assert ("fresh", "payload") in bridge._recent_local_pub
    assert len(bridge._recent_local_pub) == 1


def test_is_recent_local_echo_deletes_expired_entry():
    bridge = _bridge_with_prune_methods(suppress_window=0.05)
    bridge._recent_local_pub[("topic", "payload")] = time.time() - 1.0
    assert bridge._is_recent_local_echo("topic", "payload") is False
    assert ("topic", "payload") not in bridge._recent_local_pub
