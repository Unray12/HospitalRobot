"""Tests for the LogThrottler helper."""

from mqtt_bridge.log_throttler import LogThrottler


def test_throttle_emits_first_and_skips_within_period():
    t = LogThrottler()
    assert t.should_emit_throttled("k", period=1.0, enabled=True, now=100.0) is True
    assert t.should_emit_throttled("k", period=1.0, enabled=True, now=100.5) is False
    assert t.should_emit_throttled("k", period=1.0, enabled=True, now=101.5) is True


def test_throttle_respects_enabled():
    t = LogThrottler()
    assert t.should_emit_throttled("k", period=1.0, enabled=False, now=100.0) is False


def test_throttle_keys_are_independent():
    t = LogThrottler()
    assert t.should_emit_throttled("a", period=1.0, enabled=True, now=100.0) is True
    assert t.should_emit_throttled("b", period=1.0, enabled=True, now=100.1) is True
    assert t.should_emit_throttled("a", period=1.0, enabled=True, now=100.2) is False


def test_change_detection_emits_only_when_value_changes():
    t = LogThrottler()
    assert t.should_emit_on_change("k", "1", enabled=True) is True
    assert t.should_emit_on_change("k", "1", enabled=True) is False
    assert t.should_emit_on_change("k", "0", enabled=True) is True
    assert t.should_emit_on_change("k", "0", enabled=True) is False


def test_change_detection_respects_enabled():
    t = LogThrottler()
    assert t.should_emit_on_change("k", "1", enabled=False) is False
