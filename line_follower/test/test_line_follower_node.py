"""Unit tests for LineFollowerNode ROS callbacks.

These tests bind LineFollowerNode's unbound callback methods to a SimpleNamespace
so the FSM, queue/dict state, and parse logic can be exercised without booting
rclpy. They focus on callback inputs → state mutations and reject paths.
"""

import json
import sys
import time
import types

import pytest

pytest.importorskip("rclpy", reason="rclpy not installed (e.g. Windows dev)")

from line_follower.line_follower_node import LineFollowerNode


class _FakeMsg:
    def __init__(self, data):
        self.data = data


class _FakeLog:
    def __init__(self):
        self.entries = []

    def info(self, msg, event=None, **kw):
        self.entries.append(("info", event, msg))

    def warning(self, msg, event=None, **kw):
        self.entries.append(("warn", event, msg))

    def error(self, msg, event=None, **kw):
        self.entries.append(("err", event, msg))


def _bind(node, *names):
    for n in names:
        setattr(node, n, getattr(LineFollowerNode, n).__get__(node))


def _make_node():
    n = types.SimpleNamespace()
    n.log = _FakeLog()
    n._last_frame = None
    n._last_frame_ts = 0.0
    n._last_huskylens_frame = None
    n._last_huskylens_ts = 0.0
    n._auto_mode = False
    _bind(n, "_frame_cb", "_auto_cb", "_huskylens_cb")
    return n


def test_frame_cb_accepts_valid_frame():
    n = _make_node()
    msg = _FakeMsg([1, 2, 3, 1, 0, 1])
    n._frame_cb(msg)
    assert n._last_frame == {
        "left_count": 1, "mid_count": 2, "right_count": 3,
        "left_full": True, "mid_full": False, "right_full": True,
    }
    assert n._last_frame_ts > 0


def test_frame_cb_rejects_short_frame():
    n = _make_node()
    n._frame_cb(_FakeMsg([1, 2, 3]))
    assert n._last_frame is None
    assert any(e[1] == "FRAME" for e in n.log.entries)


def test_huskylens_cb_parses_json_payload():
    n = _make_node()
    payload = {
        "HuskylenSensor": {
            "connected": 1, "algorithm_set": 1, "valid": 1,
            "tail_offset_x": 4.5, "angle_deg": 12.0, "direction": -1,
            "y_type": 2, "line_length_y": 80, "y_head": 30, "y_tail": 110,
        }
    }
    n._huskylens_cb(_FakeMsg(json.dumps(payload)))
    assert n._last_huskylens_frame["valid"] == 1
    assert n._last_huskylens_frame["tail_offset_x"] == 4.5
    assert n._last_huskylens_frame["y_type"] == 2


def test_huskylens_cb_accepts_legacy_sensor_key():
    n = _make_node()
    payload = {"HuskyLensSensor": {"valid": 1}}
    n._huskylens_cb(_FakeMsg(json.dumps(payload)))
    assert n._last_huskylens_frame is not None
    assert n._last_huskylens_frame["valid"] == 1


def test_huskylens_cb_empty_clears_frame():
    n = _make_node()
    n._last_huskylens_frame = {"valid": 1}
    n._huskylens_cb(_FakeMsg(""))
    assert n._last_huskylens_frame is None


def test_huskylens_cb_malformed_json_clears_frame_silently():
    n = _make_node()
    n._last_huskylens_frame = {"valid": 1}
    n._huskylens_cb(_FakeMsg("not-json"))
    assert n._last_huskylens_frame is None


def test_huskylens_cb_missing_sensor_key_clears_frame():
    n = _make_node()
    n._last_huskylens_frame = {"valid": 1}
    n._huskylens_cb(_FakeMsg(json.dumps({"WrongKey": {}})))
    assert n._last_huskylens_frame is None


def test_auto_cb_delegates_to_set_auto_mode():
    n = _make_node()
    calls = []
    n._set_auto_mode = lambda v: calls.append(v)
    n._auto_cb(_FakeMsg(True))
    n._auto_cb(_FakeMsg(False))
    assert calls == [True, False]
