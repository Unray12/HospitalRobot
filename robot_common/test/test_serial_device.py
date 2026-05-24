"""Tests for SerialDevice base class without requiring real hardware.

Patches `robot_common.serial_device.serial.Serial` so reconnect/open paths can be
exercised on Windows/CI without USB devices attached.
"""

from unittest.mock import MagicMock, patch

import pytest

from robot_common import serial_device


class _FakeSerial:
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.port = kwargs.get("port", args[0] if args else "")
        self.baudrate = kwargs.get("baudrate", 115200)
        self.kwargs = kwargs

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def close(self):
        self.is_open = False


@pytest.fixture
def patched_serial():
    fake_module = MagicMock()
    fake_module.Serial = _FakeSerial
    fake_module.SerialException = Exception
    with patch.object(serial_device, "serial", fake_module), \
         patch.object(serial_device, "list_ports", MagicMock(comports=lambda: [])):
        yield fake_module


def test_init_opens_port_on_construction(patched_serial):
    dev = serial_device.SerialDevice(port="/dev/ttyUSB0")
    assert dev.is_connected() is True
    assert dev.port == "/dev/ttyUSB0"


def test_close_clears_serial_handle(patched_serial):
    dev = serial_device.SerialDevice(port="/dev/ttyUSB0")
    dev.close()
    assert dev.ser is None
    assert dev.is_connected() is False


def test_reconnect_returns_true_when_already_connected(patched_serial):
    dev = serial_device.SerialDevice(port="/dev/ttyUSB0")
    assert dev.reconnect() is True


def test_reconnect_falls_back_to_alternate_port(patched_serial):
    def raise_on_first(*args, **kwargs):
        port = kwargs.get("port", args[0] if args else "")
        if port == "/dev/ttyUSB0":
            raise OSError("no such device")
        return _FakeSerial(*args, **kwargs)

    patched_serial.Serial = raise_on_first
    dev = serial_device.SerialDevice(port="/dev/ttyUSB0")
    assert dev.is_connected() is False

    assert dev.reconnect(fallback_ports=["/dev/ttyUSB1"]) is True
    assert dev.port == "/dev/ttyUSB1"


def test_reconnect_fails_when_no_candidates_open(patched_serial):
    def always_raise(*args, **kwargs):
        raise OSError("device missing")

    patched_serial.Serial = always_raise
    dev = serial_device.SerialDevice(port="/dev/ttyUSB0")
    assert dev.reconnect(fallback_ports=["/dev/ttyUSB1"], scan_prefixes=[]) is False


def test_reconnect_throttles_repeat_error_log(patched_serial):
    def always_raise(*args, **kwargs):
        raise OSError("device missing")

    patched_serial.Serial = always_raise

    logger = MagicMock()
    dev = serial_device.SerialDevice(port="/dev/ttyUSB0", logger=logger)
    # 1st construction logs error. Subsequent reconnects with the same error
    # should not double-log (same msg).
    err_count_first = logger.error.call_count
    dev.reconnect(fallback_ports=[], scan_prefixes=[])
    dev.reconnect(fallback_ports=[], scan_prefixes=[])
    err_count_after = logger.error.call_count
    assert err_count_after == err_count_first


def test_discover_ports_uses_glob(patched_serial, monkeypatch):
    monkeypatch.setattr(
        serial_device, "glob", MagicMock(glob=lambda p: ["/dev/ttyACM0", "/dev/ttyACM1"])
    )
    dev = serial_device.SerialDevice(port="/dev/ttyACM0")
    found = dev._discover_ports(["/dev/ttyACM"])
    assert "/dev/ttyACM0" in found
    assert "/dev/ttyACM1" in found


def test_matches_prefix_case_insensitive(patched_serial):
    dev = serial_device.SerialDevice(port="/dev/ttyUSB0")
    assert dev._matches_prefix("/dev/ttyUSB0", ["/dev/ttyUSB"]) is True
    assert dev._matches_prefix("COM3", ["com"]) is True
    assert dev._matches_prefix("/dev/random", ["/dev/ttyUSB"]) is False


def test_exclusive_falls_back_when_pyserial_lacks_kwarg(patched_serial):
    calls = []

    def selective_raise(*args, **kwargs):
        calls.append(dict(kwargs))
        if "exclusive" in kwargs:
            raise TypeError("exclusive kwarg not supported")
        return _FakeSerial(*args, **kwargs)

    patched_serial.Serial = selective_raise
    dev = serial_device.SerialDevice(port="/dev/ttyUSB0", exclusive=True)
    assert dev.is_connected() is True
    # First call with exclusive=True; second without.
    assert any("exclusive" in c for c in calls)
    assert any("exclusive" not in c for c in calls)
