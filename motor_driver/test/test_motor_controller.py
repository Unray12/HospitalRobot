"""Unit tests for MotorController.move wire format and error handling."""

import threading

import pytest

from motor_driver.motor_controller import MotorController


class _FakeSerial:
    """Minimal pyserial Serial stand-in capturing writes."""

    def __init__(self, fail_on_write=False):
        self.is_open = True
        self.writes = []
        self._fail = fail_on_write

    def write(self, data):
        if self._fail:
            raise OSError("simulated write failure")
        self.writes.append(data)
        return len(data)

    def close(self):
        self.is_open = False

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass


def _controller_with_fake(fake):
    mc = MotorController.__new__(MotorController)
    mc._logger = None
    mc.ser = fake
    mc.port = "fake"
    mc.baudrate = 115200
    mc.timeout = 0.1
    mc._exclusive = False
    mc._last_open_error = None
    mc._lock = threading.Lock()
    return mc


def test_move_writes_frame_with_paren_after_newline_inside():
    fake = _FakeSerial()
    mc = _controller_with_fake(fake)

    result = mc.move("Forward", 8)

    assert result == [-8, -8, -8, -8]
    assert fake.writes == [b"(-8,-8,-8,-8)\n"]
    # Closing paren must come BEFORE newline so MCU parses one frame per line.
    written = fake.writes[0]
    assert written.endswith(b")\n")
    assert b"\n)" not in written


def test_move_speeds_are_integers_even_for_float_input():
    fake = _FakeSerial()
    mc = _controller_with_fake(fake)

    mc.move("Forward", 8.7)
    assert fake.writes[-1] == b"(-9,-9,-9,-9)\n"

    mc.move("Forward", 2.3)
    assert fake.writes[-1] == b"(-2,-2,-2,-2)\n"


def test_move_rotateleft_direction_vector():
    fake = _FakeSerial()
    mc = _controller_with_fake(fake)

    mc.move("RotateLeft", 5)
    assert fake.writes[-1] == b"(5,5,-5,-5)\n"


def test_move_stop_writes_zeros():
    fake = _FakeSerial()
    mc = _controller_with_fake(fake)

    mc.move("Stop", 0)
    assert fake.writes[-1] == b"(0,0,0,0)\n"


def test_move_invalid_direction_returns_none_and_writes_nothing():
    fake = _FakeSerial()
    mc = _controller_with_fake(fake)

    result = mc.move("Diagonal", 5)
    assert result is None
    assert fake.writes == []


def test_move_returns_none_on_write_failure_and_clears_serial():
    fake = _FakeSerial(fail_on_write=True)
    mc = _controller_with_fake(fake)

    result = mc.move("Forward", 8)

    assert result is None
    assert mc.ser is None


def test_move_returns_none_when_serial_unavailable():
    """When serial is not connected, move returns None immediately."""
    mc = _controller_with_fake(None)
    mc.ser = None

    result = mc.move("Backward", 4)
    assert result is None
