"""Cross-platform single-key reader and keyboard input manager for MQTTBridge."""

import os
import queue
import sys
import threading
from threading import Event

if os.name == "nt":
    import msvcrt
else:
    import select
    import termios
    import tty


def get_key(timeout=None):
    """Read one keystroke. ``timeout`` (seconds) returns ``""`` on POSIX timeout.

    On Windows the timeout is implemented via a short poll loop on ``msvcrt.kbhit``.
    """
    if os.name == "nt":
        # Windows path: msvcrt.getch is blocking, so we implement timeout via
        # kbhit polling. 50 ms poll keeps CPU usage negligible while staying
        # responsive enough for keyboard teleop.
        if timeout is None:
            key = msvcrt.getch()
        else:
            import time as _time
            deadline = _time.monotonic() + timeout
            while _time.monotonic() < deadline:
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    break
                _time.sleep(0.05)
            else:
                return ""
        try:
            return key.decode("utf-8", errors="ignore").lower()
        except Exception:
            return ""

    # POSIX path: switch terminal to raw mode so each keypress arrives as a
    # single byte without waiting for Enter. tcsetattr restores cooked mode
    # in the finally block — must run even on exceptions, otherwise the user's
    # shell is left in an unusable raw state after the node exits.
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        if timeout is not None:
            # select() is the only way to read stdin with a timeout on POSIX.
            # Without this, sys.stdin.read(1) blocks forever and the keyboard
            # thread cannot honor _stop_event between keystrokes.
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            if not ready:
                return ""
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch.lower()


class KeyboardInput:
    """Read keystrokes in a background thread and emit typed events."""

    def __init__(self, config, plan_key_map, room_plan_map, inbound_queue, log, stop_event):
        self._inbound_queue = inbound_queue
        self._log = log
        self._stop_event = stop_event
        self._thread = None

        keyboard_cfg = config.get("keyboard", {})
        self.keyboard_map = {
            str(keyboard_cfg.get("forward", "w")).lower(): "Forward",
            str(keyboard_cfg.get("backward", "s")).lower(): "Backward",
            str(keyboard_cfg.get("left", "a")).lower(): "Left",
            str(keyboard_cfg.get("right", "d")).lower(): "Right",
            str(keyboard_cfg.get("stop", " ")).lower(): "Stop",
            str(keyboard_cfg.get("rotate_left", "j")).lower(): "RotateLeft",
            str(keyboard_cfg.get("rotate_right", "p")).lower(): "RotateRight",
        }
        self.key_toggle_auto = str(keyboard_cfg.get("toggle_auto", "k")).lower()
        self.key_toggle_debug_logs = str(keyboard_cfg.get("toggle_debug_logs", "e")).lower()
        self.key_quit = str(keyboard_cfg.get("quit", "q")).lower()
        self.plan_key_map = plan_key_map

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.5)

    def _loop(self):
        self._log.info("Dieu khien WASD | q de thoat", event="KEYBOARD")
        auto_mode = False
        while not self._stop_event.is_set():
            key = get_key(timeout=0.2)
            if not key:
                continue

            if key in self.keyboard_map:
                self._inbound_queue.put(("__keyboard_vr__", self.keyboard_map[key]))
            elif key == self.key_toggle_auto:
                auto_mode = not auto_mode
                self._inbound_queue.put(("__keyboard_pick__", "1" if auto_mode else "0"))
            elif key == self.key_toggle_debug_logs:
                self._inbound_queue.put(("__keyboard_debug__", "toggle"))
            elif key in self.plan_key_map:
                plan_name = self.plan_key_map[key]
                mqtt_plan_cmd = f"room:{key}" if key.isdigit() else plan_name
                self._inbound_queue.put(("__keyboard_plan__", f"{plan_name}|{mqtt_plan_cmd}"))
            elif key == self.key_quit:
                self._log.info("Thoat chuong trinh", event="KEYBOARD")
                self._stop_event.set()
                break