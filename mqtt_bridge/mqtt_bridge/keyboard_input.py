"""Cross-platform single-key reader and keyboard input manager for MQTTBridge."""

import os
import queue
import sys
import threading
from threading import Event

if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty


def get_key():
    if os.name == "nt":
        key = msvcrt.getch()
        try:
            return key.decode("utf-8", errors="ignore").lower()
        except Exception:
            return ""

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
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
        pass

    def _loop(self):
        self._log.info("Dieu khien WASD | q de thoat", event="KEYBOARD")
        auto_mode = False
        while not self._stop_event.is_set():
            key = get_key()
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