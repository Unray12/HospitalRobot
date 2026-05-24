"""Log throttle / change-detection helpers shared by the MQTT bridge.

Kept as a pure module so it can be unit-tested without rclpy.
"""

import time


class LogThrottler:
    """Throttle log emissions to at most one per ``period`` seconds per key.

    Also supports change-detection ("emit only when value differs from last").
    The caller supplies the actual emit function; this class only decides
    whether to emit.
    """

    def __init__(self):
        self._last_ts = {}
        self._last_value = {}

    def should_emit_throttled(self, key, period, enabled, now=None):
        if not enabled:
            return False
        ts = now if now is not None else time.time()
        last = self._last_ts.get(key, 0.0)
        if (ts - last) < max(0.0, float(period)):
            return False
        self._last_ts[key] = ts
        return True

    def should_emit_on_change(self, key, value, enabled):
        if not enabled:
            return False
        previous = self._last_value.get(key)
        if previous == value:
            return False
        self._last_value[key] = value
        return True
