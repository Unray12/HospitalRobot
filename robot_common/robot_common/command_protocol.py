"""Shared parser/formatter for the repo-wide ``Direction:Speed`` motor contract."""

import math


VALID_DIRECTIONS = (
    "Forward",
    "Backward",
    "Left",
    "Right",
    "RotateLeft",
    "RotateRight",
    "Stop",
)


def _coerce_speed(value):
    """Convert a speed-like value into a non-negative integer."""
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return 0
    if not math.isfinite(numeric):
        return 0
    return max(0, int(round(numeric)))


def parse_command(text):
    """Parse a command string into ``(direction, speed)``.

    The accepted wire format is ``Direction:Speed``. For backward compatibility we
    also accept comma- and space-delimited variants that already exist in tests and
    older tooling.
    """
    raw = str(text or "").strip()
    if not raw:
        return None, None

    direction = raw
    speed = 0
    for sep in (":", ",", " "):
        if sep in raw:
            parts = [part.strip() for part in raw.split(sep) if part.strip()]
            if parts:
                direction = parts[0]
            if len(parts) > 1:
                speed = _coerce_speed(parts[1])
            break

    if direction not in VALID_DIRECTIONS:
        return None, None
    if direction == "Stop":
        return "Stop", 0
    return direction, _coerce_speed(speed)


def format_command(direction, speed=0):
    """Format a command string using the shared motor command contract."""
    if direction not in VALID_DIRECTIONS:
        return None
    if direction == "Stop":
        return "Stop:0"
    return f"{direction}:{_coerce_speed(speed)}"


__all__ = ["VALID_DIRECTIONS", "parse_command", "format_command"]
