"""Small structured logging helpers shared by ROS2 nodes in this repo."""

import os
import sys


_RESET = "\033[0m"
_LEVEL_COLORS = {
    "INFO": "\033[36m",
    "WARNING": "\033[33m",
    "ERROR": "\033[31m",
}


def _color_enabled(default=True):
    """Return whether ANSI colors should be emitted for structured log lines."""
    if os.getenv("NO_COLOR"):
        return False
    value = os.getenv("ROBOT_LOG_COLOR")
    if value is not None:
        return value.strip().lower() not in ("0", "false", "off", "no")
    if not default:
        return False
    try:
        return sys.stderr.isatty()
    except Exception:
        return False


class LogAdapter:
    """Format component-scoped log lines while preserving the rclpy logger API."""

    def __init__(self, logger, component, use_color=None):
        self._logger = logger
        self.component = component
        self.use_color = _color_enabled(default=True) if use_color is None else bool(use_color)

    def info(self, message, event="INFO", **fields):
        """Emit an informational structured log line."""
        self._logger.info(self._build("INFO", event, message, fields))

    def warning(self, message, event="WARN", **fields):
        """Emit a warning structured log line."""
        self._logger.warning(self._build("WARNING", event, message, fields))

    def error(self, message, event="ERR", **fields):
        """Emit an error structured log line."""
        self._logger.error(self._build("ERROR", event, message, fields))

    def _build(self, level, event, message, fields):
        """Build a single structured message with optional key-value fields."""
        text = f"[{self.component}] [{event}] {message}"
        if fields:
            kv = " ".join(f"{k}={v}" for k, v in fields.items())
            text = f"{text} | {kv}"
        if not self.use_color:
            return text
        color = _LEVEL_COLORS.get(level, "")
        if not color:
            return text
        return f"{color}{text}{_RESET}"

    def format(self, level, message, event=None, **fields):
        """Return a formatted message without emitting it.

        This is useful in tests and in code paths that need to pass a preformatted
        message into another logging surface.
        """
        event_name = event or level
        return self._build(level, event_name, message, fields)

    def exception(self, message, event="EXC", **fields):
        """Emit an exception-level message through the underlying error channel."""
        self._logger.error(self._build("ERROR", event, message, fields))
        if hasattr(self._logger, "exception"):
            self._logger.exception(message)

    warn = warning


def to_bool(raw, default=False):
    """Parse bool from YAML/config values (True/False/1/0/'true'/'yes'/...)."""
    if raw is None:
        return bool(default)
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, (int, float)):
        return bool(raw)
    text = str(raw).strip().lower()
    if text in {"1", "true", "yes", "on", "enable", "enabled"}:
        return True
    if text in {"0", "false", "no", "off", "disable", "disabled"}:
        return False
    return bool(default)


__all__ = ["LogAdapter", "to_bool"]
