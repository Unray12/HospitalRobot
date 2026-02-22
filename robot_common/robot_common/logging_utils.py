import os


_RESET = "\033[0m"
_LEVEL_COLORS = {
    "INFO": "\033[36m",
    "WARNING": "\033[33m",
    "ERROR": "\033[31m",
}


def _color_enabled(default=True):
    if os.getenv("NO_COLOR"):
        return False
    value = os.getenv("ROBOT_LOG_COLOR")
    if value is None:
        return default
    return value.strip().lower() not in ("0", "false", "off", "no")


class LogAdapter:
    """
    Structured logger wrapper.
    Keeps compatibility with rclpy logger API (info/warning/error).
    """

    def __init__(self, logger, component, use_color=None):
        self._logger = logger
        self.component = component
        self.use_color = _color_enabled(default=True) if use_color is None else bool(use_color)

    def info(self, message, event="INFO", **fields):
        self._logger.info(self._build("INFO", event, message, fields))

    def warning(self, message, event="WARN", **fields):
        self._logger.warning(self._build("WARNING", event, message, fields))

    def error(self, message, event="ERR", **fields):
        self._logger.error(self._build("ERROR", event, message, fields))

    def _build(self, level, event, message, fields):
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
