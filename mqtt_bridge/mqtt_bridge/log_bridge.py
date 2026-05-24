"""Bridge filtered ROS2 Log messages to MQTT."""

from rcl_interfaces.msg import Log


class LogBridge:
    """Subscribe to /rosout and forward matching log lines to MQTT."""

    def __init__(self, config, mqtt_publish_fn, log):
        log_bridge_cfg = config.get("log_bridge", {})
        self._enabled = bool(log_bridge_cfg.get("enabled", True))
        self._ros_topic = str(log_bridge_cfg.get("ros_topic", "/rosout"))
        self._sources = {
            str(name).strip().lower()
            for name in log_bridge_cfg.get("source_nodes", ["line_follower"])
            if str(name).strip()
        }
        self._keywords = [
            str(word)
            for word in log_bridge_cfg.get("keywords", ["[PLAN_STATUS]", "[PLAN]"])
            if str(word)
        ]
        self._publish = mqtt_publish_fn
        self._log = log
        self._sub = None

    @property
    def enabled(self):
        return self._enabled

    @property
    def ros_topic(self):
        return self._ros_topic

    def rosout_cb(self, msg: Log):
        if not self._enabled:
            return

        logger_name = str(msg.name or "").strip().lower()
        if self._sources and logger_name not in self._sources:
            return

        text = str(msg.msg or "")
        if self._keywords and not any(word in text for word in self._keywords):
            return

        self._publish(text)