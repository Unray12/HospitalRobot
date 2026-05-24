from robot_common.logging_utils import LogAdapter


class DummyLogger:
    def __init__(self):
        self.info_messages = []
        self.warning_messages = []
        self.error_messages = []
        self.exception_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warning(self, message):
        self.warning_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)

    def exception(self, message):
        self.exception_messages.append(message)


def test_log_adapter_formats_fields_without_color():
    logger = DummyLogger()
    adapter = LogAdapter(logger, "line_follower", use_color=False)

    adapter.info("state updated", event="FSM", step=3, mode="auto")

    assert logger.info_messages == ["[line_follower] [FSM] state updated | step=3 mode=auto"]


def test_log_adapter_warn_alias_matches_warning():
    logger = DummyLogger()
    adapter = LogAdapter(logger, "mqtt_bridge", use_color=False)

    adapter.warn("fallback route", event="PLAN", room="101")

    assert logger.warning_messages == ["[mqtt_bridge] [PLAN] fallback route | room=101"]


def test_log_adapter_exception_uses_error_and_exception_channels():
    logger = DummyLogger()
    adapter = LogAdapter(logger, "motor_driver", use_color=False)

    adapter.exception("serial failure", event="SERIAL", port="COM4")

    assert logger.error_messages == ["[motor_driver] [SERIAL] serial failure | port=COM4"]
    assert logger.exception_messages == ["serial failure"]


def test_log_adapter_format_returns_message_without_emitting():
    logger = DummyLogger()
    adapter = LogAdapter(logger, "robot", use_color=False)

    message = adapter.format("INFO", "bringup ready", event="BOOT", nodes=6)

    assert message == "[robot] [BOOT] bringup ready | nodes=6"
    assert logger.info_messages == []
