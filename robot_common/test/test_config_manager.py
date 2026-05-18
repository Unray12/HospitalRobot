from robot_common.config_manager import ConfigManager


class DummyLogger:
    def __init__(self):
        self.messages = []

    def warning(self, message):
        self.messages.append(message)


def setup_function():
    ConfigManager.clear_caches()


def test_load_returns_package_section():
    config = ConfigManager("line_follower").load()

    assert config["tracking"]["strategy"] == "huskylens"
    assert config["topics"]["cmd_vel"] == "/motor_cmd"


def test_load_merges_defaults_without_mutating_cache():
    manager = ConfigManager("robot")
    defaults = {
        "bringup": {
            "nodes": ["fallback_node"],
            "timeout": 5,
        }
    }

    config = manager.load(defaults=defaults)
    config["bringup"]["nodes"].append("mutated")

    reloaded = manager.load(defaults=defaults)

    assert reloaded["bringup"]["timeout"] == 5
    assert "mutated" not in reloaded["bringup"]["nodes"]


def test_load_plan_returns_defensive_copy():
    manager = ConfigManager("line_follower")

    plan = manager.load_plan("a19")
    plan["steps"].append({"action": "mutated"})

    reloaded = manager.load_plan("a19")

    assert reloaded["steps"][-1]["action"] != "mutated"


def test_missing_plan_logs_warning_and_returns_none():
    logger = DummyLogger()
    manager = ConfigManager("line_follower", logger=logger)

    plan = manager.load_plan("missing_plan")

    assert plan is None
    assert logger.messages
    assert "missing_plan" in logger.messages[-1]
