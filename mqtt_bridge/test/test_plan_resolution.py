import pytest

pytest.importorskip("rclpy")

from mqtt_bridge.MQTTBridgeROS import MQTTBridgeNode
from mqtt_bridge.plan_resolver import PlanCommandResolver


class _NodeStub:
    def __init__(self):
        self.plan_key_map = {"1": "a19", "x": "plan_custom"}
        self.room_plan_map = {"101": "a19", "102": "a18"}
        self._known_plans = {"a19", "a18", "plan_custom"}
        self._plan_resolver = PlanCommandResolver(
            plan_key_map=self.plan_key_map,
            room_plan_map=self.room_plan_map,
            known_plans=self._known_plans,
        )
        self.log_messages = []
        self.log = type("Log", (), {"warning": lambda self_, msg, event=None: None})()

    _resolve_plan_command = MQTTBridgeNode._resolve_plan_command


def test_resolve_plan_command_supports_room_aliases_and_clear():
    node = _NodeStub()

    assert node._resolve_plan_command("room:101") == "a19"
    assert node._resolve_plan_command("102") == "a18"
    assert node._resolve_plan_command("clear") == "clear"


def test_resolve_plan_command_supports_direct_plan_names():
    node = _NodeStub()

    assert node._resolve_plan_command("a19") == "a19"
    assert node._resolve_plan_command("plan:plan_custom") == "plan_custom"
    assert node._resolve_plan_command("x") == "plan_custom"
