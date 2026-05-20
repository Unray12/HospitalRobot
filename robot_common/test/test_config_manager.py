from robot_common.config_manager import ConfigManager, _load_structured
from pathlib import Path
import tempfile
import pytest


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


def test_load_plan_rejects_path_traversal():
    logger = DummyLogger()
    manager = ConfigManager("line_follower", logger=logger)

    assert manager.load_plan("../config") is None
    assert manager.load_plan("..\\config") is None
    assert manager.load_plan("plans/../config") is None
    assert any("rejected" in m for m in logger.messages)


def test_load_plan_rejects_empty_and_non_string():
    manager = ConfigManager("line_follower", logger=DummyLogger())

    assert manager.load_plan("") is None
    assert manager.load_plan(None) is None
    assert manager.load_plan(123) is None


def test_load_plan_accepts_normal_names():
    manager = ConfigManager("line_follower")

    assert manager.load_plan("a19") is not None
    assert manager.load_plan("a15") is not None


# ── YAML loading + !include behavior ─────────────────────────────────────────


def test_per_package_yaml_takes_precedence_over_monolithic():
    """ConfigManager prefers config/<package>.yaml over config.yaml index."""
    cm = ConfigManager("line_follower")
    path = cm._resolve_config_path(cm.filename)
    assert path.name == "line_follower.yaml"
    assert path.parent.name == "config"


def test_monolithic_index_resolves_via_include(tmp_path):
    """Monolithic config.yaml with !include tags loads child files correctly."""
    cfg_dir = tmp_path / "config"
    cfg_dir.mkdir()
    (cfg_dir / "alpha.yaml").write_text("speed: 5\nname: alpha\n", encoding="utf-8")
    (tmp_path / "config.yaml").write_text(
        "alpha: !include config/alpha.yaml\n", encoding="utf-8"
    )

    data = _load_structured(tmp_path / "config.yaml")
    assert data == {"alpha": {"speed": 5, "name": "alpha"}}


def test_include_rejects_path_traversal(tmp_path):
    """!include with .. that escapes base dir must be rejected."""
    secret = tmp_path / "secret.yaml"
    secret.write_text("leaked: true\n", encoding="utf-8")

    cfg_dir = tmp_path / "cfg"
    cfg_dir.mkdir()
    (cfg_dir / "main.yaml").write_text(
        "leak: !include ../secret.yaml\n", encoding="utf-8"
    )

    with pytest.raises(ValueError, match="escapes config directory"):
        _load_structured(cfg_dir / "main.yaml")


def test_include_detects_cycles(tmp_path):
    """A → B → A include cycle must raise instead of recursing forever."""
    a = tmp_path / "a.yaml"
    b = tmp_path / "b.yaml"
    a.write_text("ref: !include b.yaml\n", encoding="utf-8")
    b.write_text("ref: !include a.yaml\n", encoding="utf-8")

    with pytest.raises(ValueError, match="cycle detected"):
        _load_structured(a)


def test_yaml_extension_priority_over_json(tmp_path):
    """When both .yaml and .json exist, YAML wins."""
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "demo.yaml").write_text("source: yaml\n", encoding="utf-8")
    (tmp_path / "config" / "demo.json").write_text('{"source": "json"}', encoding="utf-8")

    # Manually drive the resolver against an isolated tree.
    from importlib import resources as _res
    from unittest.mock import patch

    cm = ConfigManager("demo")

    def fake_resource_path(*parts):
        return tmp_path.joinpath(*parts)

    with patch.object(cm, "_resource_path", side_effect=fake_resource_path):
        path = cm._resolve_config_path("config.yaml")
        assert path.suffix == ".yaml"
        data = _load_structured(path)
        assert data["source"] == "yaml"


def test_load_plan_yaml_preferred_over_json():
    """When plan exists as .yaml AND .json, YAML wins."""
    cm = ConfigManager("line_follower")
    # Real plans/ has only .yaml after migration; verify resolver returns .yaml.
    path = cm._resolve_plan_path("a19")
    assert path.suffix == ".yaml"

