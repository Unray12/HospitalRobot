"""Tests for plan_schema.validate_plan and structural checks on shipped plans."""

from pathlib import Path

import pytest
import yaml

from robot_common.plan_schema import validate_plan


PLANS_DIR = Path(__file__).resolve().parent.parent / "robot_common" / "plans"


@pytest.mark.parametrize("plan_file", sorted(PLANS_DIR.glob("*.yaml")))
def test_shipped_plan_is_valid(plan_file):
    plan = yaml.safe_load(plan_file.read_text(encoding="utf-8"))
    errors = validate_plan(plan)
    assert not errors, f"{plan_file.name}: {errors}"


def test_validate_plan_rejects_non_dict():
    assert validate_plan("not a dict")
    assert validate_plan(None)


def test_validate_plan_requires_name():
    plan = {"steps": [{"action": "Forward"}]}
    errors = validate_plan(plan)
    assert any("name" in e for e in errors)


def test_validate_plan_detects_invalid_action():
    plan = {
        "name": "test",
        "steps": [{"action": "MysteryAction"}],
    }
    errors = validate_plan(plan)
    assert any("MysteryAction" in e for e in errors)


def test_validate_plan_detects_duplicate_labels():
    plan = {
        "name": "test",
        "steps": [
            {"label": "L1", "action": "Forward"},
            {"label": "L1", "action": "Backward"},
        ],
    }
    errors = validate_plan(plan)
    assert any("duplicate" in e for e in errors)


def test_validate_plan_detects_bad_end_state_type():
    plan = {
        "name": "test",
        "steps": [{"action": "Forward", "end_state": "true"}],
    }
    errors = validate_plan(plan)
    assert any("end_state must be bool" in e for e in errors)


def test_validate_plan_detects_bad_numeric_type():
    plan = {
        "name": "test",
        "steps": [{"action": "Forward", "speed": "fast"}],
    }
    errors = validate_plan(plan)
    assert any("speed must be number" in e for e in errors)


def test_validate_plan_detects_bad_top_level_end_state():
    plan = {
        "name": "test",
        "end_state": "running",
        "steps": [{"action": "Forward"}],
    }
    errors = validate_plan(plan)
    assert any("end_state must be one of" in e for e in errors)


def test_validate_plan_detects_unresolved_goto_target():
    plan = {
        "name": "test",
        "steps": [{"action": "Goto", "target": "nonexistent"}],
    }
    errors = validate_plan(plan)
    assert any("Goto target" in e for e in errors)


def test_validate_plan_accepts_resolved_goto():
    plan = {
        "name": "test",
        "steps": [
            {"label": "L1", "action": "Forward"},
            {"action": "Goto", "target": "L1"},
        ],
    }
    errors = validate_plan(plan)
    assert errors == []
