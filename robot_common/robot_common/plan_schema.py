"""Schema validation for plan YAML files.

Validates structure, label uniqueness, action whitelist, and GOTO target
resolution. Returns a list of error messages (empty list = valid plan).

Used by tests and can be invoked by hand:

    from robot_common.plan_schema import validate_plan
    errors = validate_plan(my_plan_dict)
"""

VALID_ACTIONS = frozenset({
    "Forward", "Backward", "Left", "Right",
    "RotateLeft", "RotateRight",
    "AutoLine", "AutoFollow",
    "Stop", "WAIT", "Goto",
})

VALID_END_STATES = frozenset({"stop", "follow"})


def validate_plan(plan):
    """Validate a parsed plan dict. Returns list[str] of error messages."""
    errors = []

    if not isinstance(plan, dict):
        return [f"plan must be a dict, got {type(plan).__name__}"]

    name = plan.get("name")
    if not isinstance(name, str) or not name:
        errors.append("missing or empty 'name' field")

    end_state = plan.get("end_state", "stop")
    if end_state not in VALID_END_STATES:
        errors.append(f"end_state must be one of {sorted(VALID_END_STATES)}, got {end_state!r}")

    steps = plan.get("steps")
    if not isinstance(steps, list):
        errors.append("'steps' must be a list")
        return errors
    if not steps:
        errors.append("'steps' is empty")
        return errors

    labels_seen = {}
    for i, step in enumerate(steps):
        if not isinstance(step, dict):
            errors.append(f"step[{i}] must be a dict, got {type(step).__name__}")
            continue

        action = step.get("action")
        if action not in VALID_ACTIONS:
            errors.append(f"step[{i}] action={action!r} not in {sorted(VALID_ACTIONS)}")

        label = step.get("label")
        if label is not None:
            if not isinstance(label, str):
                errors.append(f"step[{i}] label must be string, got {type(label).__name__}")
            elif label in labels_seen:
                errors.append(
                    f"step[{i}] duplicate label {label!r} (also at step[{labels_seen[label]}])"
                )
            else:
                labels_seen[label] = i

        for numeric_field in ("speed", "duration", "min_duration", "timeout"):
            if numeric_field in step:
                value = step[numeric_field]
                if not isinstance(value, (int, float)):
                    errors.append(
                        f"step[{i}] {numeric_field} must be number, got {type(value).__name__}"
                    )

        for bool_field in ("enabled", "strict_line", "continue_immediately", "end_state"):
            if bool_field in step:
                value = step[bool_field]
                if not isinstance(value, bool):
                    errors.append(
                        f"step[{i}] {bool_field} must be bool, got {type(value).__name__}={value!r}"
                    )

        if action == "Goto":
            target = step.get("target") or step.get("goto")
            if not isinstance(target, str):
                errors.append(f"step[{i}] Goto requires 'target' string field")
            elif target not in labels_seen and target not in (
                s.get("label") for s in steps[i + 1:] if isinstance(s, dict)
            ):
                errors.append(f"step[{i}] Goto target {target!r} not found in plan labels")

    return errors


__all__ = ["validate_plan", "VALID_ACTIONS", "VALID_END_STATES"]
