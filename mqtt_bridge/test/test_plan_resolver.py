"""Pure-Python tests for the plan command resolver.

These tests do NOT import rclpy/paho and therefore run on Windows dev machines.
"""

from mqtt_bridge.plan_resolver import PlanCommandResolver, extract_face_ids


def _resolver():
    return PlanCommandResolver(
        plan_key_map={"1": "a19", "x": "plan_custom"},
        room_plan_map={"101": "a19", "102": "a18"},
        known_plans={"a19", "a18", "plan_custom"},
    )


def test_resolve_empty_returns_none():
    r = _resolver()
    assert r.resolve("") is None
    assert r.resolve("   ") is None
    assert r.resolve(None) is None


def test_resolve_clear_variants():
    r = _resolver()
    for token in ("0", "clear", "room:0", "room/0", "CLEAR", " Clear "):
        assert r.resolve(token) == "clear", token


def test_resolve_room_aliases():
    r = _resolver()
    assert r.resolve("room:101") == "a19"
    assert r.resolve("room/102") == "a18"
    assert r.resolve("phong:101") == "a19"
    assert r.resolve("101") == "a19"


def test_resolve_direct_plan_name_and_key():
    r = _resolver()
    assert r.resolve("a19") == "a19"
    assert r.resolve("1") == "a19"  # via plan_key_map
    assert r.resolve("x") == "plan_custom"


def test_resolve_plan_prefix_pass_through():
    r = _resolver()
    assert r.resolve("plan:plan_custom") == "plan_custom"
    assert r.resolve("plan/plan_custom") == "plan_custom"


def test_resolve_room_unmapped_returns_none():
    r = _resolver()
    assert r.resolve("room:999") is None


def test_resolve_unknown_passes_through():
    """Unknown plain text passes through; downstream decides."""
    r = _resolver()
    assert r.resolve("freestyle_plan") == "freestyle_plan"


def test_room_plan_map_falls_back_to_plan_keys_when_empty():
    r = PlanCommandResolver(plan_key_map={"1": "a19", "2": "a18"}, room_plan_map={})
    assert r.resolve("1") == "a19"
    assert r.resolve("room:2") == "a18"


def test_extract_face_ids_basic():
    assert extract_face_ids("<DEV1,FACE,1,2,3>") == [1, 2, 3]
    assert extract_face_ids("<DEV1,FACE,0>") == [0]
    assert extract_face_ids("<DEV1,NO_OBJECT>") == []
    assert extract_face_ids("") == []
    assert extract_face_ids(None) == []


def test_extract_face_ids_handles_separators():
    assert extract_face_ids("<DEV1,FACE,1;2|3>") == [1, 2, 3]
    assert extract_face_ids("<DEV1,FACE, 4 , 5 >") == [4, 5]
