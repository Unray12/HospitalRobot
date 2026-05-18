from line_follower.line_follow_strategies import (
    HybridStrategy,
    HuskyLensStrategy,
    LineSensorStrategy,
)


def test_line_sensor_strategy_calls_compute_fn():
    strategy = LineSensorStrategy(lambda ctx: ("Forward", 8) if ctx.get("ok") else None)
    assert strategy.compute({"ok": True}) == ("Forward", 8)
    assert strategy.compute({"ok": False}) is None


def test_huskylens_strategy_calls_compute_fn():
    strategy = HuskyLensStrategy(lambda ctx: ("RotateLeft", 6) if ctx.get("valid") else None)
    assert strategy.compute({"valid": True}) == ("RotateLeft", 6)
    assert strategy.compute({"valid": False}) is None


def test_hybrid_strategy_prefers_huskylens_then_fallback_line_sensor():
    huskylens = HuskyLensStrategy(lambda ctx: ("RotateRight", 6) if ctx.get("h") else None)
    line_sensor = LineSensorStrategy(lambda ctx: ("Forward", 8) if ctx.get("l") else None)
    hybrid = HybridStrategy(huskylens, line_sensor)
    assert hybrid.compute({"h": True, "l": True}) == ("RotateRight", 6)
    assert hybrid.compute({"h": False, "l": True}) == ("Forward", 8)
    assert hybrid.compute({"h": False, "l": False}) is None
