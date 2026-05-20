"""Tracking strategy abstractions used by ``LineFollowerFSM``.

The strategies stay as thin adapters over FSM compute methods because the
underlying line-sensor and HuskyLens computations have side effects on FSM
state (``self.state`` transitions, plan-lost-line bookkeeping). Extracting
them fully would require also extracting the FSM lifecycle, which is out of
scope for this layer. The adapters exist so ``LineFollowerFSM`` can swap the
active strategy via configuration without branching everywhere.
"""


class BaseLineFollowStrategy:
    name = "base"

    def compute(self, frame_context):  # pragma: no cover - interface only
        raise NotImplementedError


class LineSensorStrategy(BaseLineFollowStrategy):
    name = "line_sensor"

    def __init__(self, compute_fn):
        self._compute_fn = compute_fn

    def compute(self, frame_context):
        return self._compute_fn(frame_context)


class HuskyLensStrategy(BaseLineFollowStrategy):
    name = "huskylens"

    def __init__(self, compute_fn):
        self._compute_fn = compute_fn

    def compute(self, frame_context):
        return self._compute_fn(frame_context)


class HybridStrategy(BaseLineFollowStrategy):
    """Prefer HuskyLens; fall back to line sensor when HuskyLens is unusable."""

    name = "hybrid"

    def __init__(self, huskylens_strategy, line_sensor_strategy):
        self._huskylens_strategy = huskylens_strategy
        self._line_sensor_strategy = line_sensor_strategy

    def compute(self, frame_context):
        command = self._huskylens_strategy.compute(frame_context)
        if command is not None:
            return command
        return self._line_sensor_strategy.compute(frame_context)
