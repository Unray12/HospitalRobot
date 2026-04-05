class BaseLineFollowStrategy:
    name = "base"

    def compute(self, frame_context):
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


class HuskyLensRawStrategy(BaseLineFollowStrategy):
    name = "huskylens_raw"

    def __init__(self, compute_fn):
        self._compute_fn = compute_fn

    def compute(self, frame_context):
        return self._compute_fn(frame_context)


class HybridStrategy(BaseLineFollowStrategy):
    name = "hybrid"

    def __init__(self, huskylens_strategy, line_sensor_strategy):
        self._huskylens_strategy = huskylens_strategy
        self._line_sensor_strategy = line_sensor_strategy

    def compute(self, frame_context):
        command = self._huskylens_strategy.compute(frame_context)
        if command is not None:
            return command
        return self._line_sensor_strategy.compute(frame_context)
