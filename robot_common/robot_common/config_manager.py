import json
from importlib import resources


class ConfigManager:
    _global_cache = None
    _plan_cache = {}

    def __init__(self, package, filename="config.json", logger=None):
        self.package = package
        self.filename = filename
        self._logger = logger

    def load(self, defaults=None, force=False):
        if ConfigManager._global_cache is None or force:
            data = {}
            try:
                path = resources.files("robot_common").joinpath(self.filename)
                with path.open("r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception as exc:
                self._log_warn(f"robot_common/{self.filename} not loaded, using defaults: {exc}")

            ConfigManager._global_cache = data

        data = ConfigManager._global_cache.get(self.package, {})
        if defaults:
            data = self._deep_merge(defaults, data)
        return data

    def _deep_merge(self, base, override):
        if not isinstance(base, dict) or not isinstance(override, dict):
            return override
        merged = dict(base)
        for key, value in override.items():
            if key in merged:
                merged[key] = self._deep_merge(merged[key], value)
            else:
                merged[key] = value
        return merged

    def load_plan(self, plan_name):
        if plan_name in ConfigManager._plan_cache:
            return ConfigManager._plan_cache[plan_name]

        try:
            path = resources.files("robot_common").joinpath("plans", f"{plan_name}.json")
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            ConfigManager._plan_cache[plan_name] = data
            return data
        except Exception as exc:
            self._log_warn(f"plan {plan_name} not loaded: {exc}")
            return None

    def _log_warn(self, msg):
        if self._logger:
            self._logger.warn(msg)
        else:
            print(msg)
