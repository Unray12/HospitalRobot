"""Shared config and plan loading helpers for HospitalRobot packages."""

import copy
import json
from importlib import resources
from pathlib import Path


class ConfigManager:
    """Load package-scoped config and named plans from the shared package data.

    The manager caches raw JSON payloads to avoid repeated disk I/O, but it returns
    defensive copies so callers cannot accidentally mutate the global cache shared
    by other nodes in the same process.
    """

    _global_cache = {}
    _plan_cache = {}

    def __init__(self, package, filename="config.json", logger=None):
        self.package = package
        self.filename = filename
        self._logger = logger

    def load(self, defaults=None, force=False):
        """Load the package section from the shared JSON config file."""
        cache_key = ("robot_common", self.filename)
        if force or cache_key not in ConfigManager._global_cache:
            data = {}
            try:
                path = self._resource_path(self.filename)
                with path.open("r", encoding="utf-8") as file_obj:
                    data = json.load(file_obj)
            except Exception as exc:
                self._log_warn(f"robot_common/{self.filename} not loaded, using defaults: {exc}")

            ConfigManager._global_cache[cache_key] = data

        package_data = ConfigManager._global_cache.get(cache_key, {}).get(self.package, {})
        data = copy.deepcopy(package_data)
        if defaults:
            data = self._deep_merge(defaults, data)
        return data

    def _deep_merge(self, base, override):
        """Recursively merge override values into a copy of the base structure."""
        if not isinstance(base, dict) or not isinstance(override, dict):
            return copy.deepcopy(override)
        merged = copy.deepcopy(base)
        for key, value in override.items():
            if key in merged:
                merged[key] = self._deep_merge(merged[key], value)
            else:
                merged[key] = copy.deepcopy(value)
        return merged

    def load_plan(self, plan_name, force=False):
        """Load a named JSON plan from ``robot_common/plans``."""
        if not force and plan_name in ConfigManager._plan_cache:
            return copy.deepcopy(ConfigManager._plan_cache[plan_name])

        try:
            path = self._resource_path("plans", f"{plan_name}.json")
            with path.open("r", encoding="utf-8") as file_obj:
                data = json.load(file_obj)
            ConfigManager._plan_cache[plan_name] = data
            return copy.deepcopy(data)
        except Exception as exc:
            self._log_warn(f"plan {plan_name} not loaded: {exc}")
            return None

    def _resource_path(self, *parts):
        """Resolve package data both from an installed package and from the source tree."""
        try:
            path = resources.files("robot_common").joinpath(*parts)
            if path.exists():
                return path
        except Exception:
            pass
        return Path(__file__).resolve().parent.joinpath(*parts)

    @classmethod
    def clear_caches(cls):
        """Reset cached config and plan data for tests or force-refresh workflows."""
        cls._global_cache.clear()
        cls._plan_cache.clear()

    def _log_warn(self, msg):
        if self._logger:
            self._logger.warning(msg)
        else:
            print(msg)
