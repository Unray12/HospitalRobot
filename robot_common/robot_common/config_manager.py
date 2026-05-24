"""Shared config and plan loading helpers for HospitalRobot packages."""

import copy
import json
import re
from importlib import resources
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None


# Plan name comes from MQTT (untrusted source) → must be sanitized before being
# joined into a filesystem path. Allowing only [A-Za-z0-9_-] blocks "..",
# slashes, NULs, and any other path-traversal payload (e.g. "../config" would
# otherwise resolve to robot_common/config.yaml).
_PLAN_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def _load_structured(path, _seen=None):
    """Load a JSON or YAML file by extension. Returns parsed dict/list.

    Supports ``!include <relative_path>`` tags in YAML files so config.yaml
    can delegate each package section to its own file in config/.
    """
    resolved = path.resolve()
    if _seen is None:
        _seen = set()
    if resolved in _seen:
        raise ValueError(f"!include cycle detected: {resolved}")
    _seen.add(resolved)

    suffix = path.suffix.lower()
    with path.open("r", encoding="utf-8") as file_obj:
        if suffix in (".yaml", ".yml"):
            if yaml is None:
                raise ImportError("pyyaml not installed but YAML config requested")
            loader = _make_include_loader(path.parent, _seen)
            return yaml.load(file_obj, Loader=loader)
        return json.load(file_obj)


def _make_include_loader(base_dir, seen):
    """Return a yaml.Loader subclass that resolves !include relative to base_dir."""
    class IncludeLoader(yaml.SafeLoader):
        pass

    def _include(loader, node):
        rel = loader.construct_scalar(node)
        # Resolve and verify the included path stays inside base_dir to prevent
        # !include ../../etc/passwd style traversal in externally-editable configs.
        included = (base_dir / rel).resolve()
        if not str(included).startswith(str(base_dir.resolve())):
            raise ValueError(f"!include path escapes config directory: {rel!r}")
        return _load_structured(included, seen)

    IncludeLoader.add_constructor("!include", _include)
    return IncludeLoader


class ConfigManager:
    """Load package-scoped config and named plans from the shared package data.

    The manager caches raw config payloads to avoid repeated disk I/O, but it returns
    defensive copies so callers cannot accidentally mutate the global cache shared
    by other nodes in the same process.
    """

    _global_cache = {}
    _plan_cache = {}

    def __init__(self, package, filename="config.yaml", logger=None):
        self.package = package
        self.filename = filename
        self._logger = logger

    def load(self, defaults=None, force=False):
        """Load the package section from config.

        Tries per-package file (``config/<package>.yaml``) first — that file
        contains the section content directly (no top-level key). Falls back to
        the monolithic ``config.yaml`` where the content lives under a key equal
        to the package name.
        """
        cache_key = (self.package, self.filename)
        if force or cache_key not in ConfigManager._global_cache:
            data = {}
            try:
                path = self._resolve_config_path(self.filename)
                raw = _load_structured(path) or {}
                # Per-package file: top-level keys are config fields directly.
                # Monolithic file: top-level keys are package names.
                # Custom filename (shared mapping across packages, e.g.
                # plan_mapping.yaml) is also treated as per-package style.
                is_per_package = (
                    path.name == f"{self.package}.yaml"
                    or path.name == f"{self.package}.yml"
                    or (self.filename != "config.yaml" and path.name == self.filename)
                )
                if is_per_package:
                    data = raw if isinstance(raw, dict) else {}
                else:
                    data = raw.get(self.package, {}) if isinstance(raw, dict) else {}
            except Exception as exc:
                self._log_warn(f"config for '{self.package}' not loaded, using defaults: {exc}")

            ConfigManager._global_cache[cache_key] = data

        package_data = ConfigManager._global_cache.get(cache_key, {})
        if not isinstance(package_data, dict):
            self._log_warn(f"config for '{self.package}' is not a valid object, using defaults")
            package_data = {}
        # Deep-copy so callers can mutate the result without poisoning the
        # shared cache (e.g. node reads config, appends to a list field).
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
        """Load a named plan from ``robot_common/plans`` (YAML preferred, JSON fallback)."""
        if not isinstance(plan_name, str) or not _PLAN_NAME_RE.match(plan_name):
            self._log_warn(f"plan name rejected (invalid characters): {plan_name!r}")
            return None
        if not force and plan_name in ConfigManager._plan_cache:
            return copy.deepcopy(ConfigManager._plan_cache[plan_name])

        try:
            path = self._resolve_plan_path(plan_name)
            data = _load_structured(path)
            ConfigManager._plan_cache[plan_name] = data
            return copy.deepcopy(data)
        except Exception as exc:
            self._log_warn(f"plan {plan_name} not loaded: {exc}")
            return None

    def _resolve_config_path(self, filename):
        """Resolve config file: per-package YAML first, then monolithic, then JSON legacy."""
        # Priority:
        # 1. config/<package>.yaml  — per-package split file (new layout)
        # 2. config.yaml            — monolithic (migration fallback)
        # 3. config.json            — legacy JSON (back-compat)
        stem = filename.rpartition(".")[0] or filename
        is_custom = filename != "config.yaml"
        candidates = (
            [
                # Custom filename (e.g. "plan_mapping.yaml") must be resolved
                # before the per-package file so it is not shadowed by
                # "robot_common.yaml" when both exist under config/.
                ("config", filename),
                ("config", f"{self.package}.yaml"),
                ("config", f"{self.package}.yml"),
            ]
            if is_custom
            else [
                ("config", f"{self.package}.yaml"),
                ("config", f"{self.package}.yml"),
                ("config", filename),
                ("config", f"{stem}.yaml"),
                ("config", f"{stem}.yml"),
            ]
        ) + [
            (filename,),
            (stem + ".yaml",),
            (stem + ".yml",),
            (stem + ".json",),
        ]
        for parts in candidates:
            try:
                path = self._resource_path(*parts)
                if path.exists():
                    return path
            except Exception:
                continue
        raise FileNotFoundError(
            f"config for '{self.package}' not found (tried per-package + monolithic)"
        )

    def _resolve_plan_path(self, plan_name):
        """Resolve a plan file by name, trying YAML first then JSON."""
        for ext in (".yaml", ".yml", ".json"):
            try:
                path = self._resource_path("plans", f"{plan_name}{ext}")
                if path.exists():
                    return path
            except Exception:
                continue
        raise FileNotFoundError(
            f"plan '{plan_name}' not found (tried .yaml, .yml, .json)"
        )

    def _resource_path(self, *parts):
        """Resolve package data both from an installed package and from the source tree."""
        # In colcon-installed layout, importlib.resources finds the package data
        # under site-packages. In source-tree (dev/pytest), files() may return a
        # path that does not exist; we fall back to walking from this module's
        # __file__ to find the package data dir.
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
