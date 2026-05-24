import importlib.abc
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

# ROS2 source layout: <pkg>/<pkg>/<module>.py
# After colcon install: <pkg>/<module>.py
#
# For source-tree testing we map top-level <pkg> → inner <pkg>/<pkg> dir
# so that short-form imports (the installed form) resolve directly.
# This means tests must use the short form too.

_pkg_map = {}
for pkg_dir in ROOT.iterdir():
    inner = pkg_dir / pkg_dir.name
    if inner.is_dir() and (inner / "__init__.py").exists():
        _pkg_map[pkg_dir.name] = inner


class _ROS2PackageRedirect(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname in _pkg_map:
            inner = _pkg_map[fullname]
            spec = importlib.util.spec_from_file_location(
                fullname,
                inner / "__init__.py",
                submodule_search_locations=[str(inner)],
            )
            return spec
        return None


sys.meta_path.insert(0, _ROS2PackageRedirect())

# ---- Ignore ROS2-only linter tests (ament_*) ----
collect_ignore_glob = []
for p in ROOT.rglob("test/test_*.py"):
    text = p.read_text(encoding="utf-8", errors="replace")
    if "ament_" in text and "import" in text:
        collect_ignore_glob.append(str(p.relative_to(ROOT).as_posix()))
