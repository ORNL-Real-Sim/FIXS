"""Every third-party import in the Python FIXS ships must be in environment.yml.

#378: Carla/carla_agents' global_route_planner imports networkx, environment.yml
did not list it, and an env built from the spec could not import BasicAgent or
BehaviorAgent at all -- mainVirCarla exited at startup and run_cosim stopped the
whole stack. The declaration existed, in requirements.txt (#305) -- a file that
no install path reads -- and nothing compared the two manifests.

This walks the shipped Python instead, so the check is against what the code
actually imports rather than against a second hand-written list.

    python -m pytest tests/Python/unit/test_env_spec_covers_imports.py
"""
from __future__ import annotations

import ast
import os
import re
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

#: What the release ships and runs from the env environment.yml builds.
SHIPPED = ("Carla", "CommonLib")

#: Vendored C/C++ trees and build output -- not Python FIXS runs.
SKIP_DIRS = {"__pycache__", ".git", "yaml-cpp", "libsumo", "libcarla",
             "ProprietaryFiles", "node_modules", ".venv"}

#: import name -> the distribution that provides it, where they differ.
DIST = {"yaml": "pyyaml"}

#: Imports that environment.yml deliberately does not carry, and why.
EXEMPT = {
    # environment.yml says so in its own trailing comment: a wheel path in the
    # spec breaks `conda env create` on every machine consuming a release, so
    # carla_env_setup.ensure_carla() installs these after the env exists.
    "carla": "installed by carla_env_setup.ensure_carla()",
    "traci": "installed by carla_env_setup.ensure_carla()",
    "sumolib": "installed by carla_env_setup.ensure_carla()",
    # Unreal Editor's embedded interpreter. These scripts are launched by
    # UE4Editor -ExecutePythonScript and never run under the realsim env.
    "unreal": "provided by the Unreal Editor, not pip-installable",
}


def _declared():
    """Package names under `dependencies:` in environment.yml."""
    out, inside = set(), False
    with open(os.path.join(ROOT, "environment.yml"), encoding="utf-8") as f:
        for line in f:
            if re.match(r"^\S", line):            # a new top-level key
                inside = line.startswith("dependencies:")
                continue
            if not inside:
                continue
            m = re.match(r"\s*-\s*([A-Za-z0-9_.\-]+)", line.split("#")[0])
            if m:
                out.add(m.group(1).split("=")[0].lower())
    return out


def _local_names():
    """Top-level names that resolve inside this repo rather than site-packages.

    Broad on purpose. Entry points put both the repo root and CommonLib/ on
    sys.path, so `import ConfigHelper` and `from Carla.carla_agents import ...`
    are both top-level, and a .py inside a package directory is reachable either
    way. The cost is that a repo module named like a real package hides it from
    this scan -- CommonLib/fixs/carla.py hides carla, Carla/sumo/ hides sumo.
    Both are in EXEMPT anyway, but a future collision would pass silently, so
    prefer a repo module name that no distribution uses.
    """
    out = set()
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        out.update(dirnames)
        for fn in filenames:
            if fn.endswith(".py"):
                out.add(fn[:-3])
    return out


def _shipped_files():
    for r in SHIPPED:
        base = os.path.join(ROOT, r)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
            for fn in filenames:
                if fn.endswith(".py"):
                    yield os.path.join(dirpath, fn)


def _third_party_imports():
    """{import name: [files]} for everything not stdlib and not in this repo."""
    stdlib, local = set(sys.stdlib_module_names), _local_names()
    hits: dict[str, list[str]] = {}
    for path in _shipped_files():
        with open(path, encoding="utf-8", errors="replace") as f:
            tree = ast.parse(f.read(), path)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and not node.level:
                names = [node.module or ""]
            else:
                continue                      # relative import -> always local
            for name in names:
                top = name.split(".")[0]
                if top and top not in stdlib and top not in local:
                    hits.setdefault(top, []).append(
                        os.path.relpath(path, ROOT))
    return hits


def _undeclared(declared):
    """Third-party imports that `declared` does not cover and EXEMPT excuses."""
    return {imp: files for imp, files in _third_party_imports().items()
            if imp not in EXEMPT
            and DIST.get(imp, imp).lower() not in declared}


def test_environment_yml_declares_networkx():
    """The #378 regression itself, stated as its own line."""
    assert "networkx" in _declared(), (
        "Carla/carla_agents imports networkx; without it in environment.yml a "
        "CARLA-agent controller cannot import and mainVirCarla exits at startup")


def test_every_shipped_import_is_declared():
    missing = _undeclared(_declared())
    assert not missing, "environment.yml does not declare: " + "; ".join(
        f"{imp} (imported by {', '.join(sorted(set(files))[:3])})"
        for imp, files in sorted(missing.items()))


def test_the_check_reports_a_package_that_is_absent():
    """Negative control: a checker that cannot fail looks exactly like success.

    Drop numpy from the declared set and the same code must name it, otherwise
    the test above passes for the wrong reason.
    """
    declared = _declared() - {"numpy"}
    missing = _undeclared(declared)
    assert "numpy" in missing, "the import scan found no numpy to report"
    assert missing["numpy"], "numpy reported with no file to look at"
