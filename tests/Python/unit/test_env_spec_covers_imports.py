"""Every third-party import in the shipped Python must be declared in both
environment.yml (conda) and pyproject.toml (uv), and the two must agree."""
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
DIST = {"yaml": "pyyaml", "cv2": "opencv-python", "PIL": "pillow"}

#: Imports neither spec carries, and why.
EXEMPT = {
    "carla": "installed by carla_env_setup.ensure_carla()",
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


def _declared_pyproject():
    """Package names in pyproject.toml's [project] dependencies (the uv env).

    A regex rather than tomllib, which python 3.10 - the env's own version - lacks."""
    with open(os.path.join(ROOT, "pyproject.toml"), encoding="utf-8") as f:
        text = f.read()
    block = re.search(r"^dependencies\s*=\s*\[(.*?)^\]", text, re.M | re.S).group(1)
    lines = (ln.split("#")[0] for ln in block.splitlines())
    return {re.match(r"[A-Za-z0-9_.\-]+", s).group(0).lower()
            for ln in lines for s in re.findall(r'"([^"]+)"', ln)}


#: environment.yml entries that are conda's business, not packages uv installs.
CONDA_ONLY = {"python", "pip"}


def _local_names():
    """Top-level names that resolve inside this repo rather than site-packages.
    A repo module named like a real package hides that package from the scan."""
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


def test_every_shipped_import_is_declared_in_pyproject():
    missing = _undeclared(_declared_pyproject())
    assert not missing, "pyproject.toml does not declare: " + "; ".join(
        f"{imp} (imported by {', '.join(sorted(set(files))[:3])})"
        for imp, files in sorted(missing.items()))


def test_pyproject_and_environment_yml_declare_the_same_env():
    yml = _declared() - CONDA_ONLY
    uv = _declared_pyproject()
    assert yml == uv, (f"only in environment.yml: {sorted(yml - uv)}; "
                       f"only in pyproject.toml: {sorted(uv - yml)}")


def test_the_check_reports_a_package_that_is_absent():
    """Negative control: a checker that cannot fail looks exactly like success.

    Drop numpy from the declared set and the same code must name it, otherwise
    the test above passes for the wrong reason.
    """
    declared = _declared() - {"numpy"}
    missing = _undeclared(declared)
    assert "numpy" in missing, "the import scan found no numpy to report"
    assert missing["numpy"], "numpy reported with no file to look at"


# --------------------------------------------------------------------------
# the other half of "one manifest": the SUMO clients are pinned to the server
# --------------------------------------------------------------------------
def _setup():
    sys.path.insert(0, os.path.join(ROOT, "Carla"))
    try:
        import carla_env_setup
    finally:
        sys.path.pop(0)
    return carla_env_setup


def test_the_sumo_clients_are_pinned_to_the_manifest():
    """The clients are published in lockstep with the server, so an unpinned
    install takes whatever is newest -- measured at 1.27.1 against a build
    targeting 1.22.0, with nothing warning. FIXS#221."""
    env = _setup()
    want = env.sumo_version()
    assert want, "dependencies.yaml has a sumo version and it must be readable"
    assert env.sumo_client_pkgs() == ["eclipse-sumo==" + want,
                                      "traci==" + want,
                                      "sumolib==" + want]


def test_the_pin_follows_the_manifest_rather_than_a_constant(tmp_path, monkeypatch):
    """A hardcoded version is how requirements.txt drifted. Bump the manifest and
    the pin must move with it -- otherwise this test passes on a coincidence."""
    env = _setup()
    fake = tmp_path / "dependencies.yaml"
    fake.write_text(
        'tools:{nl}  sumo:{nl}    version: "9.99.9"{nl}'.format(nl=chr(10)),
        encoding="utf-8")
    monkeypatch.setattr(env, "DEPS_YAML", str(fake))
    assert env.sumo_client_pkgs() == ["eclipse-sumo==9.99.9", "traci==9.99.9",
                                      "sumolib==9.99.9"]


def test_an_unreadable_manifest_leaves_the_clients_unpinned(monkeypatch):
    """Unpinned beats refusing to build an env: the caller prints what it is
    about to install, so a missing pin is visible rather than silent."""
    env = _setup()
    monkeypatch.setattr(env, "DEPS_YAML", os.path.join(ROOT, "no-such-file.yaml"))
    assert env.sumo_client_pkgs() == ["eclipse-sumo", "traci", "sumolib"]
