import importlib.util
import sys
import pathlib
# Add repo root to path for all tests
sys.path.insert(0, str(pathlib.Path(__file__).parents[2]))

# tests/Python/pytest.ini makes this directory the rootdir, and pytest loads no
# conftest above it - so tests/conftest.py's no-windows guard must be picked up
# here too. By path, not `from tests.conftest import`: any installed package
# named `tests` would win that import over this repo's directory.
_spec = importlib.util.spec_from_file_location(
    "_fixs_tests_conftest", pathlib.Path(__file__).parents[1] / "conftest.py")
_shared = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_shared)
_no_windows = _shared._no_windows
