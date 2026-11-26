"""run_cosim options fixs.batch relies on (#473): build_parser, --app-args, --run-info,
and --config naming an app yaml the way apps.json does.

Run:  pytest tests/Python/unit/test_run_cosim_batch_options.py -v
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import app_catalog  # noqa: E402
import run_cosim  # noqa: E402


def test_the_parser_is_buildable_without_running_anything():
    a = run_cosim.build_parser().parse_args(
        ["--app", "mlk", "--app-args=--egoDepart 29400", "--run-info", "x.json"])
    assert a.app == "mlk" and a.app_args == "--egoDepart 29400" and a.run_info == "x.json"


def _ctx(app, staged_dir):
    src = os.path.join(app_catalog.app_dir(app), "Scen", "config_a.yaml")
    return {"app": app, "staged": [{"path": os.path.join(staged_dir, "config_a.yaml"),
                                    "title": "", "engine": "py", "source": src}]}, src


def test_config_as_apps_json_names_it_means_the_staged_copy(tmp_path):
    ctx, _src = _ctx({"id": "demo"}, str(tmp_path))
    rec = {"config": "Scen/config_a.yaml", "config_scope": "map"}
    run_cosim._scope_config(rec, ctx)
    assert rec == {"config": os.path.join(str(tmp_path), "config_a.yaml"), "config_scope": "app"}


def test_config_as_the_repo_file_means_the_staged_copy(tmp_path):
    ctx, src = _ctx({"id": "demo"}, str(tmp_path))
    rec = {"config": src, "config_scope": "map"}
    run_cosim._scope_config(rec, ctx)
    assert rec["config_scope"] == "app" and rec["config"].endswith("config_a.yaml")
    assert os.path.dirname(rec["config"]) == str(tmp_path)


def test_any_other_config_stays_a_map_yaml(tmp_path):
    ctx, _src = _ctx({"id": "demo"}, str(tmp_path))
    rec = {"config": "Scen/other.yaml", "config_scope": "map"}
    run_cosim._scope_config(rec, ctx)
    assert rec == {"config": "Scen/other.yaml", "config_scope": "map"}
