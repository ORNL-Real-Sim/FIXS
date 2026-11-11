"""fixs.batch (CommonLib/fixs/batch.py, #473): Runs are run_cosim's own options.

The runs themselves go to a stub engine, so nothing here launches a co-sim.

Run:  pytest tests/Python/unit/test_fixs_batch.py -v
"""
import json
import os
import sys
import textwrap

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from CommonLib.fixs import batch  # noqa: E402
from CommonLib.fixs.batch import BatchError, Run, grid  # noqa: E402


def test_keywords_are_run_cosim_flags():
    r = Run(app='mlk', config='a.yaml', slot=2, sumo_headless=True, no_launch=False,
            app_args='--egoDepart 29400')
    assert r.argv() == ['--app=mlk', '--config=a.yaml', '--slot=2', '--sumo-headless',
                        '--app-args=--egoDepart 29400']


def test_an_option_run_cosim_does_not_have_fails_when_the_run_is_made():
    with pytest.raises(BatchError, match='--sumo-headles'):
        Run(app='mlk', sumo_headles=True)


def test_a_bad_value_fails_when_the_run_is_made():
    with pytest.raises(BatchError):
        Run(engine='java')


def test_replace_changes_only_what_it_names():
    base = Run(name='b', app='mlk', config='a.yaml')
    r = base.replace(config='b.yaml', app_args='--x 1')
    assert r.options == {'app': 'mlk', 'config': 'b.yaml', 'app_args': '--x 1'}
    assert r.name == 'b' and base.options == {'app': 'mlk', 'config': 'a.yaml'}
    assert '--config' not in ' '.join(base.replace(config=None).argv())


def test_grid_is_every_combination():
    runs = grid(Run(app='mlk'), config=['a.yaml', 'b.yaml'], app_args=['--x 1', '--x 2', '--x 3'])
    assert len(runs) == 6
    assert runs[1].options == {'app': 'mlk', 'config': 'a.yaml', 'app_args': '--x 2'}


_STUB = textwrap.dedent('''
    import json, os, sys, time
    args = sys.argv[1:]
    info = args[args.index('--run-info') + 1]
    json.dump({'run': 'stub-%d' % os.getpid(), 'slot': 0}, open(info, 'w'))
    print('stub got', ' '.join(a for a in args if a.startswith('--app-args')))
    print('env', os.environ.get('COSIM_APP_ARGS'), os.environ.get('FIXS_BATCH_RUN'))
    if '--app=sleep' in args:
        time.sleep(30)
    sys.exit(3 if '--app=fail' in args else 0)
''')


@pytest.fixture
def stub(tmp_path, monkeypatch):
    p = tmp_path / 'run_cosim_stub.py'
    p.write_text(_STUB)
    monkeypatch.setattr(batch, 'RUN_COSIM', str(p))
    monkeypatch.setenv('COSIM_APP_ARGS', '--leaked')
    return tmp_path


def test_runs_go_one_after_another_and_land_in_the_manifest(stub):
    runs = [Run(name='one', app='ok', app_args='--egoDepart 1'), Run(app='fail')]
    out = stub / 'out'
    res = batch.run(runs, out_dir=str(out), cwd=str(stub), log=lambda m: None)
    assert [r['status'] for r in res] == ['ok', 'failed']
    assert res[1]['returncode'] == 3
    assert res[0]['run']['run'].startswith('stub-')
    log0 = open(res[0]['log'], encoding='utf-8').read()
    assert '--app-args=--egoDepart 1' in log0
    assert 'env None 1/2' in log0                   # COSIM_APP_ARGS does not leak; the run knows its place
    assert json.load(open(out / 'manifest.json'))[0]['name'] == 'one'


def test_stop_on_fail_stops(stub):
    res = batch.run([Run(app='fail'), Run(app='ok')], out_dir=str(stub / 'o'), cwd=str(stub),
                    stop_on_fail=True, log=lambda m: None)
    assert len(res) == 1


def test_a_run_past_its_timeout_is_stopped(stub):
    res = batch.run([Run(app='sleep', timeout_s=2)], out_dir=str(stub / 'o'), cwd=str(stub),
                    log=lambda m: None)
    assert res[0]['status'] == 'timeout' and res[0]['duration_s'] < 20


def test_a_yaml_is_a_base_and_runs_over_it(tmp_path):
    p = tmp_path / 'runs.yaml'
    p.write_text('base: {app: mlk, config: a.yaml}\n'
                 'runs:\n  - {name: e1, app_args: --egoDepart 1}\n  - {config: b.yaml}\n')
    runs = batch.load(str(p))
    assert [r.name for r in runs] == ['e1', None]
    assert runs[1].options == {'app': 'mlk', 'config': 'b.yaml'}
