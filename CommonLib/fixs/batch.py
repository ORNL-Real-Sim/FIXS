"""fixs.batch -- co-sim runs one after another, unattended (#473).

    from fixs import batch

    base = batch.Run(app='mlk_eco_driving', map='mlk_uturn',
                     config='MLK_Sumo_Scenario/config_Sumo_Carla_ecoDriving_L2_dynosim_udp.yaml')
    runs = [base.replace(app_args='--egoDepart 29400 --runTag e29400'),
            base.replace(app_args='--egoDepart 28800 --runTag e28800')]
    results = batch.run(runs)

A Run's keywords are run_cosim's own options, dashes as underscores: True is a bare
flag, False or None leaves it out, anything else is its value. They are checked
against run_cosim's parser when the Run is made, so a typo fails before the first
run. Only ``name`` and ``timeout_s`` are the batch's own.

``batch.grid(base, app_args=[...], config=[...])`` makes a Run for every combination.

From a shell, with the same keywords in yaml::

    python FIXS/CommonLib/fixs/batch.py runs.yaml

    base: {app: mlk_eco_driving, map: mlk_uturn, config: MLK_Sumo_Scenario/....yaml}
    runs:
      - {name: e29400, app_args: --egoDepart 29400}
      - {name: e28800, app_args: --egoDepart 28800}
"""
import argparse
import datetime
import itertools
import json
import os
import platform
import subprocess
import sys
import threading
import time

FIXS_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUN_COSIM = os.path.join(FIXS_ROOT, 'Carla', 'run_cosim.py')
#: Each run gets only what its Run says; these would otherwise leak in from the caller.
_RUN_ENV = ('COSIM_APP_ARGS', 'COSIM_CARLA_ARGS', 'COSIM_SUMO_ARGS')

__all__ = ['Run', 'grid', 'run', 'BatchError']


class BatchError(ValueError):
    """A Run that run_cosim would refuse."""


_PARSER = None


def _parser():
    """run_cosim's own parser, raising BatchError instead of exiting."""
    global _PARSER
    if _PARSER is None:
        carla = os.path.dirname(RUN_COSIM)
        if carla not in sys.path:
            sys.path.insert(0, carla)
        import run_cosim
        p = run_cosim.build_parser()

        def error(message):
            raise BatchError(message)
        p.error = error
        _PARSER = p
    return _PARSER


def _flag(key):
    return '--' + key.replace('_', '-')


class Run(object):
    """One run_cosim call. Keywords are its options; see the module doc."""

    def __init__(self, name=None, timeout_s=None, **options):
        self.name = name
        self.timeout_s = timeout_s
        self.options = dict(options)
        self._argv = self._build()

    def _build(self):
        p = _parser()
        known = {s for a in p._actions for s in a.option_strings}
        argv = []
        for key, value in self.options.items():
            flag = _flag(key)
            if flag not in known:
                raise BatchError('run_cosim has no %s (from %s=)' % (flag, key))
            if value is None or value is False:
                continue
            if value is True:
                argv.append(flag)
            elif isinstance(value, (list, tuple)):
                argv += [flag] + [str(v) for v in value]
            else:
                argv.append('%s=%s' % (flag, value))     # a value may itself start with '-'
        # argparse.ArgumentParser.parse_args, not run_cosim's: that one asks at a terminal.
        argparse.ArgumentParser.parse_args(p, argv)
        return argv

    def argv(self):
        """The run_cosim options this Run stands for."""
        return list(self._argv)

    def replace(self, **changes):
        """A copy with these keywords changed (a value of None removes an option)."""
        kw = dict(self.options, name=self.name, timeout_s=self.timeout_s)
        kw.update(changes)
        return Run(**kw)

    def __repr__(self):
        own = [('name', self.name)] if self.name else []
        return 'Run(%s)' % ', '.join('%s=%r' % kv for kv in own + list(self.options.items()))


def grid(base, **values):
    """A Run for every combination of the listed values, each a base.replace()."""
    keys = list(values)
    return [base.replace(**dict(zip(keys, combo)))
            for combo in itertools.product(*(values[k] for k in keys))]


def _app_root():
    """The application repo a bundle sits in (<repo>/FIXS), else the current folder."""
    parent = os.path.dirname(FIXS_ROOT)
    if os.path.isfile(os.path.join(parent, 'apps', 'apps.json')):
        return parent
    return os.getcwd()


def _kill_tree(pid):
    if platform.system() == 'Windows':
        subprocess.call(['taskkill', '/F', '/T', '/PID', str(pid)],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        try:
            os.killpg(os.getpgid(pid), 9)
        except OSError:
            pass


def _pump(stream, sinks):
    for line in iter(stream.readline, ''):
        for s in sinks:
            s.write(line)
            s.flush()


def _readJson(path):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def run(runs, *, timeout_s=None, out_dir=None, stop_on_fail=False, cwd=None, env=None,
        python=None, log=print):
    """Run each Run in turn; return one result dict per run started.

    Each run's console is echoed here and kept as run_NN.log in `out_dir`, beside
    manifest.json (rewritten after every run). `timeout_s` applies to a Run without
    its own; a run past it is stopped with run_cosim --stop and then killed. Each run
    sees FIXS_BATCH_RUN ('2/3') and FIXS_BATCH_NAME in its environment.
    """
    runs = list(runs)
    cwd = cwd or _app_root()
    out_dir = out_dir or os.path.join(cwd, 'RealSim_tmp', 'batch',
                                      datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)
    base_env = dict(os.environ, **(env or {}))
    for k in _RUN_ENV:
        base_env.pop(k, None)
    base_env['PYTHONUNBUFFERED'] = '1'
    python = python or sys.executable
    manifest = os.path.join(out_dir, 'manifest.json')
    results = []
    log('[batch] %d runs, output in %s' % (len(runs), out_dir))
    for i, r in enumerate(runs, 1):
        tag = 'run_%02d' % i
        info = os.path.join(out_dir, tag + '.run.json')
        logPath = os.path.join(out_dir, tag + '.log')
        cmd = [python, RUN_COSIM] + r.argv() + ['--run-info', info]
        limit = r.timeout_s if r.timeout_s is not None else timeout_s
        log('[batch] %d/%d %s: %s' % (i, len(runs), r.name or tag,
                                     subprocess.list2cmdline(r.argv())))
        started = time.time()
        kw = {}
        if platform.system() != 'Windows':
            kw['start_new_session'] = True
        # Which batch run this is, for an app or recorder to label it.
        runEnv = dict(base_env, FIXS_BATCH_RUN='%d/%d' % (i, len(runs)),
                      FIXS_BATCH_NAME=r.name or tag)
        with open(logPath, 'w', encoding='utf-8', errors='replace') as logFile:
            # A closed pipe for stdin: not a terminal, so nothing can stop at a prompt.
            proc = subprocess.Popen(cmd, cwd=cwd, env=runEnv, stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, encoding='utf-8', errors='replace', **kw)
            proc.stdin.close()
            pump = threading.Thread(target=_pump, args=(proc.stdout, (sys.stdout, logFile)),
                                    daemon=True)
            pump.start()
            status = None
            try:
                rc = proc.wait(timeout=limit)
            except subprocess.TimeoutExpired:
                status = 'timeout'
                log('[batch] %s ran past %s s; stopping it' % (r.name or tag, limit))
                record = _readJson(info) or {}
                if record.get('run'):
                    subprocess.call([python, RUN_COSIM, '--stop', record['run']], cwd=cwd,
                                    env=base_env, stdin=subprocess.DEVNULL,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                _kill_tree(proc.pid)
                rc = proc.wait()
            except KeyboardInterrupt:
                _kill_tree(proc.pid)
                proc.wait()
                raise
            pump.join(timeout=10)
        ended = time.time()
        result = {'index': i, 'name': r.name, 'options': r.options, 'argv': r.argv(),
                  'status': status or ('ok' if rc == 0 else 'failed'), 'returncode': rc,
                  'started': datetime.datetime.fromtimestamp(started).isoformat(timespec='seconds'),
                  'duration_s': round(ended - started, 1), 'log': logPath,
                  'run': _readJson(info)}
        results.append(result)
        with open(manifest, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2)
        log('[batch] %d/%d %s: %s (exit %s, %.0f s)' % (i, len(runs), r.name or tag,
                                                      result['status'], rc, ended - started))
        if result['status'] != 'ok' and stop_on_fail:
            log('[batch] stopping: --stop-on-fail')
            break
    ok = sum(1 for x in results if x['status'] == 'ok')
    log('[batch] %d of %d ok; manifest %s' % (ok, len(runs), manifest))
    return results


def load(path):
    """The Runs a yaml file describes: `base` (optional) and a `runs` list overriding it."""
    import yaml
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    base = dict(doc.get('base') or {})
    entries = doc.get('runs') or [{}]
    return [Run(**dict(base, **(e or {}))) for e in entries]


def main(argv=None):
    ap = argparse.ArgumentParser(description='Run co-sims one after another (fixs.batch).')
    ap.add_argument('runs', help='yaml with base: and runs: (see fixs/batch.py)')
    ap.add_argument('--out-dir', default=None, help='where the logs and manifest go')
    ap.add_argument('--timeout', type=float, default=None,
                    help='seconds per run, for runs without their own timeout_s')
    ap.add_argument('--stop-on-fail', action='store_true', help='stop at the first failed run')
    a = ap.parse_args(argv)
    try:
        runs = load(a.runs)
    except BatchError as exc:
        sys.exit('[batch] %s: %s' % (a.runs, exc))
    results = run(runs, timeout_s=a.timeout, out_dir=a.out_dir, stop_on_fail=a.stop_on_fail)
    return 0 if results and all(r['status'] == 'ok' for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
