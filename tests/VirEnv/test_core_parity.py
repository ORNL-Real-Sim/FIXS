"""Parity guard: the Python VirEnvCore makes the same decisions as the C++ one.

Two implementations of one bridge have to agree, and #325 names the place where
disagreement is most expensive: **who owns the ego**. ``carlaOwnsId``
(TrafficHelper) and ``carlaOwnsEgo`` (VirCarlaEnv) once answered that differently
and produced a run where TrafficLayer kept the ego on traffic-sim kinematics while
a physics ego free-ran with no anchor -- 0.7 m of agreement for 30 s, then
background traffic teleported through the physics body at 10.34 m/s and separation
ended at 124.8 m. Two processes deciding one thing with no arbiter.

A core that disagrees about ego ownership disagrees HERE first, in which verbs it
calls for which id. So the guard is the verb transcript, and it is compared, not
assumed.

What is compared
----------------
The transcript is grouped into steps by the ``== step t=...`` markers the driver
emits, and the two runs must produce the same step labels in the same order with
the same MULTISET of events inside each. Order WITHIN a step is deliberately not
compared: it is C++ ``unordered_map`` iteration order, which the standard leaves
unspecified and an insertion-ordered Python dict cannot reproduce. Two vehicles'
poses applied in either order within one refresh is not a decision -- they are
independent writes, and Carla batches them before they reach the server anyway.

Everything else IS compared, character for character: the verb, the handle, the
pose to three decimals, the light bits, the ego id.

How it runs without MSVC
------------------------
The C++ side is pinned as a committed golden transcript, so the Python test runs
anywhere. When ``replay_core.exe`` has been built, a second test regenerates the
golden from it and compares -- so the golden cannot silently go stale against a
change to VirEnvCore.cpp. Build it with ``tests/VirEnvCore/build_and_run.bat``.
"""

import json
import os
import re
import subprocess
import sys

import pytest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, REPO_ROOT)

PY_REPLAY = os.path.join(REPO_ROOT, 'tests', 'VirEnv', 'replay_core.py')
CPP_REPLAY_EXE = os.path.join(REPO_ROOT, 'tests', 'VirEnvCore', 'replay_core.exe')
GOLDEN = os.path.join(os.path.dirname(__file__), 'golden', 'replay_core_cpp.json')

#: ``--- <title> transcript (N) ---`` then one two-space-indented event per line.
_HEADER = re.compile(r'^--- (?P<title>.+?) transcript \((?P<n>\d+)\) ---$')

#: The C++ scenario titles, in the order replay_core.cpp runs them, mapped to the
#: keys replay_core.py --json uses. Kept explicit so a renamed scenario fails here
#: rather than quietly comparing nothing.
SCENARIO_TITLES = {'Carla-style': 'scenario1', 'CarMaker-style': 'scenario2'}


def parseTranscripts(stdout):
    """(string) -> dict -- ``{scenario key: [event, ...]}`` from a replay's output."""
    out, current = {}, None
    for line in stdout.splitlines():
        m = _HEADER.match(line.strip())
        if m:
            title = m.group('title')
            assert title in SCENARIO_TITLES, (
                'unknown scenario %r -- update SCENARIO_TITLES' % title)
            current = out.setdefault(SCENARIO_TITLES[title], [])
            continue
        if current is not None and line.startswith('  ') and not line.startswith('   '):
            current.append(line[2:])
        elif current is not None and not line.startswith(' '):
            current = None          # the transcript ended at the PASS line
    return out


def groupBySteps(events):
    """(list) -> list -- ``[(step label, sorted events), ...]``.

    The sort inside a step is what makes the comparison independent of
    ``unordered_map`` iteration order; see the module docstring for why that order
    is not a decision worth pinning.
    """
    groups, label, bucket = [], '<before first step>', []
    for e in events:
        if e.startswith('== '):
            if bucket or groups or label != '<before first step>':
                groups.append((label, sorted(bucket)))
            label, bucket = e[3:], []
        else:
            bucket.append(e)
    groups.append((label, sorted(bucket)))
    return groups


def runPythonReplay():
    """() -> dict -- the Python transcripts. Its own assertions run too."""
    proc = subprocess.run([sys.executable, PY_REPLAY, '--json'],
                          capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode == 0, (
        'python replay_core.py failed:\n%s\n%s' % (proc.stdout, proc.stderr))
    return json.loads(proc.stdout)


def runCppReplay():
    """() -> dict -- the C++ transcripts, parsed from replay_core.exe stdout."""
    proc = subprocess.run([CPP_REPLAY_EXE], capture_output=True, text=True,
                          cwd=os.path.dirname(CPP_REPLAY_EXE))
    assert proc.returncode == 0, (
        'replay_core.exe failed:\n%s\n%s' % (proc.stdout, proc.stderr))
    return parseTranscripts(proc.stdout)


def loadGolden():
    with open(GOLDEN, encoding='utf-8') as f:
        return json.load(f)


def assertSameDecisions(expected, actual, whatExpected, whatActual):
    """Compare two transcript dicts step by step, reporting the first difference."""
    assert sorted(expected) == sorted(actual), (
        'different scenarios: %s has %s, %s has %s'
        % (whatExpected, sorted(expected), whatActual, sorted(actual)))

    for key in sorted(expected):
        exp = groupBySteps(expected[key])
        act = groupBySteps(actual[key])
        assert [g[0] for g in exp] == [g[0] for g in act], (
            '%s: step sequence differs.\n  %s: %s\n  %s: %s'
            % (key, whatExpected, [g[0] for g in exp], whatActual, [g[0] for g in act]))
        for (label, expEvents), (_, actEvents) in zip(exp, act):
            if expEvents != actEvents:
                onlyExpected = [e for e in expEvents if e not in actEvents]
                onlyActual = [e for e in actEvents if e not in expEvents]
                pytest.fail(
                    '%s, step "%s": the two cores decided differently.\n'
                    '  only in %s: %s\n'
                    '  only in %s: %s'
                    % (key, label, whatExpected, onlyExpected, whatActual, onlyActual))


def test_python_core_matches_cpp_golden():
    """The Python VirEnvCore reproduces the C++ core's verb decisions."""
    assertSameDecisions(loadGolden(), runPythonReplay(),
                        'the C++ golden', 'the Python core')


@pytest.mark.skipif(not os.path.isfile(CPP_REPLAY_EXE),
                    reason='replay_core.exe not built '
                           '(run tests/VirEnvCore/build_and_run.bat)')
def test_cpp_golden_is_current():
    """The committed golden still matches what VirEnvCore.cpp actually does.

    Without this, a change to the C++ core would leave the golden -- and therefore
    the Python parity test -- asserting behaviour that no longer exists.
    """
    assertSameDecisions(loadGolden(), runCppReplay(),
                        'the committed golden', 'replay_core.exe')


if __name__ == '__main__':
    # Regenerate the golden from the C++ build: python test_core_parity.py --update
    if '--update' in sys.argv:
        os.makedirs(os.path.dirname(GOLDEN), exist_ok=True)
        transcripts = runCppReplay()
        with open(GOLDEN, 'w', encoding='utf-8') as f:
            json.dump(transcripts, f, indent=1)
            f.write('\n')
        print('golden written from replay_core.exe -> %s' % GOLDEN)
        for key, events in sorted(transcripts.items()):
            print('  %s: %d events' % (key, len(events)))
    else:
        sys.exit(pytest.main([__file__, '-v']))
