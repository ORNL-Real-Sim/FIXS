"""Compare two trace-replay digest files and name the first exchange that differs.

``replay_core.py --trace ... --digest-out`` and ``replay_core.exe --trace ...
--digest-out`` write the same JSON shape. This reads both and reports whether the
two cores made the same decisions, exchange by exchange::

    python tests/VirEnv/compare_digests.py mlk.cpp.json mlk.py.json

The digests are per HOST TICK, not per exchange, so with ``--substeps N`` there are
N of them per exchange; the report converts an index back to (exchange, substep)
because that is what a person needs in order to go and look at the trace.

Exit status is 0 when every tick matches and 1 otherwise, so it can gate a script.
"""

import argparse
import json
import sys


def load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def compare(a, b, labelA='A', labelB='B'):
    """(dict, dict) -> (ok, list of report lines)."""
    out = []
    ok = True

    for key in ('exchanges', 'substeps', 'egoId'):
        if a.get(key) != b.get(key):
            ok = False
            out.append('  %-12s %s=%r  %s=%r  << the two runs are not comparable'
                       % (key, labelA, a.get(key), labelB, b.get(key)))

    countsA, countsB = a.get('counts', {}), b.get('counts', {})
    out.append('  %-12s %s' % ('exchanges', a.get('exchanges')))
    out.append('  %-12s %s' % ('substeps', a.get('substeps')))
    for key in sorted(set(countsA) | set(countsB)):
        mark = 'OK  ' if countsA.get(key) == countsB.get(key) else 'DIFF'
        if countsA.get(key) != countsB.get(key):
            ok = False
        out.append('  %s %-10s %s=%s  %s=%s'
                   % (mark, key, labelA, countsA.get(key), labelB, countsB.get(key)))

    dA, dB = a.get('digests', []), b.get('digests', [])
    if len(dA) != len(dB):
        ok = False
        out.append('  DIFF digests   %s has %d ticks, %s has %d'
                   % (labelA, len(dA), labelB, len(dB)))

    n = min(len(dA), len(dB))
    diffs = [i for i in range(n) if dA[i] != dB[i]]
    substeps = max(1, int(a.get('substeps') or 1))
    if diffs:
        ok = False
        out.append('  DIFF digests   %d of %d ticks differ' % (len(diffs), n))
        for i in diffs[:5]:
            out.append('       first at tick %d = exchange %d, substep %d  (%s %s / %s %s)'
                       % (i, i // substeps, i % substeps, labelA, dA[i], labelB, dB[i]))
        if len(diffs) > 5:
            out.append('       ... and %d more' % (len(diffs) - 5))
    else:
        out.append('  OK   digests   all %d ticks identical' % n)
    return ok, out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('fileA')
    ap.add_argument('fileB')
    ap.add_argument('--label-a', default=None)
    ap.add_argument('--label-b', default=None)
    args = ap.parse_args(argv)

    labelA = args.label_a or 'A'
    labelB = args.label_b or 'B'
    ok, lines = compare(load(args.fileA), load(args.fileB), labelA, labelB)
    print('=' * 72)
    print('%s : %s' % (labelA, args.fileA))
    print('%s : %s' % (labelB, args.fileB))
    print('=' * 72)
    for line in lines:
        print(line)
    print('')
    print('RESULT: %s' % ('identical decisions' if ok else 'THE TWO CORES DIVERGE'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
