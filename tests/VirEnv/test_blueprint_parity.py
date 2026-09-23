"""The C++ and Python bridges give every vehicle the same CARLA model.

The blueprint is not cosmetic: it fixes bounding_box.extent.x, and both bridges
anchor a vehicle by stepping back that half-length from the nose the wire gives
them. FIXS#358 made the Python choice a function of (seed, vehicle id) after a
shared random draw re-dealt 46 vehicles and walked the ego 150 m off its
baseline. The C++ bridge kept the random draw for months afterwards, and nothing
could see it: the two interfaces did not even take the same arguments.

This runs the C++ picker (CommonLib/BlueprintPick.h, via blueprint_pick.exe)
and holds it to the Python one on three things, each of which alone re-deals
vehicles if it drifts:

- BLAKE2b itself, against hashlib, across the 128-byte block boundaries;
- every pool, membership AND order (the index is taken against the order);
- the pick, for ~300 SUMO-shaped ids in every vClass, under two seeds.

    python -m pytest tests/VirEnv/test_blueprint_parity.py
"""

import hashlib
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, HERE)
import cpp_build                                                    # noqa: E402

pytest.importorskip("carla", reason="Carla/VirEnv/BridgeHelper imports the CARLA PythonAPI")
from Carla.VirEnv.BridgeHelper import BridgeHelper, _kBlueprintSeed  # noqa: E402


def _pattern(n):
    return ''.join(chr(ord('a') + i % 26) for i in range(n)).encode('ascii')


@pytest.fixture(scope='module')
def cpp():
    """The C++ picker's output, split by kind: {'HASH': [...], 'POOL': [...], 'PICK': [...]}."""
    exe = cpp_build.ensureBuilt()['blueprint_pick']
    proc = subprocess.run([exe], capture_output=True, cwd=os.path.dirname(exe))
    assert proc.returncode == 0, 'blueprint_pick.exe failed: %r' % proc.stderr
    out = {'HASH': [], 'POOL': [], 'PICK': []}
    # bytes, split on single spaces: ids are hex so the lines are pure ASCII, and
    # a field is never empty -- but split(' ') rather than split() keeps it that way.
    for line in proc.stdout.decode('ascii').splitlines():
        kind, _, rest = line.partition(' ')
        out[kind].append(rest.split(' '))
    assert out['HASH'] and out['POOL'] and out['PICK'], 'blueprint_pick.exe printed nothing'
    return out


def test_blake2b_matches_hashlib(cpp):
    """The C++ BLAKE2b is Python's hashlib.blake2b(digest_size=8), bit for bit,
    including 0 bytes and either side of every block boundary it will ever see."""
    wrong = [(n, h, hashlib.blake2b(_pattern(int(n)), digest_size=8).hexdigest())
             for n, h in cpp['HASH']
             if hashlib.blake2b(_pattern(int(n)), digest_size=8).hexdigest() != h]
    assert not wrong, 'BLAKE2b disagrees at (len, c++, python): %s' % wrong
    assert {int(n) for n, _ in cpp['HASH']} >= {0, 127, 128, 129, 255, 256, 257}


def test_the_pools_are_the_same_lists_in_the_same_order(cpp):
    cppPools = {}
    for vClass, i, bp in cpp['POOL']:
        cppPools.setdefault(vClass, []).append((int(i), bp))
    cppPools = {k: [bp for _, bp in sorted(v)] for k, v in cppPools.items()}
    pyPools = {k: list(v) for k, v in BridgeHelper._BY_VCLASS.items()}
    assert sorted(cppPools) == sorted(pyPools), 'the two bridges know different vClasses'
    for vClass in pyPools:
        assert cppPools[vClass] == pyPools[vClass], (
            '%s pool differs (order matters -- the index is taken against it):\n'
            '  c++:    %s\n  python: %s' % (vClass, cppPools[vClass], pyPools[vClass]))


def test_every_vehicle_gets_the_same_model(cpp):
    wrong, seen = [], 0
    try:
        for seed, vClass, idHex, cppBp in cpp['PICK']:
            BridgeHelper.setBlueprintSeed(int(seed))
            vehId = bytes.fromhex(idHex).decode('utf-8')
            pyBp = BridgeHelper.map_Sumo_vClass_to_Carla_blueprintId(vClass, vehId)
            seen += 1
            if pyBp != cppBp:
                wrong.append((seed, vClass, vehId, cppBp, pyBp))
    finally:
        BridgeHelper.setBlueprintSeed(_kBlueprintSeed)
    assert seen > 4000, 'compared only %d picks' % seen
    assert not wrong, ('%d of %d vehicles get a different model in the C++ bridge; '
                       'first few (seed, vClass, id, c++, python): %s'
                       % (len(wrong), seen, wrong[:5]))


def test_the_seed_is_part_of_the_answer(cpp):
    """Two seeds must give different picks for most vehicles, or the seed is not
    reaching the hash and every run is the same run whatever the config says."""
    by = {}
    for seed, vClass, idHex, bp in cpp['PICK']:
        if vClass == 'passenger':
            by.setdefault(idHex, {})[seed] = bp
    differ = sum(1 for picks in by.values() if len(set(picks.values())) > 1)
    assert differ > len(by) // 2, 'only %d of %d ids changed model with the seed' % (differ, len(by))
