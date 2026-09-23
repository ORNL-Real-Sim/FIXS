"""Simulator-free guard for the Python VirEnvCore (#325).

The Python half of ``tests/VirEnvCore/``: the same mock backend, the same replay
scenarios, plus ``test_core_parity.py``, which asserts the two implementations
make the same verb decisions. Everything here runs with no CARLA, no CarMaker and
no TrafficLayer.
"""
