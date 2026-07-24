"""Probe VISSIM 2022 (VISSIM.Vissim.2200) COM dispatch health.

Exit 0 = the ProgID answers (and we immediately close the throwaway probe
instance so it leaves nothing behind). Exit 1 = dispatch failed -- e.g. the
Windows 11 24H2 zombie state, HRESULT 0x80080005 "Server execution failed".

run_cm_office_demo.bat calls this BEFORE launching the co-sim: if the probe
fails, the bat reaps the zombie VISSIM + stale locks and retries, so a single
bad dispatch no longer wedges every subsequent run. Pure pywin32, no args.
"""
import sys

try:
    import pythoncom
    import win32com.client
except Exception:
    # pywin32 not available in this interpreter -- we cannot health-check, so do
    # NOT block the demo (exit 0 = "skip"). Point %PYTHON% at a pywin32-equipped
    # env (e.g. the realsim_dev conda env) to make the check active.
    sys.exit(0)

try:
    pythoncom.CoInitialize()
    v = win32com.client.Dispatch("VISSIM.Vissim.2200")
except Exception:
    # dispatch is poisoned (zombie / stale token) -- signal the bat to clean up
    sys.exit(1)

# Healthy: close the throwaway probe instance so it does not linger as a second
# VISSIM (which would itself become the next zombie / contend for the license).
# TrafficLayer's DSProxy spawns its OWN VISSIM for the actual co-sim.
try:
    v.Exit()
except Exception:
    pass
sys.exit(0)
