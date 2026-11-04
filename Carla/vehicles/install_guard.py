"""Who else is using this CARLA install, and can a stock asset be written now.

Several CARLA servers and editors may run from one install. Each keeps the packages it
loaded open, so a save over them fails. This module finds those processes and tests the
one stock file the importer edits; it never stops a process.
"""

import json
import os
import platform
import subprocess

PROCESS_NAMES = ("ue4editor", "carlaue4")


def _norm(p):
    return os.path.normcase(os.path.abspath(p)).replace("\\", "/").rstrip("/") + "/"


def list_processes():
    """[{pid, name, exe, cmd}] for UE4Editor / CarlaUE4 processes; [] if they cannot be listed."""
    if platform.system() == "Windows":
        ps = ("Get-CimInstance Win32_Process | Where-Object { $_.Name -like 'UE4Editor*' -or "
              "$_.Name -like 'CarlaUE4*' } | Select-Object ProcessId,Name,ExecutablePath,"
              "CommandLine | ConvertTo-Json -Compress")
        try:
            out = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                                 capture_output=True, text=True, timeout=30).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return []
        if not out:
            return []
        try:
            rows = json.loads(out)
        except ValueError:
            return []
        rows = rows if isinstance(rows, list) else [rows]
        return [{"pid": r.get("ProcessId"), "name": r.get("Name") or "",
                 "exe": r.get("ExecutablePath") or "", "cmd": r.get("CommandLine") or ""}
                for r in rows]
    try:
        out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True,
                             timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    rows = []
    for line in out.splitlines():
        pid, _, args = line.strip().partition(" ")
        exe = args.split(" ", 1)[0]
        name = os.path.basename(exe)
        if pid.isdigit() and name.lower().startswith(PROCESS_NAMES):
            rows.append({"pid": int(pid), "name": name, "exe": exe, "cmd": args})
    return rows


def instances_on_install(processes, carla_root, exclude_pids=()):
    """The processes whose executable or command line lies under `carla_root`."""
    root = _norm(carla_root)
    hits = []
    for p in processes:
        if p.get("pid") in exclude_pids:
            continue
        if not str(p.get("name", "")).lower().startswith(PROCESS_NAMES):
            continue
        text = (str(p.get("exe", "")) + " " + str(p.get("cmd", ""))).replace("\\", "/")
        if root.lower() in text.lower() or root.rstrip("/").lower() + '"' in text.lower():
            hits.append(p)
    return hits


def describe(procs):
    return "\n".join("    pid {:>6}  {}  {}".format(p.get("pid"), p.get("name"),
                                                    (p.get("cmd") or p.get("exe") or "")[:160])
                     for p in procs)


def file_in_use(path):
    """True when another process holds `path` open (Windows: an exclusive open fails)."""
    if not os.path.isfile(path):
        return False
    if platform.system() != "Windows":
        return False  # POSIX does not lock on open; the instance check is what guards there
    import ctypes
    from ctypes import wintypes
    k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    k32.CreateFileW.restype = wintypes.HANDLE
    k32.CreateFileW.argtypes = (wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
                                wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE)
    GENERIC_READ, OPEN_EXISTING, NO_SHARING = 0x80000000, 3, 0
    h = k32.CreateFileW(path, GENERIC_READ, NO_SHARING, None, OPEN_EXISTING, 0, None)
    if h == wintypes.HANDLE(-1).value:
        return ctypes.get_last_error() == 32  # ERROR_SHARING_VIOLATION
    k32.CloseHandle(h)
    return False
