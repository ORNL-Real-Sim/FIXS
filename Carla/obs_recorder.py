"""Record a FIXS co-sim run with OBS: this run's windows, laid out as an app's panels, into a folder.

An app's recorder gives a scene name, a title and its Panels, and calls record() from a
process started with spawn(). record() uses the open OBS (WebSocket :4455, or OBS_WS_URL and
OBS_WS_PASSWORD); when another run holds it, it is recording, or none is open, the run starts an OBS
of its own, so runs side by side are each recorded. It finds only this run's windows, records into
the app's out_dir once the app says the run is ready, and stops when the app says it is over or the
watched process exits.
Windows only. Setup: Carla/README.md, "Recording a run with OBS".
"""
import base64
import collections
import ctypes
import ctypes.wintypes as wt
import hashlib
import json
import os
import subprocess
import sys
import time
import uuid

u = ctypes.windll.user32 if os.name == "nt" else None
k = ctypes.windll.kernel32 if os.name == "nt" else None

#: Layout units: panels are placed on this canvas and scaled to OBS's own.
CANVAS = (1920, 1080)
#: sumo-gui's bars at 96 dpi, top and bottom, when its map canvas can't be found.
SUMO_BARS = (85, 61)

#: A window to capture: `key` names a finder; x, y, w, h on CANVAS; `label` is drawn in its top-left
#: corner, or bottom-left with label_bottom (to keep clear of a HUD).
Panel = collections.namedtuple("Panel", "key x y w h label label_bottom", defaults=("", False))

FINDERS = {
    "carla": lambda t, c, e: c == "UnrealWindow" and t.startswith("CarlaUE4"),
    "sumo": lambda t, c, e: e.lower() == "sumo-gui.exe" and c == "FXTopWindow",
}


def title_finder(prefix):
    """A finder for a window whose title starts with `prefix`, e.g. 'FIXS view: chase'."""
    return lambda t, c, e: t.startswith(prefix)


def log(*a):
    print("[rec]", *a, flush=True)


def spawn(script, *args):
    """Start `script` beside the caller, outside its process tree, so killing that tree leaves it time to stop OBS."""
    cmd = [sys.executable, os.path.abspath(script), *args]
    launcher = "import subprocess, sys; subprocess.Popen(sys.argv[1:], creationflags=0x200)"
    subprocess.call([sys.executable, "-c", launcher, *cmd])


# ---- OBS WebSocket v5 ----------------------------------------------------------------

def obs_password():
    """OBS_WS_PASSWORD, else the password in this machine's obs-websocket settings."""
    if "OBS_WS_PASSWORD" in os.environ:
        return os.environ["OBS_WS_PASSWORD"]
    path = os.path.join(os.environ.get("APPDATA", ""), "obs-studio", "plugin_config", "obs-websocket", "config.json")
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f).get("server_password", "")
    except (OSError, ValueError):
        return ""


def obs_connect(url, password):
    import websocket
    ws = websocket.create_connection(url, timeout=10)
    hello = json.loads(ws.recv())["d"]
    ident = {"rpcVersion": 1}
    auth = hello.get("authentication")
    if auth:
        secret = base64.b64encode(hashlib.sha256((password + auth["salt"]).encode()).digest())
        ident["authentication"] = base64.b64encode(
            hashlib.sha256(secret + auth["challenge"].encode()).digest()).decode()
    ws.send(json.dumps({"op": 1, "d": ident}))
    reply = ws.recv()  # empty: OBS closed the socket on a wrong password
    if not reply or json.loads(reply)["op"] != 2:
        raise RuntimeError("OBS refused the connection (wrong OBS_WS_PASSWORD?)")
    return ws


def call(ws, rtype, data=None):
    rid = str(uuid.uuid4())
    ws.send(json.dumps({"op": 6, "d": {"requestType": rtype, "requestId": rid, "requestData": data or {}}}))
    while True:
        m = json.loads(ws.recv())
        if m["op"] == 7 and m["d"]["requestId"] == rid:
            st = m["d"]["requestStatus"]
            if not st["result"]:
                raise RuntimeError(f"OBS {rtype}: {st.get('code')} {st.get('comment')}")
            return m["d"].get("responseData", {})


OWNER = {"realm": "OBS_WEBSOCKET_DATA_REALM_GLOBAL", "slotName": "mlkrec owner"}


def restore_dir(ws, folder):
    """Put OBS's recording folder back and forget this run."""
    for _ in range(20):  # OBS refuses a new folder until the file is closed
        try:
            call(ws, "SetRecordDirectory", {"recordDirectory": folder})
            break
        except RuntimeError:
            time.sleep(0.25)
    call(ws, "SetPersistentData", {**OWNER, "slotValue": ""})


def clean_up_killed_run(ws):
    """A run killed outright can leave OBS recording into its folder: stop that and put OBS's folder back."""
    owner = call(ws, "GetPersistentData", OWNER).get("slotValue")
    if not isinstance(owner, dict) or alive(owner.get("pid", 0)):
        return
    if call(ws, "GetRecordStatus")["outputActive"]:
        log("stopped a recording left by a run that was killed:", call(ws, "StopRecord")["outputPath"])
    restore_dir(ws, owner["dir"])


def stop_on_console_close(ws, prev_dir):
    """Closing the console kills the whole run at once: hand OBS the stop and the folder reset in one batch."""
    batch = json.dumps({"op": 8, "d": {"requestId": "close", "executionType": 0, "requests": [
        {"requestType": "StopRecord"},
        {"requestType": "Sleep", "requestData": {"sleepMillis": 1500}},
        {"requestType": "SetRecordDirectory", "requestData": {"recordDirectory": prev_dir}}]}})

    def handler(event):
        if event in (2, 5, 6):  # CTRL_CLOSE, CTRL_LOGOFF, CTRL_SHUTDOWN
            ws.send(batch)
        return False
    handler = ctypes.WINFUNCTYPE(wt.BOOL, wt.DWORD)(handler)
    k.SetConsoleCtrlHandler(handler, True)
    return handler  # kept alive by the caller


# ---- one OBS per recording -----------------------------------------------------------

#: The OBS the user keeps open; runs that find it taken start their own on the next free port.
OBS_PORT = 4455
SPARE_PORTS = range(4456, 4476)
OBS_CONFIG = os.path.join(os.environ.get("APPDATA", ""), "obs-studio")


def _claim(port):
    """Hold the lock on `port` for this process's life; None if another recorder holds it."""
    import msvcrt
    import tempfile
    folder = os.path.join(tempfile.gettempdir(), "fixs_obs")
    os.makedirs(folder, exist_ok=True)
    f = open(os.path.join(folder, f"{port}.lock"), "a+")
    f.seek(0)
    try:
        msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
        return f
    except OSError:
        f.close()
        return None


def obs_exe():
    """OBS_EXE, else the running OBS's own exe, else the default install."""
    if os.environ.get("OBS_EXE"):
        return os.environ["OBS_EXE"]
    for pid, _, name in _procs():
        if name.lower() == "obs64.exe" and _path_of(pid):
            return _path_of(pid)
    exe = os.path.join(os.environ.get("ProgramFiles", r"C:\Program Files"), "obs-studio", "bin", "64bit", "obs64.exe")
    if os.path.isfile(exe):
        return exe
    raise RuntimeError("OBS not found; set OBS_EXE to obs64.exe")


def _ini_get(path, section, key):
    sec = None
    for line in open(path, encoding="utf-8-sig", errors="replace"):
        line = line.strip()
        if line.startswith("["):
            sec = line[1:-1]
        elif sec == section and line.startswith(key + "="):
            return line[len(key) + 1:]
    return None


def _ini_set(path, section, values):
    """Set `values` ({key: value}) in `section` of an OBS ini, leaving the rest as it is."""
    out, sec, left = [], None, dict(values)
    for line in open(path, encoding="utf-8-sig", errors="replace").read().splitlines():
        if line.startswith("["):
            sec = line.strip()[1:-1]
        key = line.split("=", 1)[0]
        if sec == section and "=" in line and key in left:
            line = f"{key}={left.pop(key)}"
        out.append(line)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")


class SpareObs:
    """A second OBS for one run, on its own WebSocket port.

    It runs --portable, so its settings live in <OBS install>/config and the user's OBS settings are
    never touched; its profile is a copy of the user's, so the video settings match.
    """

    def __init__(self, port):
        import shutil
        self.port, self.name, slug = port, f"FIXS rec {port}", f"FIXS_rec_{port}"
        self.password = uuid.uuid4().hex
        exe = obs_exe()
        root = os.path.normpath(os.path.join(os.path.dirname(exe), "..", "..", "config", "obs-studio"))
        for n in ("global.ini", "user.ini"):  # present, so OBS skips its first-run wizard
            if os.path.isfile(os.path.join(OBS_CONFIG, n)) and not os.path.isfile(os.path.join(root, n)):
                os.makedirs(root, exist_ok=True)
                shutil.copy(os.path.join(OBS_CONFIG, n), root)
        ws_cfg = os.path.join(root, "plugin_config", "obs-websocket", "config.json")
        os.makedirs(os.path.dirname(ws_cfg), exist_ok=True)
        with open(ws_cfg, "w", encoding="utf-8") as f:
            json.dump({"server_enabled": True, "auth_required": True, "first_load": False,
                       "alerts_enabled": False}, f)
        user_ini = os.path.join(OBS_CONFIG, "user.ini")
        user_profile = _ini_get(user_ini, "Basic", "ProfileDir") if os.path.isfile(user_ini) else None
        self.profile = os.path.join(root, "basic", "profiles", slug)
        shutil.rmtree(self.profile, ignore_errors=True)
        shutil.copytree(os.path.join(OBS_CONFIG, "basic", "profiles", user_profile or "Untitled"), self.profile)
        _ini_set(os.path.join(self.profile, "basic.ini"), "General", {"Name": self.name})
        self.collection = os.path.join(root, "basic", "scenes", slug + ".json")
        os.makedirs(os.path.dirname(self.collection), exist_ok=True)
        # One empty scene: obs-websocket crashes on GetCurrentProgramScene in a collection with none.
        scene = {"id": "scene", "versioned_id": "scene", "name": "Scene", "uuid": str(uuid.uuid4()),
                 "settings": {"items": [], "id_counter": 0, "custom_size": False}}
        with open(self.collection, "w", encoding="utf-8") as f:
            json.dump({"name": self.name, "sources": [scene], "scene_order": [{"name": "Scene"}],
                       "current_scene": "Scene", "current_program_scene": "Scene"}, f)
        self.proc = subprocess.Popen(
            [exe, "--portable", "--multi", "--disable-updater", "--disable-shutdown-check",
             "--profile", self.name, "--collection", self.name,
             f"--websocket_port={port}", f"--websocket_password={self.password}"],
            cwd=os.path.dirname(exe))
        log(f"started a second OBS for this run on :{port}")

    def connect(self, timeout=60.0):
        """Its WebSocket once OBS is fully up: a request sent while it is still starting can hang it."""
        end, ws = time.time() + timeout, None
        while True:
            try:
                # OBS shows its main window last. Minimised or hidden, it hangs on requests, so it goes to the back.
                main = next(h for h, t in _shown_windows(self.proc.pid) if t.startswith("OBS "))
                ws = ws or obs_connect(f"ws://127.0.0.1:{self.port}", self.password)
                call(ws, "GetSceneList")
                u.SetWindowPos(main, 1, 0, 0, 0, 0, 0x13)  # to the back: NOSIZE|NOMOVE|NOACTIVATE
                return ws
            except Exception:
                if time.time() > end or self.proc.poll() is not None:
                    raise RuntimeError(f"the second OBS on :{self.port} did not come up")
                time.sleep(1.0)

    def close(self):
        """End it (its recording is already stopped and written), and remove its profile and collection."""
        self.proc.kill()  # a normal quit can hang
        try:
            self.proc.wait(30)
        except subprocess.TimeoutExpired:
            log(f"the second OBS (pid {self.proc.pid}) is slow to exit")
        import shutil
        shutil.rmtree(self.profile, ignore_errors=True)
        for path in (self.collection, self.collection + ".bak"):
            try:
                os.remove(path)
            except OSError:
                pass


def open_obs():
    """(ws, spare, lock): the user's OBS when no other run has it and it is not recording, else a SpareObs."""
    if os.environ.get("OBS_WS_URL"):
        return obs_connect(os.environ["OBS_WS_URL"], obs_password()), None, None
    lock = _claim(OBS_PORT)
    if lock:
        try:
            ws = obs_connect(f"ws://127.0.0.1:{OBS_PORT}", obs_password())
            try:
                clean_up_killed_run(ws)
            except Exception as e:
                log(f"could not clean up after an earlier run: {e}")
            if not call(ws, "GetRecordStatus")["outputActive"]:
                return ws, None, lock
            log("the open OBS is already recording; this run gets its own")
            ws.close()
        except Exception as e:
            log(f"no OBS answering on :{OBS_PORT} ({type(e).__name__}); this run starts its own")
        lock.close()
    else:
        log("the open OBS is recording another run; this run gets its own")
    for port in SPARE_PORTS:
        lock = _claim(port)
        if lock:
            spare = SpareObs(port)
            try:
                return spare.connect(), spare, lock
            except Exception:
                spare.close()
                raise
    raise RuntimeError(f"all {len(SPARE_PORTS)} spare OBS ports are taken")


# ---- windows -------------------------------------------------------------------------

def _path_of(pid):
    h = k.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
    if not h:
        return ""
    buf, n = ctypes.create_unicode_buffer(512), wt.DWORD(512)
    k.QueryFullProcessImageNameW(h, 0, buf, ctypes.byref(n))
    k.CloseHandle(h)
    return buf.value


def _exe_of(pid):
    return os.path.basename(_path_of(pid))


def _shown_windows(pid):
    """(hwnd, title) of the visible top-level windows of `pid`."""
    return [(w[0], w[1]) for w in windows() if w[4] == pid]


def windows():
    """(hwnd, title, class, exe, pid) of every visible titled window."""
    out = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wt.HWND, wt.LPARAM)
    def cb(h, _):
        n = u.GetWindowTextLengthW(h)
        if u.IsWindowVisible(h) and n:
            t, c, p = ctypes.create_unicode_buffer(n + 1), ctypes.create_unicode_buffer(256), wt.DWORD()
            u.GetWindowTextW(h, t, n + 1)
            u.GetClassNameW(h, c, 256)
            u.GetWindowThreadProcessId(h, ctypes.byref(p))
            out.append((h, t.value, c.value, _exe_of(p.value), p.value))
        return True

    u.EnumWindows(cb, 0)
    return out


class _PE(ctypes.Structure):
    _fields_ = [("dwSize", wt.DWORD), ("cntUsage", wt.DWORD), ("th32ProcessID", wt.DWORD),
                ("th32DefaultHeapID", ctypes.c_size_t), ("th32ModuleID", wt.DWORD),
                ("cntThreads", wt.DWORD), ("th32ParentProcessID", wt.DWORD),
                ("pcPriClassBase", ctypes.c_long), ("dwFlags", wt.DWORD), ("szExeFile", ctypes.c_wchar * 260)]


def _procs():
    """(pid, parent pid, exe name) of every process."""
    k.CreateToolhelp32Snapshot.restype = wt.HANDLE  # the default int truncates a 64-bit handle
    k.Process32FirstW.argtypes = k.Process32NextW.argtypes = [wt.HANDLE, ctypes.POINTER(_PE)]
    k.CloseHandle.argtypes = [wt.HANDLE]
    snap = k.CreateToolhelp32Snapshot(0x2, 0)  # TH32CS_SNAPPROCESS
    pe, out = _PE(), []
    pe.dwSize = ctypes.sizeof(_PE)
    ok = k.Process32FirstW(snap, ctypes.byref(pe))
    while ok:
        out.append((pe.th32ProcessID, pe.th32ParentProcessID, pe.szExeFile))
        ok = k.Process32NextW(snap, ctypes.byref(pe))
    k.CloseHandle(snap)
    return out


def _parents():
    """{pid: parent pid} for every process."""
    return {p: pp for p, pp, _ in _procs()}


def descendants(pid):
    """`pid` and every process under it."""
    parents, fam, todo = _parents(), set(), [pid]
    while todo:
        p = todo.pop()
        fam.add(p)
        todo += [c for c, pp in parents.items() if pp == p and c not in fam]
    return fam


def run_family(pid):
    """Pids started by the run_cosim that started `pid`: this run's CARLA, sumo-gui, bridge."""
    parents = _parents()
    root, fam = parents.get(pid), set()
    todo = [root] if root else []
    while todo:
        p = todo.pop()
        fam.add(p)
        todo += [c for c, pp in parents.items() if pp == p and c not in fam]
    return fam


def find(pred, family=None):
    """The first window `pred` matches, among this run's processes when `family` is given."""
    return next((w for w in windows() if pred(*w[1:4]) and (family is None or w[4] in family)), None)


def tag_title(w, tag):
    """Give this run's window a title no other run's window has, so OBS can match it exactly."""
    if not w[1].endswith(tag):
        u.SetWindowTextW(w[0], f"{w[1]} {tag}")
    return (w[0], u_title(w[0])) + tuple(w[2:])


def u_title(h):
    n = u.GetWindowTextLengthW(h)
    t = ctypes.create_unicode_buffer(n + 1)
    u.GetWindowTextW(h, t, n + 1)
    return t.value


def sumo_crop(h):
    """(left, top, right, bottom) pixels of sumo-gui's client area around its map canvas."""
    c, o = wt.RECT(), wt.POINT(0, 0)
    u.GetClientRect(wt.HWND(h), ctypes.byref(c))
    u.ClientToScreen(wt.HWND(h), ctypes.byref(o))
    canvases = []

    @ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
    def each(ch, _):
        cls = ctypes.create_unicode_buffer(32)
        u.GetClassNameW(ch, cls, 32)
        if cls.value == "FXGLCanvas" and u.IsWindowVisible(ch):
            canvases.append(window_rect(ch))
        return True
    u.EnumChildWindows(wt.HWND(h), each, 0)
    if not canvases:
        s = u.GetDpiForWindow(wt.HWND(h)) / 96 or 1.0
        return 0, round(SUMO_BARS[0] * s), 0, round(SUMO_BARS[1] * s)
    x, y, w, hh = max(canvases, key=lambda r: r[2] * r[3])
    x, y = x - o.x, y - o.y
    return x, y, c.right - x - w, c.bottom - y - hh


def fill_crop(h, crop, w, hh):
    """`crop` widened on two sides so what is left of h's client area has a w x hh panel's shape."""
    c = wt.RECT()
    u.GetClientRect(wt.HWND(h), ctypes.byref(c))
    cw, ch = c.right - crop[0] - crop[2], c.bottom - crop[1] - crop[3]
    if cw <= 0 or ch <= 0:
        return crop
    if cw * hh > ch * w:
        cut = (cw - ch * w / hh) / 2
        return round(crop[0] + cut), crop[1], round(crop[2] + cut), crop[3]
    cut = (ch - cw * hh / w) / 2
    return crop[0], round(crop[1] + cut), crop[2], round(crop[3] + cut)


def size_sumo(h, w, hh):
    """Size sumo-gui so its map canvas is a w x hh panel's shape at the window's dpi."""
    s = u.GetDpiForWindow(wt.HWND(h)) / 96 or 1.0
    crop = sumo_crop(h)
    x, y, ww, wh = window_rect(h)
    c = wt.RECT()
    u.GetClientRect(wt.HWND(h), ctypes.byref(c))
    cw, ch = round(w * s) + crop[0] + crop[2], round(hh * s) + crop[1] + crop[3]
    if (c.right, c.bottom) != (cw, ch):
        place(h, x, y, cw + ww - c.right, ch + wh - c.bottom)


def window_rect(h):
    r = wt.RECT()
    u.GetWindowRect(wt.HWND(h), ctypes.byref(r))
    return r.left, r.top, r.right - r.left, r.bottom - r.top


def place(h, x, y, w, hh):
    u.ShowWindow(h, 9)  # SW_RESTORE
    u.SetWindowPos(h, 0, x, y, w, hh, 0x0004 | 0x0010)  # NOZORDER | NOACTIVATE


def alive(pid):
    h = k.OpenProcess(0x1000, False, pid)
    if not h:
        return False
    code = wt.DWORD()
    k.GetExitCodeProcess(h, ctypes.byref(code))
    k.CloseHandle(h)
    return code.value == 259  # STILL_ACTIVE


# ---- scene ---------------------------------------------------------------------------

def window_source(w):
    esc = lambda s: s.replace("#", "#22").replace(":", "#3A")
    # priority 0: match the exact title, which tag_title() made unique to this run.
    return {"window": f"{esc(w[1])}:{esc(w[2])}:{esc(w[3])}", "priority": 0, "method": 2,
            "cursor": False, "client_area": True}


class Scene:
    """Builds one OBS scene on CANVAS units; items are kept in the order they are put, back to front."""

    def __init__(self, ws, name, prefix):
        self.ws, self.name, self.prefix = ws, name, prefix
        if name not in [s["sceneName"] for s in call(ws, "GetSceneList")["scenes"]]:
            call(ws, "CreateScene", {"sceneName": name})
        self.items = {i["sourceName"]: i["sceneItemId"]
                      for i in call(ws, "GetSceneItemList", {"sceneName": name})["sceneItems"]}
        self.inputs = {i["inputName"] for i in call(ws, "GetInputList")["inputs"]}
        video = call(ws, "GetVideoSettings")
        self.k = min(video["baseWidth"] / CANVAS[0], video["baseHeight"] / CANVAS[1])
        self.order = []

    def item(self, name, kind, settings):
        name = f"{self.prefix} {name}"
        if name in self.inputs and call(self.ws, "GetInputSettings", {"inputName": name})["inputKind"] != kind:
            # An input keeps its kind: one left by an older layout is replaced.
            call(self.ws, "RemoveInput", {"inputName": name})
            for _ in range(40):  # OBS frees the name a moment later
                if name not in {i["inputName"] for i in call(self.ws, "GetInputList")["inputs"]}:
                    break
                time.sleep(0.05)
            self.inputs.discard(name)
            self.items.pop(name, None)
        if name in self.items or name in self.inputs:
            call(self.ws, "SetInputSettings", {"inputName": name, "inputSettings": settings})
            if name in self.items:
                return self.items[name]
            return call(self.ws, "CreateSceneItem", {"sceneName": self.name, "sourceName": name})["sceneItemId"]
        return call(self.ws, "CreateInput", {"sceneName": self.name, "inputName": name, "inputKind": kind,
                                             "inputSettings": settings})["sceneItemId"]

    def put(self, sid, x, y, w=None, h=None, crop=(0, 0, 0, 0)):
        k = self.k
        tf = {"positionX": x * k, "positionY": y * k, "alignment": 5,
              "cropLeft": crop[0], "cropTop": crop[1], "cropRight": crop[2], "cropBottom": crop[3]}
        if w:
            tf.update({"boundsType": "OBS_BOUNDS_SCALE_INNER", "boundsWidth": w * k, "boundsHeight": h * k,
                       "boundsAlignment": 0})
        else:
            tf.update({"boundsType": "OBS_BOUNDS_NONE", "scaleX": k, "scaleY": k})
        call(self.ws, "SetSceneItemTransform", {"sceneName": self.name, "sceneItemId": sid, "sceneItemTransform": tf})
        self.order.append(sid)

    @staticmethod
    def font(size):
        return {"face": "Segoe UI", "size": size, "style": "Bold", "flags": 1}

    def text(self, name, text, x, y, size=20, color=0xFFF0EDEB, box=False):
        extra = {"bk_color": 0xFF000000, "bk_opacity": 65} if box else {}
        self.put(self.item(name, "text_gdiplus_v3", {"read_from_file": False, "text": text,
                                                     "font": self.font(size), "color": color, **extra}), x, y)

    def color(self, name, argb, x, y, w, h):
        self.put(self.item(name, "color_source_v3", {"color": argb, "width": w, "height": h}), x, y)

    def window(self, name, w, x, y, pw, ph, crop=(0, 0, 0, 0)):
        blank = {"window": "", "priority": 2, "method": 2, "cursor": False, "client_area": True}
        self.put(self.item(name, "window_capture", window_source(w) if w else blank), x, y, pw, ph, crop)

    def browser(self, name, url, x, y, pw, ph):
        """A web page rendered by OBS itself at the panel's size, e.g. a FIXS plot."""
        k = self.k
        settings = {"url": url, "width": round(pw * k), "height": round(ph * k), "reroute_audio": False}
        self.put(self.item(name, "browser_source", settings), x, y, pw, ph)

    def finish(self):
        """Drop anything else in the scene, which would cover a panel, and stack the rest in put order."""
        for name, sid in self.items.items():
            if sid not in self.order:
                call(self.ws, "RemoveSceneItem", {"sceneName": self.name, "sceneItemId": sid})
        for i, sid in enumerate(self.order):
            call(self.ws, "SetSceneItemIndex", {"sceneName": self.name, "sceneItemId": sid, "sceneItemIndex": i})


def layout(scene, title, panels, wins, urls=None):
    """The common frame: background, title bar, one window capture (or page, by urls) per panel, labels."""
    scene.color("bg", 0xFF18140F, 0, 0, *CANVAS)
    for p in panels:
        if (urls or {}).get(p.key):
            scene.browser(p.key, urls[p.key], p.x, p.y, p.w, p.h)
            continue
        crop = (0, 0, 0, 0)
        if p.key == "sumo" and wins.get(p.key):
            # A sumo-gui opened mid-recording keeps its own shape: fill the panel, centred on the ego.
            crop = fill_crop(wins[p.key][0], sumo_crop(wins[p.key][0]), p.w, p.h)
        scene.window(p.key, wins.get(p.key), p.x, p.y, p.w, p.h, crop)
    scene.text("title", title, 24, 10, size=26)
    for p in panels:
        if p.label:
            scene.text("label " + p.key, f" {p.label} ", p.x + 16, p.y + (p.h - 52 if p.label_bottom else 16), box=True)


CLOCK_CSV = "obs_recording_clock.csv"


def retime_run(out_dir, src=None, start=None):
    """retime() out_dir's recording by its clock log, optionally from sim time `start`; the output path."""
    with open(os.path.join(out_dir, CLOCK_CSV), encoding="utf-8") as f:
        pairs = [tuple(float(x) for x in line.split(",")) for line in f if line.strip()]
    if start is not None:
        pairs = [p for p in pairs if p[1] >= start]
    dst = os.path.join(out_dir, "obs_recording_simtime.mp4")
    retime(src or os.path.join(out_dir, "obs_recording.mp4"), dst, pairs)
    return dst


def retime(src, dst, pairs, fps=30):
    """Write video `src` again at sim speed: `pairs` are (seconds into src, sim time) samples."""
    import cv2
    import numpy as np
    wall, sim = (np.array(x, float) for x in zip(*pairs))
    keep = np.r_[True, np.diff(sim) > 0]         # sim time must rise: drop paused samples
    wall, sim = wall[keep], sim[keep]
    cap = cv2.VideoCapture(src)
    src_fps, n = cap.get(cv2.CAP_PROP_FPS) or 60.0, int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    size = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out = cv2.VideoWriter(dst, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    pos, frame = -1, None
    for j in range(int((sim[-1] - sim[0]) * fps)):
        want = min(n - 1, int(round(np.interp(sim[0] + j / fps, sim, wall) * src_fps)))
        while pos < want:
            pos += 1
            if pos < want:
                ok = cap.grab()                    # skipped: not decoded
            else:
                ok, frame = cap.read()
            if not ok:
                break
        if frame is not None:
            out.write(frame)
    out.release()
    cap.release()


# ---- the recorder process ------------------------------------------------------------

def record(scene_name, prefix, title, panels, finders, pid, out_dir, ready, over, build=None, tick=None,
           urls=None, clock=None, tail_s=0.0, family=None, follow=False):
    """Record this run until `over()` or `pid` exits; with `tail_s`, up to that long past its exit.

    finders: {panel key: predicate(title, class, exe)}, FINDERS plus the app's own.
    ready(): True once recording should start; over(): True once it should stop.
    build(scene, wins): the app's items on top of layout(); tick(): called every 0.2 s.
    urls(): {panel key: page} shown as browser sources instead of windows.
    clock(): the sim time or None; with it, out_dir/obs_recording_clock.csv logs (seconds into the
    video, sim time), from which this file's command line writes the video at sim speed.
    family(): the pids whose windows may be captured (default: run_family(pid)).
    follow: while recording, re-capture a panel whose window closed or was replaced (one
    recording across several runs).
    """
    u.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))  # real pixels: window sizes match OBS's canvas
    tag = f"[{os.environ.get('FIXS_RUN_ID') or 'rec %d' % pid}]"
    sumo_panel = next((p for p in panels if p.key == "sumo"), None)
    ws, prev_scene, prev_dir, recording, busy, wins, sumo_rect = None, None, None, False, False, {}, None
    spare, lock = None, None
    started, pairs, clock_log = 0.0, [], None   # wall time recording started; (seconds into the video, sim time)
    try:
        ws, spare, lock = open_obs()
        prev_scene = call(ws, "GetCurrentProgramScene").get("currentProgramSceneName")
        next_scan = 0.0
        gone = None                                 # when `pid` exited, for tail_s
        captured = None                             # the windows the scene shows, for `follow`

        def compose():
            scene = Scene(ws, scene_name, prefix)
            layout(scene, title, panels, wins, urls() if urls else {})
            if build:
                build(scene, wins)
            scene.finish()
            return tuple((n, wins[n][0] if n in wins else None) for n in finders)

        while True:
            if not alive(pid):
                gone = gone or time.time()
                if not recording or time.time() - gone >= tail_s:
                    break
            if tick:
                tick()
            start = not recording and not busy and ready()
            # A fresh scan before the layout, so a window that opened since the last one is in it.
            if start or time.time() >= next_scan:
                # Only this run's windows, so another co-sim's CARLA or sumo-gui is never captured.
                pids = family() if family else run_family(pid)
                for n, pred in finders.items():
                    w = wins.get(n)
                    if not (w and u.IsWindow(w[0])):
                        w = find(pred, pids)
                    if w:
                        if u.IsIconic(w[0]):
                            u.ShowWindow(w[0], 4)  # SW_SHOWNOACTIVATE: OBS can't capture a minimised window
                        wins[n] = tag_title(w, tag)
                    else:
                        wins.pop(n, None)
                if follow and recording and captured != tuple((n, wins[n][0] if n in wins else None)
                                                              for n in finders):
                    captured = compose()
                if sumo_panel and wins.get("sumo") and not recording:
                    # Put back before it closes, since sumo-gui saves its size then.
                    if sumo_rect is None:
                        sumo_rect = window_rect(wins["sumo"][0])
                    # Re-checked each scan: sumo-gui resets itself to its saved size after it opens.
                    size_sumo(wins["sumo"][0], sumo_panel.w, sumo_panel.h)
                next_scan = time.time() + 2
            if recording and clock:
                sim = clock()
                if sim is not None:
                    pairs.append((time.time() - started, sim))
                    # Kept on disk as it goes, so a killed recorder's video can still be retimed.
                    clock_log = clock_log or open(os.path.join(out_dir, CLOCK_CSV), "w", encoding="utf-8")
                    clock_log.write("%.3f,%.3f\n" % pairs[-1])
                    clock_log.flush()
            if recording and over():
                break
            if start:
                if call(ws, "GetRecordStatus")["outputActive"]:
                    busy = True
                    log("*** THIS RUN IS NOT RECORDED: its OBS was started recording by hand ***")
                    continue
                pages = urls() if urls else {}
                for p in panels:
                    if not wins.get(p.key) and not pages.get(p.key) and not follow:
                        log(f"no {p.key} window started by this run; that panel stays empty")
                captured = compose()
                call(ws, "SetCurrentProgramScene", {"sceneName": scene_name})
                # Written straight into out_dir, so a killed run still keeps its video there.
                prev_dir = call(ws, "GetRecordDirectory")["recordDirectory"]
                if not spare:
                    call(ws, "SetPersistentData", {**OWNER, "slotValue": {"pid": os.getpid(), "dir": prev_dir}})
                call(ws, "SetRecordDirectory", {"recordDirectory": out_dir})
                _close_handler = stop_on_console_close(ws, prev_dir)  # noqa: F841 (kept alive)
                call(ws, "StartRecord")
                started = time.time()
                recording = True
                log("OBS recording")
            time.sleep(0.2)
    except Exception as e:
        log(f"*** {'RECORDING STOPPED' if recording else 'THIS RUN IS NOT RECORDED'}: {type(e).__name__}: {e} ***")
    finally:
        # Quick steps only, most important first: the run's teardown can end this process.
        out = None
        if recording:
            try:
                out = call(ws, "StopRecord")["outputPath"]
            except Exception as e:
                log(f"OBS StopRecord failed: {e}")
        if sumo_rect and wins.get("sumo") and u.IsWindow(wins["sumo"][0]):
            place(wins["sumo"][0], *sumo_rect)
        if prev_dir and not spare:
            try:
                restore_dir(ws, prev_dir)
            except Exception as e:
                log(f"OBS recording folder left at {out_dir}: {e}")
        if out:
            dst = os.path.join(out_dir, "obs_recording" + os.path.splitext(out)[1])
            for _ in range(20):  # OBS can hold the file a moment after StopRecord returns
                try:
                    os.replace(out, dst)
                    out = dst
                    break
                except OSError:
                    time.sleep(0.25)
            log("video:", out)
        if spare:
            if ws:
                ws.close()  # an open client holds up OBS's exit
            spare.close()
        elif ws and prev_scene:
            try:
                call(ws, "SetCurrentProgramScene", {"sceneName": prev_scene})
            except Exception:
                pass
        if clock_log:
            clock_log.close()
        if out and len(pairs) > 1:
            # Not done here: the run's job object ends this process with the run.
            log("at sim speed: python %s %s" % (os.path.abspath(__file__), out_dir))


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Write a run's OBS recording again at sim speed, by its clock log.")
    ap.add_argument("out_dir")
    ap.add_argument("--video", default=None, help="the recording, if not out_dir/obs_recording.mp4")
    ap.add_argument("--from", dest="start", type=float, default=None, help="start at this sim time [s]")
    a = ap.parse_args()
    print(retime_run(a.out_dir, a.video, a.start))
