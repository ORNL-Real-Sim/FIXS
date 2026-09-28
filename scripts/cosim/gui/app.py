"""
gui/app.py - the FIXS window (run_cosim --gui, #78). Qt Widgets via PySide6.

Three tabs, one per question a user brings to FIXS:

  Run       what to run and running it: a saved setup or a new one, Run / Stop,
            and the live log of the whole stack, filterable by component.
  Scenario  the scenario yaml a run reads. Today it opens the file; the editor
            designed in #121 / #142 takes this tab over.
  Machine   whether this computer can run a co-sim: what is installed, the
            Doctor's checks, CARLA setup, and cleaning up after a crashed run.

Nothing here decides anything the engine decides. Every action is core.py
turning the form into run_cosim flags, and the command is shown before it runs,
so what the window does can always be done from a terminal - and a user who
outgrows the window already knows the command.
"""
import os
import sys
import threading
import time

from . import prefer_system_icu

prefer_system_icu()                     # before the first PySide6 import - see there

from PySide6.QtCore import QObject, Qt, QTimer, QUrl, Signal  # noqa: E402
from PySide6.QtGui import (QColor,  # noqa: E402
                           QDesktopServices, QFont, QFontDatabase,
                           QPalette, QTextCharFormat, QTextCursor)
from PySide6.QtWidgets import (QApplication,  # noqa: E402
                               QButtonGroup, QCheckBox, QComboBox,
                               QFileDialog, QFormLayout, QGridLayout, QGroupBox,
                               QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                               QListWidget, QListWidgetItem, QMainWindow,
                               QMessageBox, QPlainTextEdit, QPushButton,
                               QRadioButton, QSplitter, QTabWidget, QTreeWidget,
                               QTreeWidgetItem, QVBoxLayout, QWidget)

from . import core  # noqa: E402

NEW_RUN = "__new__"
BROWSE = "__browse__"
# How long a stop request is given to unwind the stack before Stop turns into
# Force stop. The native stack notices within a second and its teardown waits up
# to 5 s per process; CARLA takes longest to let go.
STOP_GRACE_S = 20


# --------------------------------------------------------------------------- #
# Threads -> the UI thread
# --------------------------------------------------------------------------- #
class _Async(QObject):
    """Run a function off the UI thread and hand its result (or exception) to a
    callback on the UI thread. The signal crosses threads as a queued call."""
    _done = Signal(object, object)

    def __init__(self):
        super().__init__()
        self._done.connect(lambda cb, res: cb(res))

    def run(self, fn, cb):
        def work():
            try:
                res = fn()
            except Exception as exc:          # delivered, not raised
                res = exc
            self._done.emit(cb, res)
        threading.Thread(target=work, daemon=True).start()


class _Pipe(QObject):
    """A Runner's callbacks, re-emitted on the UI thread."""
    line = Signal(str)
    exited = Signal(int)


def _mono():
    f = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
    f.setStyleHint(QFont.StyleHint.Monospace)
    return f


def _dark(widget):
    return widget.palette().color(QPalette.ColorRole.Window).lightness() < 128


def _open_path(path):
    if path:
        QDesktopServices.openUrl(QUrl.fromLocalFile(path))


# --------------------------------------------------------------------------- #
# Log view
# --------------------------------------------------------------------------- #
class LogView(QWidget):
    """The stack's output, coloured by who printed it and filterable to one
    component or to problems only. Lines arrive in bursts, so they are buffered
    and drawn a few times a second rather than one repaint per line."""

    FILTERS = ["Everything", *core.STREAMS, "Problems only"]

    def __init__(self, parent=None, max_lines=50000):
        super().__init__(parent)
        self.entries = []                # (stream, level, text)
        self.pending = []
        self.last_tag = None
        self.max_lines = max_lines

        self.view = QPlainTextEdit(readOnly=True)
        self.view.setFont(_mono())
        self.view.setMaximumBlockCount(max_lines)
        self.view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.filter = QComboBox()
        self.filter.addItems(self.FILTERS)
        self.filter.currentIndexChanged.connect(self._redraw)
        self.follow = QCheckBox("Follow", checked=True)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.clear)
        save = QPushButton("Save...")
        save.clicked.connect(self._save)

        bar = QHBoxLayout()
        bar.addWidget(QLabel("Show:"))
        bar.addWidget(self.filter)
        bar.addStretch(1)
        bar.addWidget(self.follow)
        bar.addWidget(clear)
        bar.addWidget(save)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addLayout(bar)
        lay.addWidget(self.view)

        self._formats = self._make_formats()
        self._timer = QTimer(self, interval=100)
        self._timer.timeout.connect(self._flush)
        self._timer.start()

    def _make_formats(self):
        dark = _dark(self)
        colors = ({"FIXS": "#82aaff", "SUMO": "#8bd17c", "TrafficLayer": "#c792ea",
                   "Bridge": "#5fd7d7", "App": "#ffcb6b", "problem": "#ff6b6b",
                   "warning": "#ffb74d", "note": "#9aa5b1"} if dark else
                  {"FIXS": "#1f4e9e", "SUMO": "#2e7d32", "TrafficLayer": "#6a1b9a",
                   "Bridge": "#00838f", "App": "#8d4e00", "problem": "#c62828",
                   "warning": "#a15c00", "note": "#5f6b7a"})
        fmts = {}
        for key, color in colors.items():
            f = QTextCharFormat()
            f.setForeground(QColor(color))
            if key == "problem":
                f.setFontWeight(QFont.Weight.Bold)
            fmts[key] = f
        fmts[None] = QTextCharFormat()
        return fmts

    def add(self, text, note=False):
        tag = core.tag_of(text, self.last_tag)
        self.last_tag = tag
        entry = ("note" if note else core.stream_of(tag),
                 None if note else core.level_of(text), text)
        self.entries.append(entry)
        if len(self.entries) > self.max_lines:
            del self.entries[: len(self.entries) - self.max_lines]
        self.pending.append(entry)

    def note(self, text):
        """A line from the window itself (what it ran, how it ended)."""
        self.add(text, note=True)

    def clear(self):
        self.entries, self.pending, self.last_tag = [], [], None
        self.view.clear()

    def text(self):
        return "\n".join(t for _s, _l, t in self.entries)

    def _shown(self, entry):
        which = self.filter.currentText()
        stream, level, _t = entry
        if which == "Everything" or stream == "note":
            return True
        if which == "Problems only":
            return level == "problem"
        return stream == which

    def _flush(self):
        if not self.pending:
            return
        batch, self.pending = self.pending, []
        self._write([e for e in batch if self._shown(e)])

    def _redraw(self):
        self.pending = []
        self.view.clear()
        self._write([e for e in self.entries if self._shown(e)])

    def _write(self, entries):
        if not entries:
            return
        bar = self.view.verticalScrollBar()
        at_end = bar.value() >= bar.maximum() - 2
        cur = QTextCursor(self.view.document())
        cur.movePosition(QTextCursor.MoveOperation.End)
        cur.beginEditBlock()
        for stream, level, text in entries:
            if not self.view.document().isEmpty():
                cur.insertBlock()
            cur.insertText(text, self._formats.get(level or stream,
                                                   self._formats[None]))
        cur.endEditBlock()
        if self.follow.isChecked() or at_end:
            bar.setValue(bar.maximum())

    def _save(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save log", "fixs_run.log",
                                              "Log files (*.log *.txt)")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.text() + "\n")


# --------------------------------------------------------------------------- #
# Run tab
# --------------------------------------------------------------------------- #
class RunTab(QWidget):
    configChanged = Signal(str)          # the scenario yaml the form points at

    def __init__(self, win):
        super().__init__()
        self.win = win
        self.runner = core.Runner(engine=win.engine)
        self.pipe = _Pipe()
        self.pipe.line.connect(self._on_line)
        self.pipe.exited.connect(self._on_exit)
        self.apps, self.maps, self.setups, self.last = [], [], {}, None
        self.baseline = None             # the saved setup's values, when one is open
        self._loading = False
        self._build()
        self._tick = QTimer(self, interval=1000)
        self._tick.timeout.connect(self._update_status)

    # ------------------------------------------------------------------ layout
    def _build(self):
        # Saved setups
        self.setup_list = QListWidget()
        self.setup_list.currentItemChanged.connect(self._on_setup_picked)
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self.reload)
        self.setup_list.setMinimumWidth(240)
        left = QGroupBox("Saved setups")
        ll = QVBoxLayout(left)
        ll.addWidget(self.setup_list)
        ll.addWidget(refresh)

        # What to run
        self.setup_label = QLabel()
        self.app_box = QComboBox()
        self.app_box.currentIndexChanged.connect(self._on_app_changed)
        self.map_box = QComboBox(editable=True)
        self.map_box.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.map_box.currentTextChanged.connect(self._on_map_changed)
        self.map_info = QLabel()
        self.map_info.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.config_box = QComboBox()
        self.config_box.activated.connect(self._on_config_activated)
        self.config_box.currentIndexChanged.connect(self._changed)
        self.sumo_gui = QCheckBox("Show the SUMO window")
        self.sumo_only = QCheckBox("Traffic only (no CARLA)")
        self.fast = QCheckBox("As fast as possible (not real time)")
        self.log_file = QCheckBox("Also save the log under RealSim_tmp/")
        self.engine_box = QComboBox()
        for text, data in (("as the scenario says", None),
                           ("py - Python VirEnvCore", "py"),
                           ("cpp - VirCarlaEnv.exe", "cpp")):
            self.engine_box.addItem(text, data)
        self.app_args = QLineEdit()
        for w in (self.sumo_gui, self.sumo_only, self.fast, self.log_file):
            w.toggled.connect(self._changed)
        self.engine_box.currentIndexChanged.connect(self._changed)
        self.app_args.textChanged.connect(self._changed)

        opts = QGridLayout()
        opts.addWidget(self.sumo_gui, 0, 0)
        opts.addWidget(self.sumo_only, 0, 1)
        opts.addWidget(self.fast, 1, 0)
        opts.addWidget(self.log_file, 1, 1)

        mapcol = QVBoxLayout()
        mapcol.setSpacing(2)
        mapcol.addWidget(self.map_box)
        mapcol.addWidget(self.map_info)

        form = QFormLayout()
        form.addRow("Setup:", self.setup_label)
        form.addRow("Application:", self.app_box)
        form.addRow("Map:", mapcol)
        form.addRow("Scenario:", self.config_box)
        form.addRow("Bridge engine:", self.engine_box)
        form.addRow("App arguments:", self.app_args)
        form.addRow("Options:", opts)

        self.command = QLineEdit(readOnly=True)
        self.command.setFont(_mono())
        copy = QPushButton("Copy")
        copy.clicked.connect(lambda: QApplication.clipboard().setText(self.command.text()))
        cmdrow = QHBoxLayout()
        cmdrow.addWidget(self.command, 1)
        cmdrow.addWidget(copy)

        self.run_btn = QPushButton("Run")
        self.run_btn.setDefault(True)
        self.run_btn.setMinimumWidth(110)
        self.run_btn.clicked.connect(self.start)
        self.stop_btn = QPushButton("Stop", enabled=False)
        self.stop_btn.setMinimumWidth(110)
        self.stop_btn.clicked.connect(self.stop)
        logs = QPushButton("Open RealSim_tmp")
        logs.clicked.connect(lambda: _open_path(
            os.path.join(core.engine_cwd(self.win.engine), "RealSim_tmp")))
        self.status = QLabel("Ready.")
        btns = QHBoxLayout()
        btns.addWidget(self.run_btn)
        btns.addWidget(self.stop_btn)
        btns.addWidget(self.status, 1)
        btns.addWidget(logs)

        right = QGroupBox("What to run")
        rl = QVBoxLayout(right)
        rl.addLayout(form)
        rl.addWidget(QLabel("Command:"))
        rl.addLayout(cmdrow)
        rl.addLayout(btns)

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(left)
        top.addWidget(right)
        top.setStretchFactor(0, 1)
        top.setStretchFactor(1, 3)

        self.log = LogView()
        split = QSplitter(Qt.Orientation.Vertical)
        split.addWidget(top)
        split.addWidget(self.log)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        lay = QVBoxLayout(self)
        lay.addWidget(split)

    # ------------------------------------------------------------------ data
    def reload(self, select=None):
        """Re-read setups, apps and maps from the engine (three --list queries)."""
        self.status.setText("Loading setups, applications and maps ...")
        engine = self.win.engine
        q = self.win.backend

        def fetch():
            return (q.list_setups(engine=engine), q.list_apps(engine=engine),
                    q.list_maps(engine=engine))
        self.win.run_async(fetch, lambda res: self._loaded(res, select))

    def _loaded(self, res, select):
        if isinstance(res, Exception):
            self.status.setText("Could not read setups - see the log.")
            self.log.note(f"[gui] listing failed: {res}")
            return
        setups, apps, maps = res
        self.apps = apps.get("apps", [])
        self.maps = maps.get("maps", [])
        self.setups = {s["name"]: s for s in setups.get("setups", [])}
        self.last = setups.get("last")
        self._loading = True
        self.app_box.clear()
        self.app_box.addItem("(no application)", "")
        for a in self.apps:
            self.app_box.addItem(f"{a['title']}  [{a['id']}]", a["id"])
        self.setup_list.clear()
        item = QListWidgetItem("+ New run")
        item.setData(Qt.ItemDataRole.UserRole, NEW_RUN)
        self.setup_list.addItem(item)
        want = select or self.last
        chosen = item
        for name, s in self.setups.items():
            label = name + ("   (last run)" if name == self.last else "")
            it = QListWidgetItem(label)
            it.setData(Qt.ItemDataRole.UserRole, name)
            it.setToolTip(s.get("summary") or "")
            self.setup_list.addItem(it)
            if name == want:
                chosen = it
        self.setup_list.setCurrentItem(chosen)
        self._loading = False
        self._on_setup_picked(chosen)
        if not self.runner.running:
            self.status.setText("Ready.")

    def _app(self, app_id=None):
        app_id = self.app_box.currentData() if app_id is None else app_id
        return next((a for a in self.apps if a["id"] == app_id), None)

    # ------------------------------------------------------------------ form
    def _set_combo(self, box, data):
        i = box.findData(data)
        if i >= 0:
            box.setCurrentIndex(i)
        return i >= 0

    def _fill_maps(self, app):
        current = self.map_box.currentText()
        self.map_box.blockSignals(True)
        self.map_box.clear()
        declared = (app or {}).get("maps") or []
        for m in declared:
            self.map_box.addItem(m)
        known = [m["name"] for m in self.maps if m["name"] not in declared]
        if declared and known:
            self.map_box.insertSeparator(self.map_box.count())
        for name in known:
            self.map_box.addItem(name)
        self.map_box.setEditText(current)
        self.map_box.blockSignals(False)

    def _fill_configs(self, app, keep=None, default_first=True):
        self.config_box.blockSignals(True)
        self.config_box.clear()
        self.config_box.addItem("(automatic - generated for the map)", None)
        for c in (app or {}).get("configs") or []:
            self.config_box.addItem(c["title"], c["staged"])
            self.config_box.setItemData(self.config_box.count() - 1,
                                        f"{c['source']}\nstaged at {c['staged']}",
                                        Qt.ItemDataRole.ToolTipRole)
        if keep and self.config_box.findData(keep) < 0:
            self.config_box.addItem(os.path.basename(keep), keep)
        self.config_box.addItem("Other yaml file...", BROWSE)
        # A saved setup keeps its yaml; a new run starts on the app's first declared
        # scenario - the one its manifest lists first, usually marked "(default)" -
        # and only falls back to a generated one for an app that declares none.
        if keep:
            index = self.config_box.findData(keep)
        else:
            index = 1 if default_first and (app or {}).get("configs") else 0
        self.config_box.setCurrentIndex(max(0, index))
        self.config_box.blockSignals(False)

    def _on_setup_picked(self, item, _prev=None):
        if self._loading or item is None:
            return
        name = item.data(Qt.ItemDataRole.UserRole)
        self._loading = True
        if name == NEW_RUN:
            self.baseline = None
            self.setup_label.setText("new - it is saved under a new name when it runs")
            app = self.apps[0] if self.apps else None
            self._set_combo(self.app_box, app["id"] if app else "")
            self._fill_maps(app)
            self.map_box.setEditText(((app or {}).get("maps") or [""])[0])
            self._fill_configs(app)
            self.sumo_gui.setChecked(True)
        else:
            s = self.setups[name]
            self.baseline = {"name": name, "app": s.get("app") or "",
                             "map": s.get("map") or "", "config": s.get("config"),
                             "sumo_gui": bool(s.get("sumo_gui", True))}
            self.setup_label.setText(f"{name}  -  changes you run are saved into it")
            if not self._set_combo(self.app_box, self.baseline["app"]):
                self.app_box.setCurrentIndex(0)
            app = self._app()
            self._fill_maps(app)
            self.map_box.setEditText(self.baseline["map"])
            self._fill_configs(app, keep=self.baseline["config"], default_first=False)
            self.sumo_gui.setChecked(self.baseline["sumo_gui"])
        for w in (self.sumo_only, self.fast, self.log_file):
            w.setChecked(False)
        self.engine_box.setCurrentIndex(0)
        self.app_args.clear()
        self._loading = False
        self._on_map_changed(self.map_box.currentText())
        self._sync_app_args()
        self._changed()

    def _on_app_changed(self):
        if self._loading:
            return
        app = self._app()
        self._fill_maps(app)
        self.map_box.setEditText(((app or {}).get("maps") or [""])[0])
        self._fill_configs(app)
        self._sync_app_args()
        self._changed()

    def _sync_app_args(self):
        app = self._app()
        launch = (app or {}).get("launch")
        self.app_args.setEnabled(bool(launch))
        self.app_args.setPlaceholderText(
            f"passed to the app as COSIM_APP_ARGS  (it starts: {launch})" if launch
            else "this application starts no program of its own")

    def _on_map_changed(self, text):
        m = next((m for m in self.maps if m["name"] == text.strip()), None)
        if not text.strip():
            info = "type a map name, or pick one"
        elif m is None:
            info = "not on this machine - the run looks it up in the map library"
        else:
            bits = [w for w, on in (("cooked into this CARLA", m.get("cooked")),
                                    ("bundle downloaded", m.get("cached")),
                                    ("in the map library", m.get("library"))) if on]
            info = (m.get("title") + " - " if m.get("title") else "") + ", ".join(bits)
        self.map_info.setText(f"<small>{info}</small>")
        self._changed()

    def _on_config_activated(self, index):
        if self.config_box.itemData(index) != BROWSE:
            return
        path, _ = QFileDialog.getOpenFileName(self, "Scenario yaml", "",
                                              "Scenario yaml (*.yaml *.yml)")
        if not path:
            self.config_box.setCurrentIndex(0)
            return
        path = os.path.normpath(path)
        i = self.config_box.findData(path)
        if i < 0:
            self.config_box.insertItem(self.config_box.count() - 1,
                                       os.path.basename(path), path)
            i = self.config_box.findData(path)
        self.config_box.setCurrentIndex(i)

    def _config(self):
        data = self.config_box.currentData()
        return None if data == BROWSE else data

    def request(self):
        """The form as a RunRequest. With a saved setup open, only what differs
        from it is passed - so an untouched setup runs as `--profile NAME` and
        nothing else, exactly as it would from a terminal."""
        app = self.app_box.currentData() or ""
        map_ = self.map_box.currentText().strip() or None
        config = self._config()
        sumo_gui = self.sumo_gui.isChecked()
        extra = dict(sumo_only=self.sumo_only.isChecked(),
                     engine=self.engine_box.currentData(),
                     fast=self.fast.isChecked(),
                     app_args=(self.app_args.text() if self.app_args.isEnabled()
                               and self.app_args.text().strip() else None),
                     log=self.log_file.isChecked())
        b = self.baseline
        if b is None:
            # A new setup shows SUMO's window unless told otherwise, so only the
            # "off" answer needs saying.
            return core.RunRequest(app=app, map=map_, config=config,
                                   sumo_gui=None if sumo_gui else False, **extra)
        return core.RunRequest(
            setup=b["name"],
            app=app if app != b["app"] else None,
            map=map_ if (map_ or "") != b["map"] else None,
            config=config if config != b["config"] and config else None,
            sumo_gui=sumo_gui if sumo_gui != b["sumo_gui"] else None,
            **extra)

    def _changed(self, *_):
        if self._loading:
            return
        req = self.request()
        self.command.setText(core.format_command(
            [os.path.basename(self.runner.python), self.win.engine, *req.args()]))
        self.command.setCursorPosition(0)
        self.configChanged.emit(self._config() or "")

    # ------------------------------------------------------------------ running
    def start(self):
        if self.runner.running:
            return
        req = self.request()
        if req.setup is None and not req.map:
            QMessageBox.warning(self, "FIXS", "Pick or type a map to run.")
            return
        try:
            argv = self.runner.start(req.args(), self.pipe.line.emit,
                                     self.pipe.exited.emit)
        except OSError as exc:
            QMessageBox.critical(self, "FIXS", f"Could not start the engine:\n{exc}")
            return
        self.log.note(f"=== {time.strftime('%H:%M:%S')}  {core.format_command(argv)}")
        self._set_running(True)

    def stop(self):
        r = self.runner
        if not r.running:
            return
        if r.stop_requested_at is not None and \
                time.monotonic() - r.stop_requested_at >= STOP_GRACE_S:
            self.log.note("[gui] force-stopping the run and everything it started. "
                          "If anything is left over, use Machine > Clean up.")
            r.kill()
            return
        if r.request_stop():
            self.log.note("[gui] stop requested - the run is shutting its stack down.")
            self.stop_btn.setEnabled(False)
            self.stop_btn.setText("Stopping...")

    def _on_line(self, text):
        self.log.add(text)

    def _on_exit(self, rc):
        stopped = self.runner.stop_requested_at is not None or rc == 130
        verdict = ("stopped" if stopped else "finished" if rc == 0
                   else f"failed (exit {rc})")
        self.log.note(f"=== {time.strftime('%H:%M:%S')}  run {verdict}")
        self._set_running(False)
        self.status.setText(f"Run {verdict}.")
        # A run saves its setup (a new one under a new name), so the list moved.
        self.reload(select=None)

    def _set_running(self, on):
        for w in (self.run_btn, self.setup_list, self.app_box, self.map_box,
                  self.config_box, self.engine_box, self.app_args, self.sumo_gui,
                  self.sumo_only, self.fast, self.log_file):
            w.setEnabled(not on)
        if not on:
            self._sync_app_args()
        self.stop_btn.setEnabled(on)
        self.stop_btn.setText("Stop")
        self.win.set_busy(on)
        if on:
            self._tick.start()
            self._update_status()
        else:
            self._tick.stop()

    def _update_status(self):
        r = self.runner
        if not r.running or r.started_at is None:
            return
        secs = int(time.monotonic() - r.started_at)
        clock = f"{secs // 3600:d}:{secs // 60 % 60:02d}:{secs % 60:02d}"
        if r.stop_requested_at is None:
            self.status.setText(f"Running  {clock}")
            return
        waited = time.monotonic() - r.stop_requested_at
        if waited >= STOP_GRACE_S:
            self.stop_btn.setText("Force stop")
            self.stop_btn.setEnabled(True)
            self.status.setText(f"Still stopping after {int(waited)} s - "
                                f"Force stop ends it now.")
        else:
            self.status.setText(f"Stopping ...  {int(waited)} s")


# --------------------------------------------------------------------------- #
# Scenario tab
# --------------------------------------------------------------------------- #
class ScenarioTab(QWidget):
    """The scenario yaml the Run tab points at. The editor from #121 / #142 moves
    in here; until then this opens the file in the system's editor."""

    def __init__(self, win):
        super().__init__()
        self.path = ""
        intro = QLabel(
            "<b>Scenario editor - coming next.</b><br>"
            "The form-based editor for the scenario yaml (SimulationSetup, "
            "ApplicationSetup, XilSetup, CarlaSetup ...) will live in this tab "
            "(FIXS issue #142). Until then, open the yaml the Run tab points at in "
            "your editor. A run re-reads it, so saved edits apply to the next run.")
        intro.setWordWrap(True)
        self.label = QLabel()
        self.label.setFont(_mono())
        self.label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.label.setWordWrap(True)
        self.open_btn = QPushButton("Open in editor")
        self.open_btn.clicked.connect(lambda: _open_path(self.path))
        self.folder_btn = QPushButton("Show folder")
        self.folder_btn.clicked.connect(lambda: _open_path(os.path.dirname(self.path)))
        row = QHBoxLayout()
        row.addWidget(self.open_btn)
        row.addWidget(self.folder_btn)
        row.addStretch(1)
        lay = QVBoxLayout(self)
        lay.addWidget(intro)
        lay.addSpacing(12)
        lay.addWidget(QLabel("Scenario yaml of the current Run setup:"))
        lay.addWidget(self.label)
        lay.addLayout(row)
        lay.addStretch(1)
        self.show_path("")

    def show_path(self, path):
        self.path = path
        exists = bool(path) and os.path.isfile(path)
        if not path:
            self.label.setText("(automatic - the run generates one for the map)")
        elif exists:
            self.label.setText(path)
        else:
            self.label.setText(f"{path}\n(not created yet - the first run stages it)")
        self.open_btn.setEnabled(exists)
        self.folder_btn.setEnabled(bool(path) and os.path.isdir(os.path.dirname(path)))


# --------------------------------------------------------------------------- #
# Machine tab
# --------------------------------------------------------------------------- #
class MachineTab(QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.runner = core.Runner(engine=win.engine)
        self.pipe = _Pipe()
        self.pipe.line.connect(lambda t: self.log.add(t))
        self.pipe.exited.connect(self._task_done)
        self.task_name = None
        self._build()

    def _build(self):
        # What is installed
        self.info = QFormLayout()
        info_box = QGroupBox("This machine")
        info_box.setLayout(self.info)

        # Doctor
        self.doctor_btn = QPushButton("Run checks")
        self.doctor_btn.clicked.connect(self.run_doctor)
        self.verdict = QLabel()
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Check", "Status", "Detail"])
        self.tree.header().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.tree.header().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        drow = QHBoxLayout()
        drow.addWidget(self.doctor_btn)
        drow.addWidget(self.verdict, 1)
        doc_box = QGroupBox("Doctor - can this machine run a co-sim?")
        dl = QVBoxLayout(doc_box)
        dl.addLayout(drow)
        dl.addWidget(self.tree)

        # CARLA setup
        self.mode_group = QButtonGroup(self)
        modes = QHBoxLayout()
        for i, (mode, text) in enumerate((("source", "Source build"),
                                          ("packaged", "Packaged release"),
                                          ("client", "No CARLA on this machine"))):
            rb = QRadioButton(text)
            rb.setProperty("mode", mode)
            self.mode_group.addButton(rb, i)
            modes.addWidget(rb)
        modes.addStretch(1)
        self.mode_group.buttonToggled.connect(self._mode_toggled)
        self.carla_root = QLineEdit()
        self.ue4_root = QLineEdit()
        self.env_python = QLineEdit()
        self.env_python.setPlaceholderText("default: the FIXS conda env, if it exists")
        setup_form = QFormLayout()
        setup_form.addRow("CARLA:", modes)
        setup_form.addRow("CARLA folder:", self._with_browse(self.carla_root, dir_=True))
        setup_form.addRow("Unreal Engine root:", self._with_browse(self.ue4_root, dir_=True))
        setup_form.addRow("Python env:", self._with_browse(self.env_python, dir_=False))
        self.setup_btn = QPushButton("Save CARLA setup")
        self.setup_btn.clicked.connect(self.save_setup)
        setup_form.addRow("", self.setup_btn)
        setup_box = QGroupBox("CARLA setup (~/.fixs/carla.json)")
        setup_box.setLayout(setup_form)

        # Maintenance
        self.cleanup_btn = QPushButton("Clean up leftover processes")
        self.cleanup_btn.setToolTip("run_cosim --cleanup: ends SUMO, TrafficLayer, "
                                    "bridges and CARLA left behind by a crashed run")
        self.cleanup_btn.clicked.connect(lambda: self._task("cleanup", ["--cleanup"]))
        self.python_btn = QPushButton("Re-resolve the python env")
        self.python_btn.setToolTip("run_cosim --update-python: rebind the env, keep "
                                   "the CARLA paths")
        self.python_btn.clicked.connect(
            lambda: self._task("update-python", ["--update-python"]))
        maint = QHBoxLayout()
        maint.addWidget(self.cleanup_btn)
        maint.addWidget(self.python_btn)
        maint.addStretch(1)
        maint_box = QGroupBox("Maintenance")
        maint_box.setLayout(maint)

        self.log = LogView(max_lines=5000)

        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.addWidget(info_box)
        ll.addWidget(setup_box)
        ll.addWidget(maint_box)
        ll.addWidget(QLabel("Output:"))
        ll.addWidget(self.log, 1)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(left)
        split.addWidget(doc_box)
        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 1)
        lay = QVBoxLayout(self)
        lay.addWidget(split)

    def _with_browse(self, edit, dir_):
        btn = QPushButton("Browse...")

        def pick():
            if dir_:
                path = QFileDialog.getExistingDirectory(self, "Select folder", edit.text())
            else:
                path, _ = QFileDialog.getOpenFileName(self, "Select python", edit.text())
            if path:
                edit.setText(os.path.normpath(path))
        btn.clicked.connect(pick)
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(edit, 1)
        h.addWidget(btn)
        return row

    def _mode_toggled(self, *_):
        mode = self._mode()
        self.carla_root.setEnabled(mode in ("source", "packaged"))
        self.ue4_root.setEnabled(mode == "source")

    def _mode(self):
        b = self.mode_group.checkedButton()
        return b.property("mode") if b else None

    def show_fingerprint(self, fp):
        while self.info.rowCount():
            self.info.removeRow(0)
        if isinstance(fp, Exception):
            self.info.addRow("Error:", QLabel(str(fp)))
            return
        missing = [m for m, ok in (fp.get("modules") or {}).items() if not ok]
        rows = [("FIXS", f"{fp.get('fixs')}  ({fp.get('fixs_root')})"),
                ("Application repo", fp.get("app_root")),
                ("Python", fp.get("python")),
                ("Modules", "all present" if not missing
                 else "MISSING: " + ", ".join(missing)),
                ("SUMO", fp.get("sumo") or "not on PATH"),
                ("CARLA", f"{fp.get('carla_mode') or 'not configured'}"
                          + (f"  @ {fp['carla_root']}" if fp.get("carla_root") else "")),
                ("CARLA endpoint", f"{fp.get('carla_host')}:{fp.get('carla_port')}")]
        for k, v in rows:
            lab = QLabel(str(v))
            lab.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            lab.setWordWrap(True)
            self.info.addRow(k + ":", lab)
        # Pre-fill the setup form from what is configured.
        for b in self.mode_group.buttons():
            if b.property("mode") == fp.get("carla_mode"):
                b.setChecked(True)
        if fp.get("carla_root") and not self.carla_root.text():
            self.carla_root.setText(os.path.normpath(fp["carla_root"]))
        if fp.get("ue4_root") and not self.ue4_root.text():
            self.ue4_root.setText(os.path.normpath(fp["ue4_root"]))
        if fp.get("python") and not self.env_python.text():
            self.env_python.setText(fp["python"])
        self._mode_toggled()

    def run_doctor(self):
        self.doctor_btn.setEnabled(False)
        self.verdict.setText("Checking ...")
        self.tree.clear()
        engine = self.win.engine
        self.win.run_async(lambda: self.win.backend.doctor(engine=engine),
                           self._doctor_done)

    def _doctor_done(self, rep):
        self.doctor_btn.setEnabled(True)
        if isinstance(rep, Exception):
            self.verdict.setText("The checks could not run - see Output.")
            self.log.note(f"[gui] doctor failed: {rep}")
            return
        dark = _dark(self)
        colors = {"OK": "#8bd17c" if dark else "#2e7d32",
                  "WARN": "#ffb74d" if dark else "#a15c00",
                  "FAIL": "#ff6b6b" if dark else "#c62828"}
        sections = {}
        for row in rep.get("rows", []):
            sec = sections.get(row["section"])
            if sec is None:
                sec = sections[row["section"]] = QTreeWidgetItem([row["section"]])
                self.tree.addTopLevelItem(sec)
                sec.setExpanded(True)
            it = QTreeWidgetItem([row["label"], row["status"], row["detail"]])
            it.setForeground(1, QColor(colors.get(row["status"], "#888888")))
            it.setToolTip(2, row["detail"])
            sec.addChild(it)
        worst = rep.get("worst")
        self.verdict.setText({"OK": "Usable.", "WARN": "Usable, with warnings.",
                              "FAIL": "Problems found."}.get(worst, str(worst))
                             + f"   ({rep.get('host')}, role: {rep.get('role')})")

    def save_setup(self):
        mode = self._mode()
        if not mode:
            QMessageBox.warning(self, "FIXS", "Choose which CARLA this machine has.")
            return
        args = ["--setup", "--carla-mode", mode]
        if mode in ("source", "packaged"):
            if not self.carla_root.text().strip():
                QMessageBox.warning(self, "FIXS", "Select the CARLA folder.")
                return
            args += ["--carla-root", self.carla_root.text().strip()]
        if mode == "source":
            if not self.ue4_root.text().strip():
                QMessageBox.warning(self, "FIXS", "Select the Unreal Engine root.")
                return
            args += ["--ue4-root", self.ue4_root.text().strip()]
        if self.env_python.text().strip():
            args += ["--env-python", self.env_python.text().strip()]
        self._task("setup", args)

    def _task(self, name, args):
        if self.runner.running:
            return
        if self.win.busy and name == "cleanup":
            QMessageBox.information(self, "FIXS", "A run is in progress; Clean up "
                                    "would end it. Stop the run first.")
            return
        self.task_name = name
        argv = self.runner.start(args, self.pipe.line.emit, self.pipe.exited.emit,
                                 stoppable=False)
        self.log.note(f"=== {core.format_command(argv)}")
        for b in (self.setup_btn, self.cleanup_btn, self.python_btn):
            b.setEnabled(False)

    def _task_done(self, rc):
        self.log.note(f"=== {self.task_name} {'done' if rc == 0 else f'failed (exit {rc})'}")
        for b in (self.setup_btn, self.cleanup_btn, self.python_btn):
            b.setEnabled(True)
        if self.task_name in ("setup", "update-python"):
            self.win.refresh_fingerprint()


# --------------------------------------------------------------------------- #
# Window
# --------------------------------------------------------------------------- #
class MainWindow(QMainWindow):
    def __init__(self, engine=None, backend=None):
        super().__init__()
        self.engine = engine or core.ENGINE
        self.backend = backend or core
        self.busy = False
        self._async = _Async()
        self.setWindowTitle("FIXS")

        self.header = QLabel("Reading this machine's setup ...")
        self.header.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.tabs = QTabWidget()
        self.run_tab = RunTab(self)
        self.scenario_tab = ScenarioTab(self)
        self.machine_tab = MachineTab(self)
        self.tabs.addTab(self.run_tab, "Run")
        self.tabs.addTab(self.scenario_tab, "Scenario")
        self.tabs.addTab(self.machine_tab, "Machine")
        self.run_tab.configChanged.connect(self.scenario_tab.show_path)

        central = QWidget()
        lay = QVBoxLayout(central)
        lay.addWidget(self.header)
        lay.addWidget(self.tabs)
        self.setCentralWidget(central)
        self.statusBar().showMessage(f"Engine: {self.engine}")

        self.refresh_fingerprint()
        self.run_tab.reload()

    def run_async(self, fn, cb):
        self._async.run(fn, cb)

    def set_busy(self, on):
        self.busy = on

    def refresh_fingerprint(self):
        engine = self.engine
        self.run_async(lambda: self.backend.version(engine=engine), self._fingerprint)

    def _fingerprint(self, fp):
        self.machine_tab.show_fingerprint(fp)
        if isinstance(fp, Exception):
            self.header.setText(f"<b>FIXS</b> - could not read this machine's setup: {fp}")
            return
        missing = [m for m, ok in (fp.get("modules") or {}).items() if not ok]
        warn = (f"  <span style='color:#c62828'>missing: {', '.join(missing)}</span>"
                if missing else "")
        self.header.setText(
            f"<b>FIXS {fp.get('fixs')}</b> &nbsp;&middot;&nbsp; {fp.get('host')} "
            f"&nbsp;&middot;&nbsp; CARLA: {fp.get('carla_mode') or 'not configured'}"
            f" &nbsp;&middot;&nbsp; python: {fp.get('python')}{warn}")

    def closeEvent(self, event):
        r = self.run_tab.runner
        if r.running:
            ans = QMessageBox.question(
                self, "FIXS", "A run is in progress. Stop it and quit?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if ans != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            r.request_stop()
            deadline = time.monotonic() + STOP_GRACE_S
            while r.running and time.monotonic() < deadline:
                QApplication.processEvents()
                time.sleep(0.1)
            if r.running:
                r.kill()
        event.accept()


def main(engine=None, backend=None):
    app = QApplication.instance() or QApplication(sys.argv[:1])
    app.setApplicationName("FIXS")
    win = MainWindow(engine=engine, backend=backend)
    win.resize(1280, 860)
    win.show()
    return app.exec()
