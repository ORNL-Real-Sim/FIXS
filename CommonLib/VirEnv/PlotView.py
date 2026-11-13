"""PlotSetup: live Plotly plots of command vs actual, fed by the bridge.

Each panel's series are [command, actual, ...]: the first is drawn dashed, the rest solid.
A local HTTP server holds every plot's last ``Window`` seconds and serves a Plotly page
per plot (PlotView.html) that polls it at 10 Hz. Each page opens in a bare browser window
(Edge or Chrome app mode) titled ``FIXS plot: <Name>``; its address is printed and, in a
run_cosim run, written to ``<run dir>/plots.json`` for a recorder to show as a browser
source. ``/clock`` gives the sim time.
"""

import collections
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'PlotView.html')
PLOTLY_CDN = 'https://cdn.plot.ly/plotly-2.35.2.min.js'
_BROWSERS = (r'%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe',
             r'%ProgramFiles%\Microsoft\Edge\Application\msedge.exe',
             r'%ProgramFiles%\Google\Chrome\Application\chrome.exe',
             r'%ProgramFiles(x86)%\Google\Chrome\Application\chrome.exe',
             'microsoft-edge', 'google-chrome', 'chromium', 'chromium-browser')


def windowTitle(name):
    return 'FIXS plot: %s' % name


def unitOf(name):
    """(display unit, scale from the FIXS value) for a series name."""
    base = name.split('.', 1)[-1]
    if 'steerAngle' in base:
        return 'deg', 180.0 / math.pi
    if base.startswith('speed'):
        return 'm/s', 1.0
    if base.startswith('acceleration'):
        return 'm/s²', 1.0
    return '', 1.0


def labelOf(name):
    """'app.lateralOffset' -> 'lateral offset'."""
    return re.sub(r'(?<=[a-z0-9])(?=[A-Z])', ' ', name.split('.', 1)[-1]).lower()


def _panel(p):
    p = p if isinstance(p, dict) else {'Series': list(p)}
    series = list(p['Series'])
    return {'series': series,
            'title': p.get('Title') or labelOf(series[-1]).capitalize(),
            'unit': p.get('Unit') or unitOf(series[-1])[0],
            'labels': list(p.get('Labels') or []) or [labelOf(n) for n in series]}


class Timeline:
    """One plot's last ``Window`` seconds; add() on the bridge thread, since() on the server's."""

    def __init__(self, plot):
        self.name = plot['Name']
        self.target = plot['Target']
        self.size = tuple(plot['Size'])
        self.window = float(plot['Window'])
        self.panels = [_panel(p) for p in plot['Panels']]
        self.series = [n for p in self.panels for n in p['series']]
        self.rows = collections.deque()           # (seq, t, values in self.series order, None = no data)
        self.marks = collections.deque()          # (seq, t, label)
        self.seq = 0
        self.now = 0.0
        self.start = None
        self.last = -math.inf
        self.dt = self.window / (2.0 * self.size[0])   # about two samples per pixel are kept
        self._lock = threading.Lock()

    def add(self, t, values, marks=()):
        """Append the sample at sim time ``t``: values maps series names to FIXS values (None = no data)."""
        with self._lock:
            self.now = t
            if self.start is None:
                self.start = t
            for label in marks:
                self.seq += 1
                self.marks.append((self.seq, t, label))
            if t - self.last >= self.dt:
                self.last = t
                row = []
                for n in self.series:
                    v = values.get(n)
                    row.append(v * unitOf(n)[1] if v is not None and math.isfinite(v) else None)
                self.seq += 1
                self.rows.append((self.seq, t, row))
            cut = t - self.window
            for q in (self.rows, self.marks):
                while q and q[0][1] < cut:
                    q.popleft()

    def spec(self):
        return {'name': self.name, 'title': windowTitle(self.name), 'window': self.window,
                'maxPoints': int(self.window / self.dt) + 16, 'panels': self.panels}

    def since(self, seq):
        """What the page has not seen: rows and marks after ``seq``, times in seconds since the first sample."""
        with self._lock:
            start = self.start if self.start is not None else 0.0
            return {'seq': self.seq, 'now': self.now - start,
                    'rows': [[t - start] + row for s, t, row in self.rows if s > seq],
                    'marks': [[t - start, label] for _, t, label in self.marks]}

    def clock(self):
        with self._lock:
            return {'sim': self.now, 't': self.now - (self.start if self.start is not None else self.now)}


def _plotlyJs():
    """The bytes of plotly.min.js from the installed plotly package, or None."""
    try:
        import plotly
        path = os.path.join(os.path.dirname(plotly.__file__), 'package_data', 'plotly.min.js')
        with open(path, 'rb') as f:
            return f.read()
    except (ImportError, OSError):
        return None


def _browser():
    for b in _BROWSERS:
        path = os.path.expandvars(b) if os.sep in b or '%' in b else shutil.which(b)
        if path and os.path.isfile(path):
            return path
    return None


class PlotWindows:
    """The PlotSetup plots: add() from the bridge loop; a server thread answers the pages."""

    def __init__(self, plots, open_windows=True):
        self.timelines = {p['Name']: Timeline(p) for p in plots}
        self._closed = False
        self._browsers = []
        self.urls = {}
        if not self.timelines:
            return
        self._js = _plotlyJs()
        self._server = ThreadingHTTPServer(('127.0.0.1', 0), self._handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, name='fixs-plots', daemon=True).start()
        base = 'http://127.0.0.1:%d' % self._server.server_address[1]
        self.urls = {n: '%s/plot/%s' % (base, n) for n in self.timelines}
        runDir = os.environ.get('FIXS_RUN_DIR')
        if runDir and os.path.isdir(runDir):
            with open(os.path.join(runDir, 'plots.json'), 'w', encoding='utf-8') as f:
                json.dump({'clock': base + '/clock', 'plots': self.urls}, f, indent=1)
        for i, (n, tl) in enumerate(self.timelines.items()):
            print("PlotSetup '%s': %s" % (n, self.urls[n]))
            if open_windows:
                self._open(tl, self.urls[n], i)

    def _open(self, tl, url, i):
        exe = _browser()
        if exe is None:
            print("PlotSetup '%s': no Edge or Chrome found; open the address above in a browser" % tl.name)
            return
        profile = tempfile.mkdtemp(prefix='fixs_plot_')
        args = [exe, '--app=' + url, '--window-size=%d,%d' % tl.size, '--window-position=%d,%d' % (80 + 60 * i, 80 + 60 * i),
                '--user-data-dir=' + profile, '--no-first-run', '--no-default-browser-check', '--disable-sync']
        try:
            self._browsers.append((subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL), profile))
        except OSError as e:
            print("PlotSetup '%s': could not open a window (%s)" % (tl.name, e), file=sys.stderr)

    def _handler(self):
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _send(self, body, ctype, code=200):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                u = urlparse(self.path)
                parts = u.path.strip('/').split('/')
                tl = owner.timelines.get(parts[1]) if len(parts) == 2 else None
                if parts == ['plotly.min.js'] and owner._js is not None:
                    return self._send(owner._js, 'application/javascript')
                if parts == ['clock'] and owner.timelines:
                    return self._send(json.dumps(next(iter(owner.timelines.values())).clock()).encode(), 'application/json')
                if tl is not None and parts[0] == 'plot':
                    with open(PAGE, encoding='utf-8') as f:
                        page = f.read().replace('{{TITLE}}', windowTitle(tl.name)).replace(
                            '{{PLOTLY}}', '/plotly.min.js' if owner._js is not None else PLOTLY_CDN).replace(
                            '{{NAME}}', tl.name)
                    return self._send(page.encode('utf-8'), 'text/html; charset=utf-8')
                if tl is not None and parts[0] == 'spec':
                    return self._send(json.dumps(tl.spec()).encode(), 'application/json')
                if tl is not None and parts[0] == 'data':
                    seq = int((parse_qs(u.query).get('since') or ['0'])[0])
                    return self._send(json.dumps(tl.since(seq)).encode(), 'application/json')
                self._send(b'not found', 'text/plain', 404)
        return Handler

    def active(self):
        return bool(self.timelines) and not self._closed

    def targets(self):
        return {tl.target for tl in self.timelines.values()}

    def add(self, t, valuesByTarget, marks=()):
        for tl in self.timelines.values():
            tl.add(t, valuesByTarget.get(tl.target) or {}, marks)

    def close(self):
        """Close the windows and stop serving."""
        if self._closed or not self.timelines:
            return
        self._closed = True
        for proc, profile in self._browsers:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                pass
            shutil.rmtree(profile, ignore_errors=True)
        self._server.shutdown()
        self._server.server_close()
