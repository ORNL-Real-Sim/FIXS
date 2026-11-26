"""fixs.plot: values an in-process controller adds to the bridge's PlotSetup windows.

A panel names them as ``app.<name>``; the bridge samples the latest value each CARLA
step, so a controller puts a value whenever it has a new one.
"""

import threading


class Plot:
    def __init__(self):
        self._lock = threading.Lock()
        self._values = {}
        self._marks = []

    def put(self, name, value):
        """Set series ``app.<name>`` to ``value`` from now on."""
        with self._lock:
            self._values[str(name)] = float(value)

    def mark(self, label):
        """A labelled vertical line at the current step on every plot."""
        with self._lock:
            self._marks.append(str(label))

    def take(self):
        """(values, marks) for this step, for the bridge: the latest values, and the marks since the last take."""
        with self._lock:
            marks, self._marks = self._marks, []
            return dict(self._values), marks


plot = Plot()
