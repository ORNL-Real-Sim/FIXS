################################################################################
# -------- A GUI to run Real-Sim CarMaker-Simulink-Sumo co-simulation -------- #
################################################################################

# Packages: Python 3.9+, PyQt6
from __future__ import annotations
import sys
import os
import time
import subprocess
from dataclasses import dataclass
from typing import List, Optional

from PyQt6.QtCore import Qt, QThread, pyqtSignal, QSize
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFileDialog, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QMessageBox, QGridLayout
)

# -------- MATLAB engine detection --------
# Note that here for 2024a, it can support Python 3.9 up to 3.11
# For more details, please refer to https://pypi.org/project/matlabengine/
MATLAB_AVAILABLE = False
try:
    import matlab.engine  # type: ignore
    MATLAB_AVAILABLE = True
except Exception:
    MATLAB_AVAILABLE = False


# ---------------- State ----------------
@dataclass
class UIState:
    modelPath: str = ""
    inputDir: str = ""      # actually the SUMO .sumocfg file path
    ipgDir: str = ""        # IPG testrun file path
    outputDir: str = ""     # output folder


# ---------------- GUI Worker ----------------
class RunnerWorker(QThread):
    message = pyqtSignal(str)
    error = pyqtSignal(str)
    finished_ok = pyqtSignal()

    def __init__(self, state: UIState, parent=None):
        super().__init__(parent)
        self.state = state

    def log(self, msg: str):
        self.message.emit(msg)

    def run(self):
        try:
            self.log("Starting simulation…")
            run_simulink_job(
                self.state.modelPath,
                self.state.inputDir,   # SUMO .sumocfg path
                self.state.ipgDir,     # IPG testrun file
                self.state.outputDir,
                log_cb=self.log
            )
            self.log(f"✅ Done.\nResults saved to: {self.state.outputDir}")
            self.finished_ok.emit()
        except Exception as e:
            self.error.emit(f"❌ Error:\n{e}")


# ---------------- GUI Main Window ----------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Sim GUI v0.0")
        self.setMinimumSize(QSize(720, 520))
        self.state = UIState()
        self.worker: Optional[RunnerWorker] = None
        self._build_ui()

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)

        lblModel = QLabel("Simulink model (.slx or .mdl)")
        self.edModel = QLineEdit(); self.edModel.setReadOnly(True)

        lblTest = QLabel("IPG Testrun file (.txt)")
        self.edTest = QLineEdit(); self.edTest.setReadOnly(True)

        lblIn = QLabel("Input SUMO configuration file (.sumocfg)")
        self.edIn = QLineEdit(); self.edIn.setReadOnly(True)

        lblOut = QLabel("Output folder")
        self.edOut = QLineEdit(); self.edOut.setReadOnly(True)

        btnPickModel = QPushButton("Browse simulink model"); btnPickModel.clicked.connect(self.onPickModel)
        btnPickTest  = QPushButton("Browse IPG Testrun");    btnPickTest.clicked.connect(self.onPickTestRun)
        btnPickIn    = QPushButton("Browse…");               btnPickIn.clicked.connect(self.onPickFileIn)
        btnPickOut   = QPushButton("Browse…");               btnPickOut.clicked.connect(self.onPickOut)

        self.btnValidate = QPushButton("Validate"); self.btnValidate.clicked.connect(self.onValidate)
        self.btnRun = QPushButton("Run");            self.btnRun.clicked.connect(self.onRun)
        self.btnQuit = QPushButton("Quit");          self.btnQuit.clicked.connect(self.close)

        self.txtLog = QTextEdit(); self.txtLog.setReadOnly(True)

        grid = QGridLayout()
        grid.addWidget(lblModel, 0, 0); grid.addWidget(self.edModel, 1, 0, 1, 3); grid.addWidget(btnPickModel, 1, 3)
        grid.addWidget(lblTest,  2, 0); grid.addWidget(self.edTest,  3, 0, 1, 3); grid.addWidget(btnPickTest,  3, 3)
        grid.addWidget(lblIn,    4, 0); grid.addWidget(self.edIn,    5, 0, 1, 3); grid.addWidget(btnPickIn,    5, 3)
        grid.addWidget(lblOut,   6, 0); grid.addWidget(self.edOut,   7, 0, 1, 3); grid.addWidget(btnPickOut,   7, 3)

        row = QHBoxLayout()
        row.addWidget(self.btnValidate); row.addWidget(self.btnRun); row.addWidget(self.btnQuit); row.addStretch(1)

        root = QVBoxLayout(central)
        root.addLayout(grid)
        root.addLayout(row)
        root.addWidget(self.txtLog)

    def log(self, msg: str):
        self.txtLog.append(msg)
        self.txtLog.moveCursor(self.txtLog.textCursor().MoveOperation.End)

    # --- Pickers ---
    def onPickModel(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select Simulink model", "", "Simulink Models (*.slx *.mdl);;All Files (*)")
        if not path: return
        self.state.modelPath = path; self.edModel.setText(path); self.log(f"Selected model: {path}")

    def onPickTestRun(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select IPG testrun", "", "All Files (*)")
        if not path: return
        self.state.ipgDir = path; self.edTest.setText(path); self.log(f"Selected testrun: {path}")

    def onPickFileIn(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select SUMO config", "", "SUMO Config (*.sumocfg);;All Files (*)")
        if not path: return
        self.state.inputDir = path; self.edIn.setText(path); self.log(f"Selected input: {path}")

    def onPickOut(self):
        path = QFileDialog.getExistingDirectory(self, "Select output folder", "")
        if not path: return
        self.state.outputDir = path; self.edOut.setText(path); self.log(f"Selected output: {path}")

    # --- Validation ---
    def validate(self, s: UIState) -> List[str]:
        errs: List[str] = []
        if not s.modelPath or not os.path.isfile(s.modelPath):
            errs.append("Model path does not exist.")
        else:
            low = s.modelPath.lower()
            if not (low.endswith(".slx") or low.endswith(".mdl")):
                errs.append("Model must be a .slx or .mdl file.")
        if not s.inputDir:
            errs.append("Input folder is invalid.")
        if not s.outputDir:
            errs.append("Output folder is empty.")
        else:
            if not os.path.isdir(s.outputDir):
                try: os.makedirs(s.outputDir, exist_ok=True)
                except Exception as e: errs.append(f"Cannot create output folder: {e}")
        return errs

    def onValidate(self):
        errs = self.validate(self.state)
        if not errs: self.log("✅ Paths look good.")
        else:
            for e in errs: self.log(f"❌ {e}")

    # --- Run ---
    def set_controls_enabled(self, enabled: bool):
        self.btnRun.setEnabled(enabled); self.btnValidate.setEnabled(enabled)

    def onRun(self):
        errs = self.validate(self.state)
        if errs:
            for e in errs: self.log(f"❌ {e}")
            return
        self.set_controls_enabled(False)
        self.worker = RunnerWorker(self.state)
        self.worker.message.connect(self.log)
        self.worker.error.connect(self._on_run_error)
        self.worker.finished_ok.connect(self._on_run_ok)
        self.worker.finished.connect(lambda: self.set_controls_enabled(True))
        self.worker.start()

    def _on_run_ok(self):
        pass

    def _on_run_error(self, msg: str):
        self.log(msg)
        QMessageBox.critical(self, "Run Error", msg)


# ---------------- MATLAB-backed runner (port of your MATLAB function) ----------------
def run_simulink_job(model_path: str, input_dir: str, ipg_dir: str, output_dir: str, log_cb=None):
    """
    Python port of your MATLAB 'run_simulink_job' for Windows.
    - Launches SUMO GUI and TrafficLayer.exe
    - Opens Simulink model, starts TM_Simulink, loads IPG testrun (cmguicmd)
    - Runs sim(model_path), assigns base vars, saves simout.mat
    - Attempts cleanup similar to your taskkill-based approach

    log_cb: optional callable(str) for UI logging
    """
    def log(msg: str):
        if log_cb: log_cb(msg)
        else: print(msg)

    if os.name != "nt":
        raise RuntimeError("This runner currently targets Windows (cmd/start/taskkill).")

    if not MATLAB_AVAILABLE:
        raise RuntimeError("MATLAB Engine for Python not found. Please install and ensure MATLAB is on this machine.")

    # Ensure output dir exists
    os.makedirs(output_dir, exist_ok=True)

    # --- Launch SUMO & Traffic Layer ---
    # Go to the SUMO file directory
    test_dir = os.path.dirname(input_dir)
    if not os.path.isdir(test_dir):
        log(f"⚠️ Warning: Directory not found: {test_dir} (continuing anyway)")

    # SUMO GUI command: start "" sumo-gui -c "<input_dir>" ... --start
    sumo_cmd = [
        "cmd", "/c", "start", "", "sumo-gui",
        "-c", input_dir,
        "--remote-port", "1337",
        "--time-to-teleport", "-1",
        "--time-to-teleport.remove", "false",
        "--max-depart-delay", "-1",
        "--step-length", "0.1",
        "--start"
    ]
    log("⚙️ Launching SUMO GUI…")
    try:
        subprocess.Popen(sumo_cmd, cwd=test_dir)
    except Exception as e:
        log(f"⚠️ Could not start SUMO GUI: {e}")

    # Go to get TrafficLayer.exe and the relevant config.yaml file
    exe_path = os.path.normpath(os.path.join(test_dir, "..", "..", "TrafficLayer.exe"))
    config_filename = os.path.join(test_dir, "RS_config_release.yaml")

    tl_cmd = ["cmd", "/c", "start", "", exe_path, "-f", config_filename]
    log("⚙️ Launching TrafficLayer.exe…")
    try:
        subprocess.Popen(tl_cmd, cwd=test_dir)
    except Exception as e:
        log(f"⚠️ Could not start TrafficLayer.exe: {e}")

    time.sleep(3)  # let them come up

    # --- MATLAB Engine sequence ---
    # Go to the Simulink model directory
    sim_src_dir = os.path.dirname(model_path)
    if not os.path.isdir(sim_src_dir):
        log(f"⚠️ Warning: Directory not found: {sim_src_dir} (continuing anyway)")

    log("⚒️ Starting MATLAB Engine…")
    eng = matlab.engine.start_matlab("-desktop")
    #eng = matlab.engine.start_matlab()
    eng.open_system(model_path, nargout=0)
    eng.CM_Simulink(nargout=0)
    log(f"LoadTestRun , {ipg_dir}")
    eng.cmguicmd(f"LoadTestRun {ipg_dir}", nargout=0)

    try:
        # Run simulation
        log("⚙️ Running Simulink simulation…")
        simOut = eng.sim(model_path, nargout=1)

        log("Simulation finished successfully.")

        # Provide variables to base workspace
        eng.assignin('base', 'INPUT_DIR',  input_dir, nargout=0)
        eng.assignin('base', 'OUTPUT_DIR', output_dir, nargout=0)
        eng.assignin('base', 'SIM_OUT',    simOut,    nargout=0)

        # Save simOut.mat
        out_mat = os.path.join(output_dir, "simout.mat")
        # put simOut into MATLAB workspace, then save from MATLAB side
        eng.workspace['simOut'] = simOut
        log(f'Saving "{out_mat}" …')
        eng.save(out_mat, 'simOut', nargout=0)

    except matlab.engine.MatlabExecutionError as mee:
        raise RuntimeError(f"MATLAB error: {mee}")
    finally:
        try:
            eng.quit()
            #print("Simulation completes!")
        except Exception:
            pass

    # --- Cleanup (mirrors your taskkill approach; ⚠️ kills all matching processes) ---
    # If you'd prefer to only kill what *we* started, I can rework this to track PIDs.
    log("Cleaning up helper terminals (cmd.exe / WindowsTerminal.exe)…")
    try:
        subprocess.run(["taskkill", "/F", "/IM", "cmd.exe"], capture_output=True, text=True)
    except Exception as e:
        log(f"⚠️ taskkill cmd.exe failed: {e}")
    try:
        subprocess.run(["taskkill", "/F", "/IM", "WindowsTerminal.exe"], capture_output=True, text=True)
    except Exception as e:
        log(f"⚠️ taskkill WindowsTerminal.exe failed: {e}")


# ---------------- Entrypoint ----------------
def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
