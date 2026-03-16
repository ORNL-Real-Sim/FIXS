# Create the step-1 GUI script and a tiny requirements file
from pathlib import Path

import json
import os
from pathlib import Path
import PySimpleGUI as sg

APP_TITLE = "Simulink Runner – Step 1 (Path Picker)"
PRESET_FILE = Path.home() / ".sim_gui_preset.json"

def validate_paths(model_path: str, input_dir: str, output_dir: str):
    errors = []
    mp = Path(model_path).expanduser()
    if not mp.is_file():
        errors.append(f"Model path does not exist: {mp}")
    elif mp.suffix.lower() != ".slx":
        errors.append(f"Model must be a .slx file: {mp.name}")

    ip = Path(input_dir).expanduser()
    if not ip.exists() or not ip.is_dir():
        errors.append(f"Input folder is not a directory: {ip}")

    op = Path(output_dir).expanduser()
    if not op.exists():
        try:
            op.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            errors.append(f"Output folder could not be created: {op} ({e})")
    elif not op.is_dir():
        errors.append(f"Output path is not a directory: {op}")

    return errors

def save_preset(values):
    data = {
        "model": values.get("-MODEL-", ""),
        "in_dir": values.get("-IN-", ""),
        "out_dir": values.get("-OUT-", ""),
    }
    try:
        with open(PRESET_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        return True, f"Preset saved to {PRESET_FILE}"
    except Exception as e:
        return False, f"Failed to save preset: {e}"

def load_preset(window):
    if not PRESET_FILE.exists():
        return False, f"No preset file found at {PRESET_FILE}"
    try:
        with open(PRESET_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        window["-MODEL-"].update(data.get("model",""))
        window["-IN-"].update(data.get("in_dir",""))
        window["-OUT-"].update(data.get("out_dir",""))
        return True, f"Preset loaded from {PRESET_FILE}"
    except Exception as e:
        return False, f"Failed to load preset: {e}"

def main():
    sg.theme("SystemDefault")
    layout = [
        [sg.Text("Simulink model (.slx)", size=(22,1)),
         sg.Input(key="-MODEL-", expand_x=True, tooltip="Full path to your .slx model"),
         sg.FileBrowse(file_types=(("Simulink Model","*.slx"),), tooltip="Browse for a .slx file")],
        [sg.Text("Input folder", size=(22,1)),
         sg.Input(key="-IN-", expand_x=True, tooltip="Folder containing your model inputs"),
         sg.FolderBrowse(tooltip="Browse for input directory")],
        [sg.Text("Output folder", size=(22,1)),
         sg.Input(key="-OUT-", expand_x=True, tooltip="Folder where results/logs will be saved"),
         sg.FolderBrowse(tooltip="Browse for output directory")],
        [sg.Button("Validate", key="-VALIDATE-"),
         sg.Button("Save Preset", key="-SAVE-"),
         sg.Button("Load Preset", key="-LOAD-"),
         sg.Push(),
         sg.Button("Run (disabled in Step 1)", key="-RUN-", disabled=True),
         sg.Button("Quit")],
        [sg.Multiline(size=(100,12), key="-LOG-", autoscroll=True, write_only=True, expand_x=True, expand_y=True)]
    ]

    window = sg.Window(APP_TITLE, layout, resizable=True)

    while True:
        event, values = window.read()
        if event in (sg.WIN_CLOSED, "Quit"):
            break

        if event == "-VALIDATE-":
            errs = validate_paths(values["-MODEL-"], values["-IN-"], values["-OUT-"])
            if errs:
                for e in errs:
                    window["-LOG-"].print("❌", e)
            else:
                window["-LOG-"].print("✅ Paths look good. (MATLAB run will be wired in Step 2)")

        elif event == "-SAVE-":
            errs = validate_paths(values["-MODEL-"], values["-IN-"], values["-OUT-"])
            if errs:
                for e in errs:
                    window["-LOG-"].print("❌", e)
            ok, msg = save_preset(values)
            window["-LOG-"].print(("✅ " if ok else "❌ ") + msg)

        elif event == "-LOAD-":
            ok, msg = load_preset(window)
            window["-LOG-"].print(("✅ " if ok else "❌ ") + msg)

        elif event == "-RUN-":
            # Disabled in Step 1. Placeholder for Step 2 wiring to MATLAB.
            window["-LOG-"].print("Run clicked (no-op in Step 1).")

    window.close()

if __name__ == "__main__":
    main()
