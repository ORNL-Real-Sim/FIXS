"""
carla_env_setup.py - one-time (or reconfigure) CARLA environment setup.

Prompts for the CARLA flavour and folder(s), validates them, and saves the choice
to ~/.fixs/carla.json. run_cosim.py reads that config and launches seamlessly; if
no config exists, run_cosim.py invokes this on the first run.

Run this any time to switch CARLA (packaged <-> source build, or a different
install/version):

    python carla_env_setup.py            # interactive
    python carla_env_setup.py --show     # print the current config
    setup_carla.bat / setup_carla.sh     # thin per-OS wrappers

The config is stored per-machine outside any repo, so every FIXS app on this
computer reuses it and it is never git-tracked.
"""
import argparse
import json
import os
import platform
import sys

CONFIG_DIR = os.path.join(os.path.expanduser("~"), ".fixs")
CONFIG_PATH = os.path.join(CONFIG_DIR, "carla.json")


# ----------------------------------------------------------------- config io

def load_config():
    """Return the saved CARLA env dict, or None if not configured / invalid."""
    try:
        with open(CONFIG_PATH, encoding="utf-8") as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        return None
    return cfg if cfg.get("mode") in ("packaged", "source") and cfg.get("carla_root") else None


def save_config(cfg):
    os.makedirs(CONFIG_DIR, exist_ok=True)
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
    print(f"[setup] saved CARLA env -> {CONFIG_PATH}")


# ------------------------------------------------------------- path resolving

def packaged_exe(carla_root):
    """The packaged CARLA server executable under carla_root, or None."""
    if platform.system() == "Windows":
        cands = [os.path.join(carla_root, "CarlaUE4.exe"),
                 os.path.join(carla_root, "WindowsNoEditor", "CarlaUE4.exe")]
    else:
        cands = [os.path.join(carla_root, "CarlaUE4.sh"),
                 os.path.join(carla_root, "LinuxNoEditor", "CarlaUE4.sh")]
    return next((c for c in cands if os.path.isfile(c)), None)


def source_paths(carla_root, ue4_root):
    """(uproject, ue4editor) paths for a source build."""
    uproject = os.path.join(carla_root, "Unreal", "CarlaUE4", "CarlaUE4.uproject")
    if platform.system() == "Windows":
        editor = os.path.join(ue4_root, "Engine", "Binaries", "Win64", "UE4Editor.exe")
    else:
        editor = os.path.join(ue4_root, "Engine", "Binaries", "Linux", "UE4Editor")
    return uproject, editor


# ----------------------------------------------------------------- prompting

def _pick_dir(title):
    """Native file-explorer folder picker; falls back to a typed path."""
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        path = filedialog.askdirectory(title=title)
        root.destroy()
        if path:
            return path
    except Exception as exc:  # no display / no tkinter
        print(f"[setup] folder picker unavailable ({exc}); type the path instead.")
    typed = input(f"{title}\n  path: ").strip().strip('"')
    return typed or None


def run_setup():
    """Interactive setup; writes and returns the config."""
    print("=== CARLA environment setup ===")
    print("Which CARLA do you want to use?")
    print("  [1] Packaged CARLA  (a released build with CarlaUE4.exe / CarlaUE4.sh)")
    print("  [2] Source build    (run through the Unreal editor: UE4Editor -game)")
    choice = input("Enter 1 or 2: ").strip()

    if choice == "1":
        root = _pick_dir("Select your PACKAGED CARLA folder (contains CarlaUE4.exe / .sh)")
        if not root:
            sys.exit("[setup] cancelled.")
        if not packaged_exe(root):
            sys.exit(f"[setup] no CarlaUE4 launcher found under {root}.")
        cfg = {"mode": "packaged", "carla_root": root}

    elif choice == "2":
        root = _pick_dir("Select your CARLA SOURCE folder (contains Unreal/CarlaUE4/CarlaUE4.uproject)")
        if not root:
            sys.exit("[setup] cancelled.")
        ue4 = os.environ.get("UE4_ROOT") or _pick_dir("Select your Unreal Engine root (UE4_ROOT)")
        if not ue4:
            sys.exit("[setup] cancelled.")
        uproject, editor = source_paths(root, ue4)
        if not os.path.isfile(uproject):
            sys.exit(f"[setup] no CarlaUE4.uproject at {uproject}.")
        if not os.path.isfile(editor):
            sys.exit(f"[setup] no UE4Editor at {editor}.")
        cfg = {"mode": "source", "carla_root": root, "ue4_root": ue4}

    else:
        sys.exit("[setup] invalid choice (expected 1 or 2).")

    save_config(cfg)
    print(f"[setup] done: {cfg['mode']} CARLA @ {cfg['carla_root']}")
    return cfg


def main():
    ap = argparse.ArgumentParser(description="Configure which CARLA run_cosim.py uses.")
    ap.add_argument("--show", action="store_true", help="print the current config and exit")
    args = ap.parse_args()
    if args.show:
        cfg = load_config()
        print(json.dumps(cfg, indent=2) if cfg else f"(no config at {CONFIG_PATH})")
        return 0
    run_setup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
