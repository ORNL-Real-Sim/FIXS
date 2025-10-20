#!/usr/bin/env python3
"""Download large simulation assets from OneDrive."""

import urllib.request
from pathlib import Path

ASSETS = {
    "tests/Applications/SUMO_CARLA_EcoDriving/MLK_Carla_Scenario/CARLAFiles/MLK_noped1002_final_debug.fbx":
        "https://outlookuga-my.sharepoint.com/:u:/g/personal/ys04893_uga_edu/EU8vuZV8DvtOlJ6HhpL7Z8MBkhCtVcloma0qxJXXMsPP7A?e=0KgETK&download=1"
}

repo_root = Path(__file__).parent.parent

for file_path, url in ASSETS.items():
    dest = repo_root / file_path
    if dest.exists():
        print(f"OK {file_path} already exists")
        continue

    print(f"Downloading {file_path}...")
    dest.parent.mkdir(parents=True, exist_ok=True)
    urllib.request.urlretrieve(url, dest)
    print(f"OK Downloaded {file_path}")

print("\nAll assets ready!")
