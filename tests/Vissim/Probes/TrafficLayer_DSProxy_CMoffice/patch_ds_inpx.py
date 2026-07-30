"""
Enable VISSIM "driving simulator" mode on a staged copy of a .inpx.

DSProxy's VISSIM_Connect requires the network's <netPara drivSimActive="true">
flag (plus drivSimVehType / drivSimPedType). PTV's shipped
driving_simulator_test.inpx has it; networks authored/imported for normal
simulation (like SimpleEcho's simple_loop.inpx) do NOT, so VISSIM_Connect
fails with "connection got cancelled by VISSIM".

This script copies the source .inpx to the staged path and flips that flag,
referencing an existing vehicle type (default 200, present in simple_loop).
It does NOT modify the committed source .inpx.

Usage:  python patch_ds_inpx.py <src.inpx> <dst.inpx> [veh_type]
"""
from __future__ import annotations
import re
import sys
import pathlib


def patch(src: pathlib.Path, dst: pathlib.Path, veh_type: str = "200") -> None:
    xml = src.read_text(encoding="utf-8")
    m = re.search(r"<netPara\b[^>]*>", xml)
    if not m:
        raise SystemExit(f"ERROR: no <netPara> element found in {src}")
    tag = orig = m.group(0)

    if 'drivSimActive="false"' in tag:
        tag = tag.replace('drivSimActive="false"', 'drivSimActive="true"')
    elif 'drivSimActive="true"' not in tag:
        # no drivSimActive attr at all -> add it
        tag = tag.replace("<netPara ", f'<netPara drivSimActive="true" ', 1)

    if "drivSimVehType=" not in tag:
        tag = tag.replace(
            'drivSimActive="true"',
            f'drivSimActive="true" drivSimPedType="{veh_type}" drivSimVehType="{veh_type}"',
            1,
        )

    xml = xml.replace(orig, tag, 1)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(xml, encoding="utf-8")
    print(f"[patch_ds_inpx] driving-simulator mode enabled -> {dst}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    src = pathlib.Path(sys.argv[1])
    dst = pathlib.Path(sys.argv[2])
    veh = sys.argv[3] if len(sys.argv) > 3 else "200"
    patch(src, dst, veh)
