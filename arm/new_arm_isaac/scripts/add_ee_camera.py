#!/usr/bin/env python3
"""Patch an EXISTING new_arm.usd to add (or refresh) the EE Logitech HD RGB camera,
WITHOUT a full URDF re-import. Idempotent — safe to run repeatedly.

Use this when you already have a good new_arm.usd (from import_urdf.py) and only want to
add/update the camera. It uses pxr directly and does NOT boot SimulationApp, so it is
fast and avoids the URDF-importer extension (and its ntfs3 symlink gotcha).

Run with Isaac Sim 5.1.0's bundled pxr (no SimulationApp needed):
    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    PKG=$(ls -d extscache/omni.usd.libs-*lx64.r.cp311 | head -1)
    PYTHONPATH="$PWD/$PKG:$PYTHONPATH" LD_LIBRARY_PATH="$PWD/$PKG/bin:$LD_LIBRARY_PATH" \
      ./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/add_ee_camera.py
    # ...or with any python that has pxr:  pip install usd-core && python add_ee_camera.py
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac import config  # noqa: E402
from new_arm_isaac.usd_camera import bake_ee_camera  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="Add the Logitech HD EE camera to new_arm.usd")
    parser.add_argument("--usd", default=config.USD_PATH)
    parser.add_argument("--verify", action="store_true",
                        help="read the camera attributes back after saving")
    args, _ = parser.parse_known_args()

    from pxr import Usd, UsdGeom

    if not os.path.exists(args.usd):
        print(f"ERROR: USD not found: {args.usd}; run import_urdf.py first.", file=sys.stderr)
        return 1

    stage = Usd.Stage.Open(args.usd)
    default_prim = stage.GetDefaultPrim()
    top = default_prim.GetPath().pathString if default_prim else "/new_arm"
    print(f"[add_ee_camera] opened {args.usd} (defaultPrim {top})", flush=True)

    cam_path = bake_ee_camera(stage, top, log=lambda m: print(m, flush=True))
    if not cam_path:
        return 1

    stage.GetRootLayer().Save()
    print(f"[add_ee_camera] saved {args.usd}", flush=True)

    if args.verify:
        v = Usd.Stage.Open(args.usd)
        cam = UsdGeom.Camera(v.GetPrimAtPath(cam_path))
        prim = cam.GetPrim()
        print(f"[verify] prim type     = {prim.GetTypeName()}", flush=True)
        print(f"[verify] xformOpOrder  = "
              f"{[op.GetOpName() for op in UsdGeom.Xformable(prim).GetOrderedXformOps()]}", flush=True)
        print(f"[verify] focalLength   = {cam.GetFocalLengthAttr().Get()}", flush=True)
        print(f"[verify] hAperture     = {cam.GetHorizontalApertureAttr().Get()}", flush=True)
        print(f"[verify] vAperture     = {cam.GetVerticalApertureAttr().Get()}", flush=True)
        print(f"[verify] clippingRange = {cam.GetClippingRangeAttr().Get()}", flush=True)
        print(f"[verify] projection    = {cam.GetProjectionAttr().Get()}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
