#!/usr/bin/env python3
"""Inspect wheel/base geometry & orientation at runtime (USD is safe inside Isaac).

Prints, for the base and each wheel collider mesh:
  * world-space AABB extents  -> true radius & width, detects tilt
  * the wheel body's world orientation (quaternion + which local axis is the spin axis)
so we can design correct cylinder colliders and verify left/right symmetry.
"""
import math, os, sys
DEFAULT_SCENE = "/home/salman/Documents/simsim/new_built_robot.usd"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.utils.stage import is_stage_loading

PRIMS = {
    "base":        "/World/robot/robot_base",
    "right_back":  "/World/robot/_9055_txm_4_inch_wheel",
    "left_back":   "/World/robot/_9055_txm_4_inch_wheel_01",
    "right_front": "/World/robot/_6466_txm_4_inch_dual_omni_wheel",
    "left_front":  "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01",
}

def log(m): print(f"[geom] {m}", flush=True)

def main():
    omni.usd.get_context().open_stage(DEFAULT_SCENE, None)
    simulation_app.update(); simulation_app.update()
    while is_stage_loading():
        simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    log("stage loaded")

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             ["default", "render", "proxy", "guide"], useExtentsHint=False)

    for name, path in PRIMS.items():
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            log(f"{name}: MISSING {path}"); continue
        # world AABB of the whole wheel/base prim
        bound = cache.ComputeWorldBound(prim)
        rng = bound.ComputeAlignedRange()
        mn, mx = rng.GetMin(), rng.GetMax()
        ext = Gf.Vec3d(mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2])
        center = Gf.Vec3d((mx[0]+mn[0])/2, (mx[1]+mn[1])/2, (mx[2]+mn[2])/2)
        # world orientation of the prim
        xf = UsdGeom.Xformable(prim)
        M = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        q = M.ExtractRotationQuat()
        imag = q.GetImaginary()
        log(f"{name:11s} world AABB ext=({ext[0]:.3f},{ext[1]:.3f},{ext[2]:.3f}) "
            f"min_z={mn[2]:+.3f} center=({center[0]:+.2f},{center[1]:+.2f},{center[2]:+.2f}) "
            f"quat(w,xyz)=({q.GetReal():+.3f},{imag[0]:+.3f},{imag[1]:+.3f},{imag[2]:+.3f})")

    # also report the collider mesh sub-prims' local extent (the actual collision geo)
    log("---- collider mesh local extents ----")
    for name, path in PRIMS.items():
        mp = stage.GetPrimAtPath(path + "/node_/mesh_")
        if not mp or not mp.IsValid():
            continue
        mesh = UsdGeom.Mesh(mp)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            log(f"{name}: no points on collider mesh"); continue
        a = np.array([[p[0], p[1], p[2]] for p in pts])
        ext = a.max(0) - a.min(0)
        log(f"{name:11s} LOCAL mesh ext=({ext[0]:.2f},{ext[1]:.2f},{ext[2]:.2f}) "
            f"npts={len(pts)} (units = wheel-local, pre-unitsResolve scale 0.01)")
    return 0

if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:  # noqa: BLE001
        import traceback; print("[geom] FATAL", repr(exc)); print(traceback.format_exc())
    finally:
        simulation_app.close()
