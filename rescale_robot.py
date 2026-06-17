#!/usr/bin/env python3
"""Scale the whole robot down so the wheels are a real 4-inch wheel, set the real
masses, and re-seat it on the ground. Runs inside Isaac (live USD/measurement).

The robot was authored ~10x too big (wheels ~0.976 m). We compute the scale ratio
s = 4in / current_wheel_diameter and apply it UNIFORMLY to every robot prim:
  * xformOp:translate  *= s   (positions shrink toward the robot origin)
  * xformOp:scale:unitsResolve *= s   (geometry shrinks)
Joint local anchors are in the same unitsResolve frame, so they scale automatically.
The D435i camera keeps its internal scale (real sensor size) — only its MOUNT
position is scaled. Masses are set from the spec: base 4.5 kg, back wheels 1.332 kg
each, front omni wheels 0.206 kg each (inertia auto-computed from the scaled hulls).

    ./python.sh rescale_robot.py
"""
import math, os, sys
SCENE = "/home/salman/Documents/simsim/new_built_robot.usd"
TARGET_WHEEL_M = 4 * 0.0254          # 4 inch outer diameter = 0.1016 m
ROBOT = "/World/robot"
BASE = "/World/robot/robot_base"
WHEELS = [
    "/World/robot/_9055_txm_4_inch_wheel",
    "/World/robot/_9055_txm_4_inch_wheel_01",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01",
]
REF_WHEEL = WHEELS[0]                 # right_back, used to compute the scale ratio
CAMERA = "/World/robot/d435i_camera"
MASSES = {                           # kg, from the user's spec
    BASE: 4.5,
    "/World/robot/_9055_txm_4_inch_wheel": 1.332,
    "/World/robot/_9055_txm_4_inch_wheel_01": 1.332,
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel": 0.206,
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01": 0.206,
}

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.utils.stage import is_stage_loading


def log(m): print(f"[scale] {m}", flush=True)


def wheel_aabb(stage, cache, path):
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    return mn, mx


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(SCENE, None)
    simulation_app.update(); simulation_app.update()
    while is_stage_loading():
        simulation_app.update()
    stage = ctx.get_stage()

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
    mn, mx = wheel_aabb(stage, cache, REF_WHEEL)
    d = max(mx[0]-mn[0], mx[2]-mn[2])      # wheel outer diameter (round in X/Z)
    s = TARGET_WHEEL_M / d
    log(f"current wheel diameter = {d:.4f} m  ->  scale ratio s = {s:.5f} (target {TARGET_WHEEL_M:.4f} m)")

    def scale_vec_attr(prim, name):
        a = prim.GetAttribute(name)
        if a and a.HasAuthoredValue():
            v = a.Get()
            a.Set(Gf.Vec3d(v[0]*s, v[1]*s, v[2]*s))
            return True
        return False

    # 1) scale geometry + position of base and every wheel
    for path in [BASE] + WHEELS:
        prim = stage.GetPrimAtPath(path)
        t = scale_vec_attr(prim, "xformOp:translate")
        u = scale_vec_attr(prim, "xformOp:scale:unitsResolve")
        log(f"{path.split('/')[-1]:32s} translate*s={t} unitsResolve*s={u}")

    # 2) camera: scale only the MOUNT (row 3 translate); keep internal scale (diagonal)
    cam = stage.GetPrimAtPath(CAMERA)
    a = cam.GetAttribute("xformOp:transform")
    M = Gf.Matrix4d(a.Get())
    r3 = M.GetRow(3)
    M.SetRow(3, Gf.Vec4d(r3[0]*s, r3[1]*s, r3[2]*s, 1.0))
    a.Set(M)
    log(f"camera mount scaled, internal scale kept ({M.GetRow(0)[0]:.4f})")

    # 3) masses (real) + auto inertia from the scaled convex hulls
    for path, m in MASSES.items():
        prim = stage.GetPrimAtPath(path)
        ma = prim.GetAttribute("physics:mass")
        if not ma:
            from pxr import Sdf
            ma = prim.CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float)
        ma.Set(float(m))
        if prim.HasAttribute("physics:diagonalInertia"):
            prim.RemoveProperty("physics:diagonalInertia")
        log(f"{path.split('/')[-1]:32s} mass={m} kg, inertia auto")

    # 4) re-seat on the ground: lowest wheel point -> z ~= +0.003
    cache2 = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
    min_z = min(wheel_aabb(stage, cache2, p)[0][2] for p in WHEELS)
    rmn, rmx = wheel_aabb(stage, cache2, REF_WHEEL)
    new_d = max(rmx[0]-rmn[0], rmx[2]-rmn[2])
    rob = stage.GetPrimAtPath(ROBOT)
    rt = rob.GetAttribute("xformOp:translate")
    v = rt.Get()
    new_z = v[2] + (0.003 - min_z)
    rt.Set(Gf.Vec3d(v[0], v[1], new_z))
    log(f"new wheel diameter = {new_d:.4f} m ; robot z {v[2]:.4f} -> {new_z:.4f} (wheels on ground)")

    # report final base size
    brng = cache2.ComputeWorldBound(stage.GetPrimAtPath(BASE)).ComputeAlignedRange()
    be = brng.GetMax() - brng.GetMin()
    log(f"new base size ~ ({be[0]:.3f}, {be[1]:.3f}, {be[2]:.3f}) m")

    ok = ctx.save_as_stage(SCENE)
    log(f"saved {SCENE}: {ok}")
    return 0


if __name__ == "__main__":
    try:
        main()
    except BaseException as e:
        import traceback; print("[scale] FATAL", repr(e)); print(traceback.format_exc())
    finally:
        simulation_app.close()
