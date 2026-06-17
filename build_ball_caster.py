#!/usr/bin/env python3
"""Make a COPY of the robot that replaces the two front "dual omni" wheels with
BALL CASTERS (Isaac Sim 5.1.0).

WHY: the dual-omni wheels can't be simulated faithfully (no real rollers), so we
drop their colliders and put a free-sliding low-friction SPHERE (ball caster) at
each front-wheel location instead — simple, stable, omnidirectional support.

WHAT it does (writes a NEW file, never touches new_built_robot.usd):
  1. Disables the collider on each front omni-wheel mesh (visual mesh stays) and
     gives those (now collider-less) wheel links a small explicit inertia so the
     articulation link stays valid.
  2. Adds two ball-caster rigid bodies welded to the base with a fixed joint
     (excludeFromArticulation, exactly like the D435i camera), one under each
     front wheel, sized so the sphere bottom sits on the ground (level robot).
  3. Binds the existing low-friction /World/Looks/casters physics material.
  4. Saves as new_built_robot_ballcaster.usd.

Run inside Isaac (USD/PhysX frame math is reliable here):
    ./python.sh build_ball_caster.py
"""
import argparse, math, os, sys

IN_SCENE = "/home/salman/Documents/simsim/new_built_robot.usd"
OUT_SCENE = "/home/salman/Documents/simsim/new_built_robot_ballcaster.usd"
ROBOT = "/World/robot"
BASE = "/World/robot/robot_base"
CASTERS_MAT = "/World/Looks/casters"
# front omni-wheel rigid-body roots + their collider meshes
FRONT = {
    "right_front": "/World/robot/_6466_txm_4_inch_dual_omni_wheel",
    "left_front":  "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01",
}

ap = argparse.ArgumentParser()
ap.add_argument("--in", dest="inp", default=IN_SCENE)
ap.add_argument("--out", default=OUT_SCENE)
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf
from isaacsim.core.utils.stage import is_stage_loading


def log(m): print(f"[ball] {m}", flush=True)


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(args.inp, None)
    simulation_app.update(); simulation_app.update()
    while is_stage_loading():
        simulation_app.update()
    stage = ctx.get_stage()
    log(f"opened {args.inp}")

    # --- base world transform (rigid part only) ---
    base_prim = stage.GetPrimAtPath(BASE)
    M_base = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    M_base_rigid = Gf.Matrix4d(M_base).GetOrthonormalized()      # drop scale/shear
    Rb = M_base_rigid.ExtractRotationQuat()                       # clean base world rotation
    M_base_inv = M_base.GetInverse()                             # world -> base-local (cm) incl. scale
    log(f"base world rot quat = {Rb}")

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])

    # --- 1) disable each front omni collider + keep the link valid ---
    for name, root in FRONT.items():
        mesh = stage.GetPrimAtPath(root + "/node_/mesh_")
        if mesh and mesh.IsValid():
            ce = mesh.GetAttribute("physics:collisionEnabled")
            if not ce:
                ce = mesh.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool)
            ce.Set(False)
            log(f"{name}: collider disabled")
        # collider-less link -> give it explicit (small) inertia so it's valid
        wb = stage.GetPrimAtPath(root)
        for attr, vt, val in [("physics:mass", Sdf.ValueTypeNames.Float, 0.6),
                              ("physics:diagonalInertia", Sdf.ValueTypeNames.Float3, Gf.Vec3f(0.01, 0.01, 0.01))]:
            a = wb.GetAttribute(attr) or wb.CreateAttribute(attr, vt)
            a.Set(val)

    # --- 2) ball casters at each front-wheel ground contact ---
    for name, root in FRONT.items():
        wheel = stage.GetPrimAtPath(root)
        bound = cache.ComputeWorldBound(wheel).ComputeAlignedRange()
        mn, mx = bound.GetMin(), bound.GetMax()
        cx, cy = (mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5
        radius = (mx[2] - mn[2]) * 0.5            # wheel half-height ~= contact radius
        wcenter = Gf.Vec3d(cx, cy, radius)        # sphere center so its bottom touches z=0
        log(f"{name}: ball caster center world={tuple(round(v,3) for v in wcenter)} R={radius:.3f}")

        cpath = f"{ROBOT}/ball_caster_{name}"
        cprim = UsdGeom.Xform.Define(stage, cpath).GetPrim()
        # place the caster body at the contact location, in /World/robot frame
        # (/World/robot has identity orient & unit scale, translate (0,0,-0.1692))
        robot_xf = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT))
        M_robot = robot_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local = M_robot.GetInverse().Transform(wcenter)
        cxf = UsdGeom.Xform(cprim)
        cxf.AddTranslateOp().Set(Gf.Vec3d(local))
        # rigid body + small mass
        UsdPhysics.RigidBodyAPI.Apply(cprim)
        mass = UsdPhysics.MassAPI.Apply(cprim)
        mass.CreateMassAttr(0.2)
        # sphere collider child (radius in the unit-scale robot frame == metres)
        sp = UsdGeom.Sphere.Define(stage, cpath + "/geom")
        sp.CreateRadiusAttr(float(radius))
        sp.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.1, 0.12)])
        spp = sp.GetPrim()
        UsdPhysics.CollisionAPI.Apply(spp)
        PhysxSchema.PhysxCollisionAPI.Apply(spp)
        spp.GetAttribute("physxCollision:contactOffset") or spp.CreateAttribute(
            "physxCollision:contactOffset", Sdf.ValueTypeNames.Float)
        spp.GetAttribute("physxCollision:contactOffset").Set(0.02)
        spp.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
        # low-friction caster material (physics purpose only)
        mb = UsdPhysics.MaterialAPI  # noqa: F841 (ensure import side-effects)
        rel = spp.CreateRelationship("material:binding:physics", False)
        rel.SetTargets([Sdf.Path(CASTERS_MAT)])

        # fixed joint base <-> caster (weld), excluded from the articulation
        jpath = cpath + "/weld_to_base"
        joint = UsdPhysics.FixedJoint.Define(stage, jpath)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(BASE)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(cpath)])
        # anchor at the caster origin (world wcenter): body1 local = origin, body0 local = base-frame coords
        lp0 = M_base_inv.Transform(wcenter)
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(lp0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(Rb.GetInverse()))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.GetPrim().CreateAttribute("physics:excludeFromArticulation",
                                        Sdf.ValueTypeNames.Bool).Set(True)
        log(f"{name}: caster body+weld authored (base-local anchor {tuple(round(v,2) for v in lp0)})")

    # --- 3) save copy ---
    ok = ctx.save_as_stage(args.out)
    log(f"saved {args.out}: {ok}")
    return 0


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:  # noqa: BLE001
        import traceback; print("[ball] FATAL", repr(exc)); print(traceback.format_exc())
    finally:
        simulation_app.close()
