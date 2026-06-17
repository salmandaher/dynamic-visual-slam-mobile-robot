#!/usr/bin/env python3
"""Characterize the TorqueNADO-limited robot: full-throttle straight + full-throttle
spin, reporting top speed, 0->top time, and max yaw rate so we know what the real
motors can actually do on this robot.

    ./python.sh motor_test.py [--scene PATH] [--gear 60]
"""
import argparse, math, os, sys
DEFAULT_SCENE = "/home/salman/Documents/simsim/new_built_robot_gui.usd"
ART_ROOT = "/World/robot/robot_base"
DRIVEN = ["right_back", "left_back"]
OZIN_TO_NM = 0.00706155
RPM_TO_RADS = math.pi / 30.0
TORQUENADO = {60: (100, 700, 1440), 40: (150, 466, 960), 20: (300, 233, 480)}

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=DEFAULT_SCENE)
ap.add_argument("--gear", type=int, default=60, choices=[60, 40, 20])
ap.add_argument("--physics-hz", type=int, default=120)
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import numpy as np, omni.usd
from pxr import Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import is_stage_loading


def log(m): print(f"[motor] {m}", flush=True)


def main():
    rpm, ozin, cpr = TORQUENADO[args.gear]
    w_free = rpm * RPM_TO_RADS
    t_stall = ozin * OZIN_TO_NM
    log(f"TorqueNADO {args.gear}:1 -> free {w_free:.2f} rad/s ({rpm} rpm), stall {t_stall:.2f} N*m")

    omni.usd.get_context().open_stage(args.scene, None)
    simulation_app.update(); simulation_app.update()
    while is_stage_loading():
        simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    # ensure the motor model is on the joints for the chosen gear
    k = t_stall / w_free
    for jp in ["/World/robot/_9055_txm_4_inch_wheel/node_/mesh_/right_back",
               "/World/robot/_9055_txm_4_inch_wheel_01/node_/mesh_/left_back"]:
        j = stage.GetPrimAtPath(jp)
        for n, v in (("drive:angular:physics:stiffness", 0.0),
                     ("drive:angular:physics:damping", k),
                     ("drive:angular:physics:maxForce", t_stall)):
            (j.GetAttribute(n) or j.CreateAttribute(n, Sdf.ValueTypeNames.Float)).Set(v)

    dt = 1.0 / args.physics_hz
    sim = SimulationContext(physics_dt=dt, rendering_dt=1/60.0, stage_units_in_meters=1.0)
    art = Articulation(prim_paths_expr=ART_ROOT, name="robot_base")
    sim.play(); simulation_app.update(); art.initialize()
    widx = np.array([art.get_dof_index(n) for n in DRIVEN], np.int32)
    radius = 0.049

    def pose():
        p, o = art.get_world_poses(); return np.asarray(p)[0], np.asarray(o)[0]
    def yaw(o):
        w, x, y, z = o; return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    def lin():
        return np.asarray(art.get_linear_velocities())[0]
    def angz():
        return float(np.asarray(art.get_angular_velocities())[0][2])

    def settle(s=1.5):
        for _ in range(int(s/dt)):
            art.set_joint_velocity_targets(np.zeros(2, np.float32).reshape(1, -1), joint_indices=widx)
            sim.step(render=False)

    # ---- FULL-THROTTLE STRAIGHT (both wheels target = -free speed -> +X) ----
    settle(2.0)
    p0, _ = pose(); t_reach = None; vmax = 0.0
    tgt = np.array([-w_free, -w_free], np.float32)
    N = int(6.0/dt)
    for i in range(N):
        art.set_joint_velocity_targets(tgt.reshape(1, -1), joint_indices=widx)
        sim.step(render=False)
        v = math.hypot(*lin()[:2]); vmax = max(vmax, v)
        if t_reach is None and v >= 0.95 * 0 + 0.95 * vmax and i > 30 and v > 0.5:
            pass
    p1, _ = pose()
    dist = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
    # time to reach 90% of vmax
    settle(2.0); t90 = None; v_hist = []
    p0, _ = pose()
    for i in range(N):
        art.set_joint_velocity_targets(tgt.reshape(1, -1), joint_indices=widx)
        sim.step(render=False)
        v = math.hypot(*lin()[:2]); v_hist.append(v)
        if t90 is None and v >= 0.9*vmax:
            t90 = i*dt
    log(f"FULL-THROTTLE STRAIGHT: top speed {vmax:.2f} m/s "
        f"(wheel free speed * r = {w_free*radius:.2f}), 0->90%% in {t90}s, "
        f"traveled {dist:.1f} m in 6 s")

    # ---- FULL-THROTTLE SPIN (wheels opposite at free speed) ----
    settle(2.0)
    o0 = pose()[1]; y0 = yaw(o0); wmax = 0.0
    tgt = np.array([w_free, -w_free], np.float32)
    for i in range(int(5.0/dt)):
        art.set_joint_velocity_targets(tgt.reshape(1, -1), joint_indices=widx)
        sim.step(render=False)
        wmax = max(wmax, abs(angz()))
    o1 = pose()[1]; y1 = yaw(o1)
    dyaw = math.degrees(((y1-y0+math.pi) % (2*math.pi)) - math.pi)
    log(f"FULL-THROTTLE SPIN: max yaw rate {wmax:.3f} rad/s ({math.degrees(wmax):.1f} deg/s); "
        f"net {dyaw:+.0f} deg in 5 s  -> ~{abs(360/(math.degrees(wmax)+1e-6)):.0f}s per 360 deg")
    sim.stop()
    return 0


if __name__ == "__main__":
    try:
        main()
    except BaseException as e:
        import traceback; print("[motor] FATAL", repr(e)); print(traceback.format_exc())
    finally:
        simulation_app.close()
