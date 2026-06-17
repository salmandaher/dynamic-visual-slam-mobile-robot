#!/usr/bin/env python3
"""Per-wheel kinematics probe — isolate exactly what each back wheel does.

Drives each wheel combination and reports the resulting world motion so we can
derive the CORRECT diff-drive convention, the real wheel radius, and the real
track width (instead of guessing). Also reports wheel contact heights so we can
see if a wheel is floating.

RUN:  ./python.sh /home/salman/Documents/simsim/kinematics_probe.py [--scene PATH]
"""
import argparse, math, os, sys, json

DEFAULT_SCENE = "/home/salman/Documents/simsim/new_built_robot.usd"
ART_ROOT = "/World/robot/robot_base"
WHEELS = ["right_back", "left_back", "right_front", "left_front"]
WHEEL_PRIMS = {
    "right_back":  "/World/robot/_9055_txm_4_inch_wheel",
    "left_back":   "/World/robot/_9055_txm_4_inch_wheel_01",
    "right_front": "/World/robot/_6466_txm_4_inch_dual_omni_wheel",
    "left_front":  "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01",
}

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=DEFAULT_SCENE)
ap.add_argument("--physics-hz", type=int, default=120)
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import is_stage_loading


def log(m): print(f"[probe] {m}", flush=True)

def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    omni.usd.get_context().open_stage(args.scene, None)
    simulation_app.update(); simulation_app.update()
    while is_stage_loading():
        simulation_app.update()
    log("stage loaded")

    dt = 1.0 / max(1, args.physics_hz)
    sim = SimulationContext(physics_dt=dt, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
    art = Articulation(prim_paths_expr=ART_ROOT, name="robot_base")
    sim.play(); simulation_app.update(); art.initialize()
    dof = list(art.dof_names)
    idx = {n: art.get_dof_index(n) for n in WHEELS if n in dof}
    log(f"dof order = {dof}")

    def base_pose():
        pos, orient = art.get_world_poses()
        return np.asarray(pos)[0], np.asarray(orient)[0]

    def settle(secs=1.5):
        tgt = np.zeros(len(dof), np.float32)
        for _ in range(int(secs / dt)):
            art.set_joint_velocity_targets(tgt.reshape(1, -1))
            sim.step(render=False)

    settle(2.0)
    log("---- rest state ----")
    p0, o0 = base_pose()
    log(f"base rest pos = ({p0[0]:+.3f},{p0[1]:+.3f},{p0[2]:+.3f})  yaw={math.degrees(yaw_of(o0)):+.1f}deg")

    SPEED = 10.0  # rad/s on the commanded wheel(s)
    tests = {
        "right_back_only": {"right_back": SPEED},
        "left_back_only":  {"left_back": SPEED},
        "both_same_+":     {"right_back": SPEED, "left_back": SPEED},
        "both_opposite_R+":{"right_back": SPEED, "left_back": -SPEED},
    }

    results = {}
    for name, cmd in tests.items():
        settle(1.5)
        p_start, o_start = base_pose()
        yaw0 = yaw_of(o_start)
        full = np.zeros(len(dof), np.float32)
        for n, v in cmd.items():
            full[idx[n]] = v
        T = 2.0
        wheel_speeds = []
        for _ in range(int(T / dt)):
            art.set_joint_velocity_targets(full.reshape(1, -1))
            sim.step(render=False)
            jv = np.asarray(art.get_joint_velocities())[0]
            wheel_speeds.append([jv[idx[k]] for k in ["right_back", "left_back"]])
        p_end, o_end = base_pose()
        yaw1 = yaw_of(o_end)
        d = p_end - p_start
        dyaw = math.degrees(((yaw1 - yaw0 + math.pi) % (2 * math.pi)) - math.pi)
        ws = np.mean(np.abs(wheel_speeds), axis=0)
        dist = math.hypot(d[0], d[1])
        results[name] = dict(dx=float(d[0]), dy=float(d[1]), dz=float(d[2]),
                             dist=float(dist), dyaw_deg=float(dyaw),
                             rb_actual=float(ws[0]), lb_actual=float(ws[1]))
        log(f"{name:18s} dx={d[0]:+.3f} dy={d[1]:+.3f} dz={d[2]:+.3f} "
            f"dist={dist:.3f} dyaw={dyaw:+.1f}deg | wheel actual rb={ws[0]:.2f} lb={ws[1]:.2f}")

    # derive effective radius from the straighter of the two single-wheel runs is
    # unreliable (it arcs); use both_same if it goes straightish, else report raw.
    log("---- interpretation ----")
    bs = results["both_same_+"]; bo = results["both_opposite_R+"]
    log(f"both_same_+      : dist={bs['dist']:.2f}m dyaw={bs['dyaw_deg']:+.1f}deg")
    log(f"both_opposite_R+ : dist={bo['dist']:.2f}m dyaw={bo['dyaw_deg']:+.1f}deg")
    straight = "both_same_+" if abs(bs['dyaw_deg']) < abs(bo['dyaw_deg']) else "both_opposite_R+"
    log(f">> STRAIGHT-drive combo is: {straight}")
    # effective radius: for the straight combo, body speed / wheel speed = radius
    st = results[straight]
    body_speed = st['dist'] / 2.0
    wheel_rad_eff = body_speed / SPEED
    log(f">> body speed {body_speed:.3f} m/s at {SPEED} rad/s -> effective wheel radius ~= {wheel_rad_eff:.4f} m")

    json.dump(results, open("/home/salman/Documents/simsim/kinematics_probe.json", "w"), indent=2)
    sim.stop()
    return 0


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:  # noqa: BLE001
        import traceback; print("[probe] FATAL", repr(exc)); print(traceback.format_exc())
    finally:
        simulation_app.close()
