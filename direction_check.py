#!/usr/bin/env python3
"""Confirm the drive sign convention: does the PROPOSED mapping make +v drive
toward the robot FRONT (+X, where the D435i looks) and +w turn LEFT (CCW, +yaw)?

Tests the negated mapping  omega = -[(v + w*t/2)/r, (v - w*t/2)/r]  and prints the
resulting dx (want >0 for forward) and dyaw (want >0 for a +w left turn)."""
import math, os, sys, numpy as np
DEFAULT_SCENE = "/home/salman/Documents/simsim/new_built_robot.usd"
ART_ROOT = "/World/robot/robot_base"
DRIVEN = ["right_back", "left_back"]
R, TRACK = 0.39, 3.5

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import is_stage_loading

def log(m): print(f"[dir] {m}", flush=True)
def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def diff_drive(v, w):
    # PROPOSED corrected mapping: +v -> front (+X), +w -> left (CCW)
    omega_r = -(v + w * TRACK * 0.5) / R
    omega_l = -(v - w * TRACK * 0.5) / R
    return np.array([omega_r, omega_l], dtype=np.float32)

def main():
    omni.usd.get_context().open_stage(DEFAULT_SCENE, None)
    simulation_app.update(); simulation_app.update()
    while is_stage_loading(): simulation_app.update()
    dt = 1/120.0
    sim = SimulationContext(physics_dt=dt, rendering_dt=1/60.0, stage_units_in_meters=1.0)
    art = Articulation(prim_paths_expr=ART_ROOT, name="robot_base")
    sim.play(); simulation_app.update(); art.initialize()
    widx = np.array([art.get_dof_index(n) for n in DRIVEN], np.int32)

    def settle(s=1.5):
        for _ in range(int(s/dt)):
            art.set_joint_velocity_targets(np.zeros(2, np.float32).reshape(1,-1), joint_indices=widx)
            sim.step(render=False)
    def drive(v, w, s=1.5):
        tgt = diff_drive(v, w); cur = np.zeros(2, np.float32)
        p0 = np.asarray(art.get_world_poses()[0])[0]; y0 = yaw_of(np.asarray(art.get_world_poses()[1])[0])
        for _ in range(int(s/dt)):
            cur = cur + np.clip(tgt-cur, -0.05, 0.05)
            art.set_joint_velocity_targets(cur.reshape(1,-1), joint_indices=widx)
            sim.step(render=False)
        p1 = np.asarray(art.get_world_poses()[0])[0]; y1 = yaw_of(np.asarray(art.get_world_poses()[1])[0])
        d = p1-p0; dyaw = math.degrees(((y1-y0+math.pi)%(2*math.pi))-math.pi)
        return d, dyaw

    settle(2.0)
    d, dyaw = drive(0.6, 0.0); log(f"FORWARD (v=+0.6): dx={d[0]:+.3f} dy={d[1]:+.3f} dyaw={dyaw:+.1f}  "
                                   f"{'OK front=+X' if d[0] > 0.05 else 'WRONG (moves -X)'}")
    settle(2.0)
    d, dyaw = drive(0.0, 0.6); log(f"TURN LEFT (w=+0.6): dyaw={dyaw:+.1f}  "
                                   f"{'OK left=CCW' if dyaw > 2 else 'WRONG'}")
    sim.stop(); return 0

if __name__ == "__main__":
    try: main()
    except BaseException as e:
        import traceback; print("[dir] FATAL", repr(e)); print(traceback.format_exc())
    finally: simulation_app.close()
