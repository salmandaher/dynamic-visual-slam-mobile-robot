#!/usr/bin/env python3
"""Standalone Isaac Sim demo of the new_arm — no ROS needed.

Drives the simulated arm through a sequence of Cartesian goals and named poses using
our analytic 3-DOF IK, computing the coupled passive wrist each step. A quick visual
confirmation that the USD + kinematics behave before wiring the full ROS stack.

    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/standalone_control.py
    # add --headless to run without a window
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac import config  # noqa: E402
from new_arm_isaac import kinematics as kin  # noqa: E402

parser = argparse.ArgumentParser(description="new_arm standalone Isaac demo")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--usd", default=config.USD_PATH)
parser.add_argument("--hold", type=int, default=120, help="sim steps to hold each goal")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

config.ensure_dirs()
simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402


GOALS = [
    ("named:home", None),
    ("named:ready", None),
    ("xyz", (0.12, 0.05, 0.18)),
    ("xyz", (0.10, -0.06, 0.20)),
    ("xyz", (0.15, 0.0, 0.16)),
    ("named:home", None),
]


def goal_to_thetas(kind, value):
    if kind.startswith("named:"):
        return list(config.NAMED_POSES[kind.split(":", 1)[1]])
    q, ok = kin.inverse_kinematics(value, elbow_up=True)
    if not ok:
        print(f"[demo] WARN target {value} out of reach; best-effort IK")
    err = np.linalg.norm(kin.forward_kinematics(q) - np.array(value))
    print(f"[demo] xyz {value} -> q {np.round(q, 3)} (FK err {err*1000:.3f} mm)")
    return list(q)


def main():
    if not os.path.exists(args.usd):
        print(f"[demo] ERROR: USD not found: {args.usd}; run import_urdf.py first.",
              file=sys.stderr)
        return

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd_path=args.usd, prim_path=config.ROBOT_PRIM)
    robot = SingleArticulation(prim_path=config.ROBOT_PRIM, name="new_arm")
    world.scene.add(robot)
    world.reset()

    dof_names = list(robot.dof_names)
    print(f"[demo] dof_names = {dof_names}")
    act_idx = [dof_names.index(j) for j in config.ACTUATED_JOINTS]
    pas_idx = dof_names.index(config.PASSIVE_JOINT)

    full_target = np.array(robot.get_joint_positions(), dtype=float)

    for kind, value in GOALS:
        thetas = goal_to_thetas(kind, value)
        passive = kin.passive_wrist(thetas[1], thetas[2])
        for k, idx in enumerate(act_idx):
            full_target[idx] = thetas[k]
        full_target[pas_idx] = passive
        for _ in range(args.hold):
            robot.apply_action(ArticulationAction(joint_positions=full_target))
            world.step(render=not args.headless)
            if not simulation_app.is_running():
                return
        meas = np.array(robot.get_joint_positions(), dtype=float)
        print(f"[demo] reached {kind}{value or ''}: "
              f"actuated={np.round([meas[i] for i in act_idx], 3)} wrist={meas[pas_idx]:.3f}")

    print("[demo] tour complete.")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
