#!/usr/bin/env python3
"""Headless motion diagnostic for the simsim mobile base (Isaac Sim 5.1.0).

PURPOSE: objectively measure whether the robot moves *correctly* — no ROS, no
GUI, just physics + the Articulation API. It commands a fixed sequence of known
body twists, samples the base pose / velocity / wheel speeds every physics step,
and prints PASS/FAIL metrics plus a JSON dump so we can compare before/after a
fix.

It answers the user's complaints quantitatively:
  * "very slippy"        -> wheel slip ratio  (1 - body_speed / wheel_surface_speed)
  * "ground is slippy"   -> translation efficiency (actual / commanded distance)
  * "motion is annoying" -> bounce (z std), wobble (roll/pitch deviation), veer
  * turning correctness  -> effective track width derived from a pure-spin test

RUN:
    source /opt/ros/humble/setup.bash      # not strictly needed (no ROS) but harmless
    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /home/salman/Documents/simsim/diagnose_motion.py [--scene PATH] [--render]

Output: prints a report; writes /home/salman/Documents/simsim/motion_diag.json
"""
import argparse
import json
import math
import os
import sys

DEFAULT_SCENE = "/home/salman/Documents/simsim/new_built_robot_gui.usd"
ART_ROOT = "/World/robot/robot_base"
DRIVEN = ["right_back", "left_back"]   # 2WD differential drive (back wheels)
OUT_JSON = "/home/salman/Documents/simsim/motion_diag.json"

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=DEFAULT_SCENE)
ap.add_argument("--render", action="store_true", help="render too (slower; default physics-only)")
ap.add_argument("--wheel-radius", type=float, default=0.049,
                help="effective rolling radius (m); calibrated default 0.049 (4-inch wheel)")
ap.add_argument("--track-width", type=float, default=0.56,
                help="assumed track for the v,w->wheel mapping (the test DERIVES the real one); default 0.56")
ap.add_argument("--physics-hz", type=int, default=120)
ap.add_argument("--accel", type=float, default=3.0,
                help="wheel-target slew rate (rad/s^2) — mirrors the real controller's ramp")
ap.add_argument("--label", default="baseline", help="tag stored in the JSON")
args, _ = ap.parse_known_args()

if not os.path.isfile(args.scene):
    sys.exit(f"[diag] scene not found: {args.scene}")

from isaacsim import SimulationApp  # noqa: E402
simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import carb  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402


def log(m):
    print(f"[diag] {m}", flush=True)


def quat_to_rpy(q):
    """q = (w, x, y, z) -> (roll, pitch, yaw) in radians."""
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def diff_drive(v, w, radius, track):
    """body twist -> [right_back, left_back] rad/s (forward = same sign)."""
    omega_r = (v + w * track * 0.5) / radius
    omega_l = (v - w * track * 0.5) / radius
    return np.array([omega_r, omega_l], dtype=np.float32)


def main():
    omni.usd.get_context().open_stage(args.scene, None)
    simulation_app.update(); simulation_app.update()
    log(f"loading {args.scene} ...")
    while is_stage_loading():
        simulation_app.update()
    log("stage loaded")

    sim = SimulationContext(physics_dt=1.0 / max(1, args.physics_hz),
                            rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
    dt = 1.0 / max(1, args.physics_hz)

    art = Articulation(prim_paths_expr=ART_ROOT, name="robot_base")
    sim.play()
    simulation_app.update()
    art.initialize()
    dof_names = list(art.dof_names)
    log(f"dof order = {dof_names}")
    wheel_idx = np.array([art.get_dof_index(n) for n in DRIVEN], dtype=np.int32)
    log(f"driven dof indices = {dict(zip(DRIVEN, wheel_idx.tolist()))}")

    def sample():
        pos, orient = art.get_world_poses()
        pos = np.asarray(pos)[0]
        orient = np.asarray(orient)[0]               # (w,x,y,z)
        lin = np.asarray(art.get_linear_velocities())[0]
        ang = np.asarray(art.get_angular_velocities())[0]
        jvel = np.asarray(art.get_joint_velocities())[0]
        wvel = jvel[wheel_idx]
        return pos, orient, lin, ang, wvel

    cur_target = np.zeros(len(DRIVEN), dtype=np.float32)

    def run_phase(name, v, w, secs, track):
        """Step the sim for `secs` at a constant body twist; slew-ramp the wheel
        targets like the real controller, collect samples."""
        nonlocal cur_target
        target = diff_drive(v, w, args.wheel_radius, track)
        max_delta = max(1e-3, args.accel) * dt
        n = int(secs / dt)
        samples = []
        for _ in range(n):
            cur_target = cur_target + np.clip(target - cur_target, -max_delta, max_delta)
            try:
                art.set_joint_velocity_targets(cur_target.reshape(1, -1), joint_indices=wheel_idx)
            except Exception as e:  # noqa: BLE001
                carb.log_warn(f"[diag] set vel failed: {e!r}")
            sim.step(render=args.render)
            pos, orient, lin, ang, wvel = sample()
            r, p, yaw = quat_to_rpy(orient)
            samples.append(dict(
                pos=pos.tolist(), rpy=[r, p, yaw],
                lin=lin.tolist(), ang=ang.tolist(),
                wvel=wvel.tolist(),
                vh=float(math.hypot(lin[0], lin[1])),     # horizontal body speed
            ))
        return dict(name=name, v=v, w=w, secs=secs, track=track,
                    target_wheel=target.tolist(), samples=samples)

    # ---- let it settle on the ground first ----
    for _ in range(int(1.0 / dt)):
        try:
            art.set_joint_velocity_targets(np.zeros(len(DRIVEN), np.float32).reshape(1, -1),
                                           joint_indices=wheel_idx)
        except Exception:
            pass
        sim.step(render=args.render)
    p0, o0, *_ = sample()
    z_rest = float(p0[2])
    log(f"settled: z_rest = {z_rest:.4f} m, start xy = ({p0[0]:.3f},{p0[1]:.3f})")

    phases = [
        run_phase("settle",   0.0,  0.0, 2.0, args.track_width),
        run_phase("forward",  0.5,  0.0, 3.0, args.track_width),
        run_phase("stop",     0.0,  0.0, 1.0, args.track_width),
        run_phase("spin",     0.0,  0.6, 3.0, args.track_width),
        run_phase("stop2",    0.0,  0.0, 1.0, args.track_width),
        run_phase("backward", -0.5, 0.0, 2.0, args.track_width),
    ]

    # ---------------- metrics ----------------
    def metrics(ph):
        s = ph["samples"]
        P = np.array([x["pos"] for x in s])
        rpy = np.array([x["rpy"] for x in s])
        vh = np.array([x["vh"] for x in s])
        wvel = np.array([x["wvel"] for x in s])     # [n, 2]
        ang = np.array([x["ang"] for x in s])
        net = P[-1] - P[0]
        net_h = float(math.hypot(net[0], net[1]))
        path = float(np.sum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)))
        yaw = np.unwrap(rpy[:, 2])
        yaw_change = float(yaw[-1] - yaw[0])
        ss = slice(len(s) // 2, None)                                    # steady-state window
        wheel_omega_ss = float(np.mean(np.abs(wvel[ss])))                # rad/s, steady-state
        wheel_surface = wheel_omega_ss * args.wheel_radius               # m/s ideal at assumed r
        body_speed = float(np.mean(vh[len(vh) // 2:]))                   # steady-state mean
        slip = float(1.0 - body_speed / wheel_surface) if wheel_surface > 1e-4 else None
        eff_radius = float(body_speed / wheel_omega_ss) if wheel_omega_ss > 1e-3 else None
        m = dict(
            net_disp_h=net_h, path_len=path, net_z=float(net[2]),
            z_mean=float(np.mean(P[:, 2])), z_std=float(np.std(P[:, 2])),
            z_off_rest=float(np.mean(P[:, 2]) - z_rest),
            roll_max_deg=float(np.max(np.abs(rpy[:, 0])) * 180 / math.pi),
            pitch_max_deg=float(np.max(np.abs(rpy[:, 1])) * 180 / math.pi),
            yaw_change_deg=float(yaw_change * 180 / math.pi),
            wheel_surface_speed=wheel_surface, body_speed=body_speed, slip=slip,
            wheel_omega_ss=wheel_omega_ss, eff_radius=eff_radius,
            yaw_rate_mean=float(np.mean(ang[len(ang) // 2:, 2])),
        )
        return m

    report = {p["name"]: metrics(p) for p in phases}

    # derived: effective track width from the spin phase
    sp = report["spin"]
    cmd_w = 0.6
    eff_track = None
    if abs(sp["yaw_rate_mean"]) > 1e-3:
        # commanded with track=args.track_width; yaw_rate = cmd_w * track_assumed / track_real
        eff_track = args.track_width * cmd_w / abs(sp["yaw_rate_mean"])

    fwd = report["forward"]
    expected_fwd = 0.5 * 3.0  # v * t
    trans_eff = fwd["net_disp_h"] / expected_fwd if expected_fwd else None

    summary = dict(
        label=args.label, scene=args.scene, z_rest=z_rest,
        translation_efficiency_fwd=trans_eff,
        forward_slip=fwd["slip"],
        forward_veer_deg=fwd["yaw_change_deg"],
        settle_drift_m=report["settle"]["net_disp_h"],
        settle_z_std=report["settle"]["z_std"],
        bounce_z_std_fwd=fwd["z_std"],
        wobble_roll_max_deg=max(report["settle"]["roll_max_deg"], fwd["roll_max_deg"]),
        wobble_pitch_max_deg=max(report["settle"]["pitch_max_deg"], fwd["pitch_max_deg"]),
        spin_yaw_rate=sp["yaw_rate_mean"], spin_commanded_w=cmd_w,
        effective_track_width_m=eff_track, assumed_track_width_m=args.track_width,
        effective_rolling_radius_m=fwd["eff_radius"], assumed_radius_m=args.wheel_radius,
    )

    log("================= MOTION DIAGNOSTIC =================")
    def verdict(cond, good, bad):
        return ("PASS " + good) if cond else ("FAIL " + bad)
    log(f"resting height z_rest      = {z_rest:+.4f} m")
    log(f"settle drift (should ~0)   = {summary['settle_drift_m']:.4f} m  "
        + verdict(summary['settle_drift_m'] < 0.02, "(stays put)", "(slides while idle!)"))
    log(f"settle z std (bounce)      = {summary['settle_z_std']*1000:.2f} mm  "
        + verdict(summary['settle_z_std'] < 0.003, "(stable)", "(bouncing!)"))
    log(f"forward translation eff.   = {trans_eff*100:.1f}% of commanded  "
        + verdict(trans_eff is not None and trans_eff > 0.6, "(moves)", "(barely moves / slipping!)"))
    log(f"forward wheel slip ratio   = {fwd['slip']}  "
        + verdict(fwd['slip'] is not None and fwd['slip'] < 0.25, "(grips)", "(SLIPPING!)"))
    log(f"forward veer (should ~0)   = {summary['forward_veer_deg']:+.1f} deg  "
        + verdict(abs(summary['forward_veer_deg']) < 10, "(straight)", "(veers!)"))
    log(f"forward bounce z std       = {summary['bounce_z_std_fwd']*1000:.2f} mm  "
        + verdict(summary['bounce_z_std_fwd'] < 0.005, "(smooth)", "(jittery!)"))
    log(f"max roll/pitch wobble      = {summary['wobble_roll_max_deg']:.1f}/{summary['wobble_pitch_max_deg']:.1f} deg  "
        + verdict(max(summary['wobble_roll_max_deg'], summary['wobble_pitch_max_deg']) < 8, "(level)", "(tipping/wobbling!)"))
    log(f"spin yaw rate (cmd {cmd_w})  = {sp['yaw_rate_mean']:+.3f} rad/s")
    log(f"effective track width      = {eff_track}  (assumed {args.track_width})")
    log(f">> CALIBRATION: effective rolling radius = {fwd['eff_radius']}  "
        f"(assumed {args.wheel_radius})")
    log(f">> CALIBRATION: set --wheel-radius {fwd['eff_radius']:.4f} --track-width {eff_track:.3f}"
        if (fwd['eff_radius'] and eff_track) else ">> CALIBRATION: (rerun, degenerate)")
    log("====================================================")

    out = dict(summary=summary, per_phase=report)
    with open(OUT_JSON, "w") as f:
        json.dump(out, f, indent=2)
    log(f"wrote {OUT_JSON}")
    sim.stop()
    return 0


if __name__ == "__main__":
    rc = 1
    try:
        rc = main() or 0
    except BaseException as exc:  # noqa: BLE001
        import traceback
        print("[diag] FATAL:", repr(exc), flush=True)
        print(traceback.format_exc(), flush=True)
    finally:
        simulation_app.close()
    sys.exit(rc)
