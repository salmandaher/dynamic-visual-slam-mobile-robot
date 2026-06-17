#!/usr/bin/env python3
"""Play the trained NewArm-Reach policy AND capture what the EE Logitech camera sees.

Confirms two things at once: (a) the trained reach policy still runs with the eye-in-hand
camera enabled, and (b) the camera baked onto end_effector_link actually renders. It saves
RGB frames from the EE camera to PNG (a tiled montage of all envs per capture) and prints
the live reach metric (mean tip error + success rate within 1 cm).

Headless capture (recommended — avoids the GUI ntfs-symlink issue; the conda XOpenDisplay
segfault is dodged by unsetting DISPLAY):
    source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
    cd /home/salman/Documents/grad_robot/IsaacLab
    env -u DISPLAY -u XAUTHORITY ./isaaclab.sh -p \
      /home/salman/Documents/our_work/new_arm_isaac/scripts/play_with_camera.py \
      --num_envs 4 --steps 200 --save_every 20

Live GUI view instead (laptop screen, keep DISPLAY=:1, drop --headless): Isaac's Viewport
can be bound to the camera prim — Viewport > Cameras menu, pick
/World/envs/env_0/NewArm/end_effector_link/logitech_camera — to watch the feed live.
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play NewArm-Reach and capture the EE camera")
parser.add_argument("--task", default="NewArm-Reach-v0")
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--steps", type=int, default=200, help="policy steps to run")
parser.add_argument("--save_every", type=int, default=20, help="save an EE-camera montage every N steps")
parser.add_argument("--experiment", default="new_arm_reach", help="logs/rsl_rl/<experiment> to load from")
parser.add_argument("--checkpoint", default=None, help="model_*.pt (default: latest run's highest)")
parser.add_argument("--out", default=None, help="output dir for PNGs (default: <run>/camera_play)")
parser.add_argument("--cam_width", type=int, default=None, help="override camera render width")
parser.add_argument("--cam_height", type=int, default=None, help="override camera render height")
parser.add_argument("--real_time", action="store_true", help="pace the loop to ~real time (for live GUI viewing)")
parser.add_argument("--view_camera", action="store_true",
                    help="GUI: open a docked viewport bound to the EE camera feed")
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()
args.enable_cameras = True  # the whole point — render the EE camera

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# ---- everything else after the app boots ----
import glob
import time

import numpy as np
import torch
import gymnasium as gym
from rsl_rl.runners import OnPolicyRunner

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_tasks.utils import parse_env_cfg

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import new_arm_isaac  # noqa: F401  registers NewArm-Reach-v0
from new_arm_isaac.agents.rsl_rl_ppo_cfg import NewArmReachPPORunnerCfg


def log(m):
    print(f"[play_cam] {m}", flush=True)


def find_checkpoint(cli_path):
    if cli_path:
        return os.path.abspath(cli_path)
    root = os.path.abspath(os.path.join("logs", "rsl_rl", args.experiment))
    runs = sorted(glob.glob(os.path.join(root, "*/")))
    for run in reversed(runs):
        models = glob.glob(os.path.join(run, "model_*.pt"))
        if models:
            models.sort(key=lambda p: int(os.path.basename(p)[6:-3]))  # model_<N>.pt
            return models[-1]
    raise FileNotFoundError(f"no model_*.pt found under {root}; pass --checkpoint")


def save_grid(rgb_np, path):
    """rgb_np: (N,H,W,3) uint8 -> a single tiled PNG montage (falls back to .npy)."""
    n, h, w = rgb_np.shape[:3]
    ncol = min(n, 3)
    nrow = -(-n // ncol)
    grid = np.zeros((nrow * h, ncol * w, 3), dtype=np.uint8)
    for i in range(n):
        r, c = divmod(i, ncol)
        grid[r * h:(r + 1) * h, c * w:(c + 1) * w] = rgb_np[i, :, :, :3]
    try:
        from PIL import Image
        Image.fromarray(grid).save(path)
        return path
    except Exception as e:
        log(f"PIL save failed ({e}); writing {path}.npy instead")
        np.save(path + ".npy", grid)
        return path + ".npy"


def main():
    resume_path = find_checkpoint(args.checkpoint)
    out_dir = args.out or os.path.join(os.path.dirname(resume_path), "camera_play")
    os.makedirs(out_dir, exist_ok=True)
    log(f"checkpoint = {resume_path}")
    log(f"output dir = {out_dir}")

    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
    env_cfg.enable_camera = True  # turn on the eye-in-hand camera for this run
    if args.cam_width:
        env_cfg.ee_camera.width = args.cam_width
    if args.cam_height:
        env_cfg.ee_camera.height = args.cam_height
    log(f"enable_camera = {env_cfg.enable_camera}, num_envs = {env_cfg.scene.num_envs}, "
        f"cam res = {tuple(new_arm_isaac.config.CAMERA_RL_RESOLUTION)}")

    agent_cfg = NewArmReachPPORunnerCfg()
    env = gym.make(args.task, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    log("policy loaded; starting rollout")

    uenv = env.unwrapped
    cam_prim = (f"/World/envs/env_0/NewArm/{new_arm_isaac.config.CAMERA_PARENT_LINK}"
                f"/{new_arm_isaac.config.CAMERA_PRIM_NAME}")

    # GUI live view: open a second, docked viewport bound to the EE camera so the camera
    # feed shows alongside the main perspective view (best-effort; harmless if unavailable).
    if args.view_camera and not args.headless:
        try:
            from omni.kit.viewport.utility import create_viewport_window
            cw, ch = new_arm_isaac.config.CAMERA_RESOLUTION  # match the camera aspect (portrait/landscape)
            sc = 480.0 / max(cw, ch)
            win = create_viewport_window("EE Camera", width=max(1, int(cw * sc)), height=max(1, int(ch * sc)))
            win.viewport_api.camera_path = cam_prim
            log(f"opened 'EE Camera' viewport bound to {cam_prim}")
        except Exception as e:  # noqa: BLE001
            log(f"could not open camera viewport ({e}); use Viewport > Cameras > "
                f"{new_arm_isaac.config.CAMERA_PRIM_NAME} to bind it manually")

    dt = getattr(uenv, "step_dt", uenv.cfg.sim.dt * uenv.cfg.decimation)
    obs = env.get_observations()
    errs, succ = [], []
    saved = []
    for step in range(args.steps):
        if not simulation_app.is_running():
            log("simulation app closed; stopping")
            break
        t0 = time.time()
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)

        d = torch.linalg.norm(uenv.target_pos_w - uenv._tip_pos_w(), dim=-1)
        errs.append(1000.0 * d.mean().item())
        succ.append((d < uenv.cfg.success_pos_error).float().mean().item())

        if step % args.save_every == 0 or step == args.steps - 1:
            rgb = uenv.camera_rgb()
            if rgb is None:
                log("camera_rgb() returned None — camera disabled?")
            else:
                rgb_np = rgb.detach().cpu().numpy().astype(np.uint8)
                nonzero = int((rgb_np > 0).sum())
                p = save_grid(rgb_np, os.path.join(out_dir, f"ee_cam_step{step:04d}.png"))
                saved.append(p)
                log(f"step {step:4d}: tip_err {errs[-1]:6.1f} mm  succ "
                    f"{100*succ[-1]:5.1f}%  | saved {os.path.basename(p)} "
                    f"shape={rgb_np.shape} nonzero_px={nonzero}")

        if args.real_time:
            time.sleep(max(0.0, dt - (time.time() - t0)))

    tail = max(1, len(errs) // 2)
    log(f"DONE. last-half mean tip err = {np.mean(errs[-tail:]):.1f} mm, "
        f"mean success = {100*np.mean(succ[-tail:]):.1f}% over {args.steps} steps")
    log(f"saved {len(saved)} camera montage(s) to {out_dir}")
    for p in saved:
        log(f"  {p}")
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
