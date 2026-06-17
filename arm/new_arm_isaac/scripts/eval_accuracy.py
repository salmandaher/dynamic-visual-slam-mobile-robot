# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Evaluate the TERMINAL insertion accuracy of one or more RSL-RL checkpoints.

`metric/success_rate` logged during training is a per-step TIME AVERAGE over the whole
rollout, so the unavoidable travel phase of each fixed-length episode bounds it well below
1 even for a perfect policy. The meaningful "accuracy" of an insertion policy is the
PER-EPISODE TERMINAL success: the fraction of episodes whose plug tip is within tolerance
at the FINAL step. This script measures exactly that, on the FULL-difficulty target
distribution (curriculum forced to frac=1), with deterministic (mean) actions.

It can sweep several checkpoints in one Isaac process (build env once, reload each ckpt):

    cd /home/salman/Documents/grad_robot/IsaacLab
    ./isaaclab.sh -p /home/salman/Documents/our_work/new_arm_isaac/scripts/eval_accuracy.py \
        --task NewArm-Insert-v0 --headless --num_envs 1024 --episodes 8 \
        --run_dir logs/rsl_rl/new_arm_insert/<run> --iters 300,500,700,900,1100,1300,1499

Or a single explicit checkpoint with --checkpoint <path/model_X.pt>.
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # for cli_args next to play.py
sys.path.insert(0, "/home/salman/Documents/grad_robot/IsaacLab/scripts/reinforcement_learning/rsl_rl")
import cli_args  # isort: skip  # noqa: E402

parser = argparse.ArgumentParser(description="Evaluate terminal insertion accuracy of RSL-RL checkpoints.")
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=1024)
parser.add_argument("--episodes", type=int, default=8, help="full episodes to evaluate per checkpoint")
# NOTE: --checkpoint is added by cli_args.add_rsl_rl_args below; reuse args_cli.checkpoint.
parser.add_argument("--run_dir", type=str, default=None, help="run dir holding model_*.pt (with --iters)")
parser.add_argument("--iters", type=str, default=None, help="comma list of iteration numbers to eval from --run_dir")
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--diagnose", action="store_true",
                    help="for a single checkpoint: bin terminal accuracy by target azimuth/radius/height")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.envs import DirectRLEnvCfg  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper  # noqa: E402

import new_arm_isaac  # noqa: F401,E402  # registers the NewArm-* tasks
from isaaclab_tasks.utils.hydra import hydra_task_config  # noqa: E402

FORCE_FULL_DIFFICULTY = 10**9  # set common_step_counter so curriculum frac == 1


def _ckpt_list():
    if args_cli.checkpoint:
        return [args_cli.checkpoint]
    if args_cli.run_dir and args_cli.iters:
        rd = os.path.abspath(args_cli.run_dir)
        out = []
        for it in args_cli.iters.split(","):
            p = os.path.join(rd, f"model_{it.strip()}.pt")
            if os.path.exists(p):
                out.append(p)
            else:
                print(f"[eval][warn] missing checkpoint: {p}")
        return out
    raise SystemExit("provide --checkpoint OR (--run_dir AND --iters)")


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: DirectRLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    base = env.unwrapped
    horizon = int(base.max_episode_length)
    tol_mm = 1000.0 * float(base.cfg.success_pos_error)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    checkpoints = _ckpt_list()
    print(f"\n[eval] task={args_cli.task} num_envs={args_cli.num_envs} episodes={args_cli.episodes} "
          f"horizon={horizon} tol={tol_mm:.1f}mm full-difficulty=ON")
    print(f"[eval] {'checkpoint':<22} {'terminal_acc':>12} {'mean_err_mm':>12} {'p90_err_mm':>11} {'n':>7}")

    results = []
    for ckpt in checkpoints:
        runner.load(ckpt)
        policy = runner.get_inference_policy(device=base.device)

        # force full-difficulty sampling, then a clean reset so targets are resampled at frac=1
        base.common_step_counter = FORCE_FULL_DIFFICULTY

        succ_chunks, err_chunks = [], []
        diag_targets, diag_succ = [], []
        steps = args_cli.episodes * horizon + 2
        with torch.inference_mode():
            # reset INSIDE inference_mode: _reset_idx does in-place sim writes that torch
            # forbids on inference tensors outside an InferenceMode context.
            ret = env.reset()
            obs = ret[0] if isinstance(ret, tuple) else ret
            for _ in range(steps):
                actions = policy(obs)
                ret = env.step(actions)
                obs = ret[0]
                policy_nn.reset(ret[2])  # reset recurrent state for done envs (no-op for MLP)
                log = getattr(base, "extras", {}).get("log", {})
                if "metric/ep_terminal_success" in log:
                    s = log["metric/ep_terminal_success"].detach().float().cpu()
                    succ_chunks.append(s)
                    err_chunks.append(log["metric/ep_terminal_err_mm"].detach().float().cpu())
                    # diagnose: episodes are synchronous so on a truncation step term_mask is
                    # all envs -> s aligns with every env's current target (pre-reset).
                    if args_cli.diagnose and s.numel() == base.num_envs:
                        tl = (base.target_pos_w - base.scene.env_origins).detach().cpu()
                        diag_targets.append(tl)
                        diag_succ.append(s)

        succ = torch.cat(succ_chunks) if succ_chunks else torch.tensor([float("nan")])
        err = torch.cat(err_chunks) if err_chunks else torch.tensor([float("nan")])
        acc = succ.mean().item()
        mean_err = err.mean().item()
        p90 = torch.quantile(err, 0.90).item() if err.numel() > 1 else float("nan")
        n = succ.numel()
        name = os.path.basename(ckpt)
        print(f"[eval] {name:<22} {acc:>12.4f} {mean_err:>12.2f} {p90:>11.2f} {n:>7d}")
        results.append((name, acc, mean_err, n))

        if args_cli.diagnose and diag_targets:
            T = torch.cat(diag_targets)        # (N,3) target in env frame
            S = torch.cat(diag_succ)           # (N,) terminal success
            x, y, z = T[:, 0], T[:, 1], T[:, 2]
            azim = torch.rad2deg(torch.atan2(y, x)).abs()   # |azimuth| from +X, 0..180
            r = torch.hypot(x, y)

            def show_bins(label, vals, edges):
                print(f"[diag] {label:>10} | " + "  ".join(
                    f"{edges[i]:.2f}-{edges[i+1]:.2f}:{S[(vals>=edges[i])&(vals<edges[i+1])].mean().item() if ((vals>=edges[i])&(vals<edges[i+1])).any() else float('nan'):.3f}"
                    f"(n{int(((vals>=edges[i])&(vals<edges[i+1])).sum())})"
                    for i in range(len(edges) - 1)))

            print(f"[diag] {name}: terminal-accuracy binned by target geometry (full difficulty)")
            show_bins("|azim|deg", azim, [0, 30, 60, 90, 120, 150, 180.001])
            show_bins("radius_m", r, [0.12, 0.20, 0.26, 0.32, 0.38, 0.4201])
            show_bins("height_m", z, [0.06, 0.12, 0.18, 0.24, 0.3001])

    if results:
        best = max(results, key=lambda r: r[1])
        print(f"\n[eval] BEST: {best[0]}  terminal_accuracy={best[1]:.4f} "
              f"({100*best[1]:.1f}%)  mean_err={best[2]:.2f}mm  n={best[3]}")
        print(f"[eval] PASS (>95%)" if best[1] > 0.95 else "[eval] FAIL (<=95%)")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
