"""NewArm-Insert — drive the PLUG TIP into the socket across many parallel envs.

Subclasses NewArm-Reach, so it inherits the EXACT proven training machinery (joint-delta
action, the bounded tanh-attractor reward, fixed-length episodes, reachable-by-construction
target sampling, the optional eye-in-hand camera). It overrides only two things:

  * the tracked point is the PLUG TIP (end of the plug baked on end_effector_link) instead
    of the wrist pivot, and
  * the targets are SOCKET positions sampled to be reachable BY THE PLUG TIP.

Why position-reaching the plug tip == inserting: the passive wrist coupling keeps the EE
LEVEL, so the plug always points radially outward (yawed by the base joint). Sampling sockets
as the plug-tip FK of random joint configs makes every socket both reachable and axially
aligned with the plug's approach — so reducing the plug-tip→socket distance brings the plug
straight into a radially-facing socket. (Contact-rich force insertion is a follow-up; this
trains the reach-and-align that precedes it.)

    cd /home/salman/Documents/grad_robot/IsaacLab
    ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task NewArm-Insert-v0 --headless
"""

from __future__ import annotations

import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply

from new_arm_isaac import config
from new_arm_isaac import kinematics_torch as kt
from new_arm_isaac.tasks.new_arm_reach_env import NewArmReachEnv, NewArmReachEnvCfg

# Plug tip offset along end_effector_link +X (matches usd_socket_plug.bake_plug geometry).
PLUG_TIP_LEN = config.PLUG_OFFSET_XYZ[0] + config.PLUG_BODY_LENGTH + config.PLUG_PRONG_LENGTH


@configclass
class NewArmInsertEnvCfg(NewArmReachEnvCfg):
    # Inherits the proven reach reward shape; the changes below REDUCE error vs the first run:
    #  - a frontal, bounded socket shell (drop the far/edge sockets that pinned the mean high),
    #  - a finer joint step for sub-cm alignment, and a stronger fine-attractor term.
    success_pos_error = 0.01                 # metres ("plug in the hole" tolerance)
    # full (end-of-curriculum) socket shell:
    sample_q_base = (-0.9, 0.9)              # frontal fan -> sockets in front
    sample_q_shoulder = (-1.2, 1.2)
    sample_q_elbow = (-1.2, 1.2)
    target_z_range = (0.06, 0.30)            # sane socket heights
    target_r_min = 0.12                      # keep the socket out in front of the gripper
    target_r_max = 0.42                      # drop the far edge-of-reach sockets
    # PRECISION endgame: the 4deg/1.0-fine config parked the tip at the ~1 cm tolerance
    # edge (89% acc, errors clustered at 10 mm). Sharpen the precision ladder so the
    # optimum is dead-centre, and give finer control + more settle time:
    #   coarse tanh (reach_std 10cm) -> fine tanh (8mm) -> gaussian centering well (4mm).
    max_joint_step = math.radians(3.0)       # finer endgame (was 4deg); 180-step horizon covers travel
    w_fine = 1.5                             # stronger sub-cm attractor
    fine_std = 0.008                         # sharper than base 0.02 -> real gradient inside 1 cm
    w_bullseye = 1.0                         # de-emphasise the binary "in/out" bonus (was 2.0)...
    w_center = 5.0                           # ...in favour of a graded pull to the exact centre
    center_std = 0.004                       # 4 mm centering well
    episode_length_s = 6.0                   # 180 steps @30Hz: more time to settle on hard targets
    dome_intensity = 1200.0                  # dimmer than reach's 2000 -> less blow-out in camera capture

    # CURRICULUM: ramp the JOINT-sample ranges from tight (easy: plug tips cluster near the
    # home-extended pose) to wide (full: plug tips spread across the shell) over curriculum_steps
    # env-steps. The r/z filter above is kept CONSTANT (reachability is by construction; the plug
    # tip is ~0.25 m radial so r is always >=~0.30 -> ramping r separately is meaningless and was
    # buggy). Mastering easy insertions first then generalizing pushes success higher.
    # Reach FULL difficulty early (~iter 250 at 24 steps/iter) so the bulk of training
    # MASTERS the full socket shell instead of chasing a still-widening distribution.
    # The previous 22000 only hit full difficulty at ~iter 917 of 1500, leaving too few
    # iters to converge on the hard targets -> the late "collapse" in the logs.
    curriculum_steps = 6000
    curr_base_easy = 0.3                     # narrow frontal azimuth early
    curr_arm_easy = 0.4                      # tight shoulder/elbow early


class NewArmInsertEnv(NewArmReachEnv):
    cfg: NewArmInsertEnvCfg

    def _plug_tip_offset(self):
        return torch.tensor([PLUG_TIP_LEN, 0.0, 0.0], device=self.device).expand(self.num_envs, 3)

    # --- track the PLUG TIP (end_effector_link world pose * local plug-tip offset) ---
    def _tip_pos_w(self):
        ee_pos = self.robot.data.body_pos_w[:, self.tip_body_id[0], :]
        ee_quat = self.robot.data.body_quat_w[:, self.tip_body_id[0], :]  # wxyz
        return ee_pos + quat_apply(ee_quat, self._plug_tip_offset())

    # --- per-env SOCKET MODEL marker (the actual socket, not just a goal point) ---
    def _create_target_markers(self):
        """A real socket (faceplate + holes) at each target, from socket_marker.usd."""
        import os
        usd = config.SOCKET_MARKER_USD_PATH
        if not os.path.exists(usd):
            raise FileNotFoundError(
                f"socket marker USD missing: {usd}; run scripts/add_socket_plug.py first.")
        self.target_markers = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/Visuals/insert_sockets",
            markers={"socket": sim_utils.UsdFileCfg(usd_path=usd)},
        ))

    def _visualize_markers(self):
        """Place a socket at each target, yawed so its opening faces the base (toward the plug)."""
        if self.target_markers is None:
            return
        local = self.target_pos_w - self.scene.env_origins      # target in each env's frame
        az = torch.atan2(local[:, 1], local[:, 0])              # socket azimuth
        half = 0.5 * az
        quat = torch.zeros((self.num_envs, 4), device=self.device)  # wxyz = Rz(az)
        quat[:, 0] = torch.cos(half)
        quat[:, 3] = torch.sin(half)
        self.target_markers.visualize(translations=self.target_pos_w, orientations=quat)

    # --- sample sockets reachable BY THE PLUG TIP, ramped easy->full by the curriculum ---
    def _sample_reachable_targets(self, n: int) -> torch.Tensor:
        # curriculum fraction 0 (easy) -> 1 (full), by total env-steps seen so far.
        frac = min(1.0, getattr(self, "common_step_counter", 0) / max(1, self.cfg.curriculum_steps))

        def lerp(a, b):
            return a + (b - a) * frac

        base = lerp(self.cfg.curr_base_easy, self.cfg.sample_q_base[1])
        arm = lerp(self.cfg.curr_arm_easy, self.cfg.sample_q_shoulder[1])
        r_min, r_max = self.cfg.target_r_min, self.cfg.target_r_max  # constant filter
        z_min, z_max = self.cfg.target_z_range
        lo = torch.tensor([-base, -arm, -arm], device=self.device)
        hi = torch.tensor([base, arm, arm], device=self.device)

        collected, got = [], 0
        for _ in range(64):
            if got >= n:
                break
            m = max(4 * (n - got), 256)
            q = lo + (hi - lo) * torch.rand((m, 3), device=self.device)
            wrist = kt.forward_kinematics(q, device=self.device)        # (m,3) wrist pivot
            c, s = torch.cos(q[:, 0]), torch.sin(q[:, 0])               # base yaw -> radial dir
            off = PLUG_TIP_LEN * torch.stack([c, s, torch.zeros_like(c)], dim=-1)
            tip = wrist + off                                          # plug tip (EE level)
            r = torch.linalg.norm(tip[:, :2], dim=-1)
            ok = ((tip[:, 2] >= z_min) & (tip[:, 2] <= z_max)
                  & (r >= r_min) & (r <= r_max))
            good = tip[ok]
            if good.shape[0] > 0:
                collected.append(good)
                got += good.shape[0]
        out = torch.cat(collected, dim=0) if collected else torch.empty((0, 3), device=self.device)
        if out.shape[0] < n:
            raise RuntimeError(
                f"insert-target sampler under-filled: {out.shape[0]}/{n} after 64 tries")
        return out[:n]
