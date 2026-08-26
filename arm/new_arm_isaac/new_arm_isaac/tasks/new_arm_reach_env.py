"""NewArm-Reach — an IsaacLab DirectRLEnv for the 3-DOF new_arm.

Mirrors the conservative style of maxarm_charger_align_env (fixed root, all joints
driven), but:
  * the ACTION is joint-space (3 joint deltas) — no IK in the hot loop, fully batched
    in torch — and
  * the passive wrist is commanded to its mechanical coupling each step (so the sim
    matches the real arm), using new_arm_isaac.kinematics_torch.

Task: drive the wrist tip (end_effector_link origin) to a randomly sampled target in
the workspace. Observation = [pos_error(3), q(3), dq(3), prev_action(3)] = 12.

Run inside IsaacLab:
    cd /home/salman/Documents/grad_robot/IsaacLab
    ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task NewArm-Reach-v0 --headless

Requires the USD from new_arm_isaac/scripts/import_urdf.py.

Verified API (IsaacLab 0.54.2): DirectRLEnv/DirectRLEnvCfg, ArticulationCfg +
ImplicitActuatorCfg, InteractiveSceneCfg, SimulationCfg, configclass — against
/home/salman/Documents/grad_robot/IsaacLab/source/isaaclab and the maxarm env.
"""

from __future__ import annotations

import math
import os
import sys
from collections.abc import Sequence

import torch

try:
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
    from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.sensors import TiledCamera, TiledCameraCfg
    from isaaclab.sim import SimulationCfg
    from isaaclab.utils import configclass
except Exception as exc:  # pragma: no cover
    raise ImportError(
        "Run from an Isaac Lab Python env (./isaaclab.sh -p ...)."
    ) from exc

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from new_arm_isaac import config  # noqa: E402
from new_arm_isaac import kinematics_torch as kt  # noqa: E402


@configclass
class NewArmReachEnvCfg(DirectRLEnvCfg):
    # --- RL timing ---
    episode_length_s = 5.0
    decimation = 4
    action_space = 3          # 3 actuated joint deltas
    observation_space = 12    # pos_error(3) + q(3) + dq(3) + prev_action(3)
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1.0 / 120.0, render_interval=decimation)

    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1024, env_spacing=1.0, replicate_physics=True, clone_in_fabric=False)

    # --- robot: the converted USD (run import_urdf.py first) ---
    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/NewArm",
        spawn=sim_utils.UsdFileCfg(
            usd_path=config.USD_PATH,
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                fix_root_link=True,
                enabled_self_collisions=False,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos={
                "after_base_full_joint": 0.0,
                "link1_joint": 0.0,
                "link2_joint": 0.0,
                "end_effector_joint": 0.0,  # coupled home value (wrist level; offset 2*pi)
            },
            joint_vel={".*": 0.0},
        ),
        actuators={
            "all_position_drives": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit_sim=100.0,
                velocity_limit_sim=20.0,
                stiffness=5000.0,
                damping=150.0,
            ),
        },
        soft_joint_pos_limit_factor=0.95,
    )

    # --- optional eye-in-hand RGB camera (Logitech HD webcam) ---
    # OFF by default: the verified proprioceptive reach policy (observation_space=12) is
    # untouched and headless training stays at full throughput. Set enable_camera=True AND
    # pass --enable_cameras to the runner to harvest rgb (e.g. for a visual policy).
    # spawn=None => reuse the camera prim ALREADY baked into new_arm.usd by import_urdf.py
    # (so its EE-frame offset + 45deg forward/down tilt + webcam intrinsics are inherited;
    # the OffsetCfg is intentionally NOT set here because spawn=None reads the prim's own
    # baked pose). Read images via env.camera_rgb() once enabled.
    enable_camera = False
    ee_camera: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/envs/env_.*/NewArm/" + config.TIP_LINK + "/" + config.CAMERA_PRIM_NAME,
        spawn=None,
        data_types=["rgb"],
        width=config.CAMERA_RL_RESOLUTION[0],
        height=config.CAMERA_RL_RESOLUTION[1],
    )

    arm_joint_names = list(config.ACTUATED_JOINTS)
    passive_joint_names = [config.PASSIVE_JOINT]
    tip_body_name = config.TIP_LINK

    # --- task ---
    # Action = per-step joint delta. Raised 3deg -> 6deg/step so the policy can
    # traverse the workspace within the ~150-step horizon (5s @ 30Hz).
    max_joint_step = math.radians(6.0)
    joint_limit = math.pi                # == URDF actuated-joint limit (+/-3.14 rad)

    # Reachable-target sampling: targets are the FK of random joint configs (so they
    # are reachable BY CONSTRUCTION). Shoulder/elbow tightened from the full +/-pi so
    # targets stay in a sane frontal shell; base yaw spans the full circle.
    sample_q_base = (-math.pi, math.pi)     # after_base_full_joint (yaw)
    sample_q_shoulder = (-1.2, 1.2)         # link1_joint (pitch)
    sample_q_elbow = (-1.2, 1.2)            # link2_joint (pitch)
    target_z_range = (0.05, 0.34)           # keep targets above the table/base
    target_r_min = 0.05                     # keep off the base axis (avoid near-base singularity)

    # Success threshold — used ONLY for the bullseye bonus + logged metric, NOT for
    # episode termination (episodes are fixed length, like the IsaacLab reach task).
    success_pos_error = 0.01             # metres (1 cm)

    # reward weights — IsaacLab-style bounded tanh attractor. Every term is bounded and
    # NOT episode-length dependent, so cumulative episode reward reflects reach quality
    # (the old -w_pos*d shape just accumulated with episode length -> worsening curve).
    w_reach = 1.0                        # coarse tanh attractor (gradient over whole workspace)
    reach_std = 0.10                     # metres
    w_fine = 0.5                         # fine tanh attractor (sub-cm precision)
    fine_std = 0.02                      # metres
    w_dist = 0.2                         # small coarse L2 distance penalty (IsaacLab weight)
    w_action_rate = 0.002                # penalize jerky action changes
    w_joint_vel = 0.001                  # penalize joint speed
    w_bullseye = 2.0                     # dense bonus while within success_pos_error
    # Graded sub-cm CENTERING bonus: a sharp gaussian that peaks at d=0 and is ~0 by the
    # 1 cm tolerance, so (unlike the binary bullseye) it keeps pulling the tip to the
    # socket CENTRE instead of letting the policy park at the tolerance edge. Default 0 ->
    # the verified reach reward is unchanged; the insert task turns it on.
    w_center = 0.0
    center_std = 0.004                   # metres (4 mm) - gaussian width of the centering well

    dome_intensity = 2000.0              # scene dome light (subclasses may dim for camera capture)
    debug_print = False


class NewArmReachEnv(DirectRLEnv):
    cfg: NewArmReachEnvCfg

    def __init__(self, cfg: NewArmReachEnvCfg, render_mode: str | None = None, **kwargs):
        self.actions = None
        self.prev_actions = None
        self.q_target_all = None
        self.target_pos_w = None
        self.arm_joint_ids = None
        self.passive_joint_ids = None
        self.tip_body_id = None
        self.target_markers = None
        self.ee_camera = None
        self._global_step = 0
        super().__init__(cfg, render_mode, **kwargs)
        self._init_handles()

    # ---------------- scene ----------------
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot
        # Optional eye-in-hand RGB camera (reuses the prim baked into the USD; spawn=None).
        # Created BEFORE cloning so it is replicated into every env.
        if self.cfg.enable_camera:
            self.ee_camera = TiledCamera(self.cfg.ee_camera)
            self.scene.sensors["ee_camera"] = self.ee_camera
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])
        light = sim_utils.DomeLightCfg(intensity=self.cfg.dome_intensity, color=(0.75, 0.75, 0.75))
        light.func("/World/Light", light)

        # Visualize the targets — ONLY when actually rendering (GUI / video / play), so
        # headless training pays nothing. Subclasses override _create_target_markers /
        # _visualize_markers to show a different marker (e.g. the insert task's socket model).
        if self.sim.render_mode >= self.sim.RenderMode.PARTIAL_RENDERING:
            self._create_target_markers()

    def _create_target_markers(self):
        """Small red spheres at the reach targets (overridden by the insert task)."""
        self.target_markers = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/Visuals/reach_targets",
            markers={
                "target": sim_utils.SphereCfg(
                    radius=0.012,
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(0.9, 0.1, 0.1), emissive_color=(0.35, 0.0, 0.0)),
                ),
            },
        ))

    def _visualize_markers(self):
        """Place the target markers each step (overridden to add orientation for sockets)."""
        if self.target_markers is not None:
            self.target_markers.visualize(translations=self.target_pos_w)

    def _init_handles(self):
        if self.arm_joint_ids is not None:
            return
        # preserve_order=True so arm_joint_ids stay in cfg order [base, shoulder, elbow];
        # the passive-wrist coupling (q[:,1]=shoulder, q[:,2]=elbow) and the obs depend
        # on this column mapping.
        self.arm_joint_ids, _ = self.robot.find_joints(
            self.cfg.arm_joint_names, preserve_order=True)
        try:
            self.passive_joint_ids, _ = self.robot.find_joints(
                self.cfg.passive_joint_names, preserve_order=True)
        except Exception:
            self.passive_joint_ids = []
        # Track the TRUE end-effector and fail loud if it is absent. A silent fallback to
        # the wrong link would put a fixed ~0.14 m offset between the reward frame and the
        # FK-sampled targets, making the task unlearnable.
        try:
            self.tip_body_id, _ = self.robot.find_bodies(self.cfg.tip_body_name)
        except Exception as exc:
            raise RuntimeError(
                f"Tip body {self.cfg.tip_body_name!r} not found in articulation; "
                f"available bodies: {self.robot.body_names}") from exc
        if len(self.arm_joint_ids) != 3:
            raise RuntimeError(
                f"Expected 3 actuated joints, got {len(self.arm_joint_ids)}; "
                f"joints={self.robot.joint_names}")

        n = self.num_envs
        self.actions = torch.zeros((n, 3), device=self.device)
        self.prev_actions = torch.zeros_like(self.actions)
        self.q_target_all = torch.zeros_like(self.robot.data.joint_pos)
        self.target_pos_w = torch.zeros((n, 3), device=self.device)

    # ---------------- actions ----------------
    def _pre_physics_step(self, actions: torch.Tensor):
        self._init_handles()
        self.prev_actions[:] = self.actions
        self.actions[:] = torch.clamp(actions, -1.0, 1.0)

        q = self.robot.data.joint_pos[:, self.arm_joint_ids]
        q_target = torch.clamp(
            q + self.actions * self.cfg.max_joint_step,
            -self.cfg.joint_limit, self.cfg.joint_limit)

        self.q_target_all = self.robot.data.joint_pos.clone()
        self.q_target_all[:, self.arm_joint_ids] = q_target
        if self.passive_joint_ids:
            passive = kt.passive_wrist(q_target[:, 1], q_target[:, 2])
            for jid in self.passive_joint_ids:
                self.q_target_all[:, jid] = passive

    def _apply_action(self):
        self._init_handles()
        self.robot.set_joint_position_target(self.q_target_all)

    # ---------------- obs / reward / done ----------------
    def _tip_pos_w(self):
        return self.robot.data.body_pos_w[:, self.tip_body_id[0], :]

    def camera_rgb(self):
        """RGB tensor (num_envs, H, W, 3) uint8 from the EE webcam, or None if disabled.

        Requires cfg.enable_camera=True and the runner launched with --enable_cameras.
        """
        if self.ee_camera is None:
            return None
        return self.ee_camera.data.output["rgb"]

    def _get_observations(self) -> dict:
        self._init_handles()
        self._visualize_markers()
        pos_error = self.target_pos_w - self._tip_pos_w()
        q = self.robot.data.joint_pos[:, self.arm_joint_ids]
        dq = self.robot.data.joint_vel[:, self.arm_joint_ids]
        return {"policy": torch.cat([pos_error, q, dq, self.prev_actions], dim=-1)}

    def _get_rewards(self) -> torch.Tensor:
        self._init_handles()
        d = torch.linalg.norm(self.target_pos_w - self._tip_pos_w(), dim=-1)
        dq = self.robot.data.joint_vel[:, self.arm_joint_ids]
        action_rate = torch.linalg.norm(self.actions - self.prev_actions, dim=-1)
        dq_norm = torch.linalg.norm(dq, dim=-1)
        within = d < self.cfg.success_pos_error

        # Bounded tanh attractor (each term lives in a fixed range, so cumulative
        # episode reward tracks reach quality, not episode length).
        r_reach = self.cfg.w_reach * (1.0 - torch.tanh(d / self.cfg.reach_std))
        r_fine = self.cfg.w_fine * (1.0 - torch.tanh(d / self.cfg.fine_std))
        r_dist = -self.cfg.w_dist * d
        r_arate = -self.cfg.w_action_rate * action_rate
        r_jvel = -self.cfg.w_joint_vel * dq_norm
        r_bull = self.cfg.w_bullseye * within.float()
        r_center = self.cfg.w_center * torch.exp(-((d / self.cfg.center_std) ** 2))

        reward = r_reach + r_fine + r_dist + r_arate + r_jvel + r_bull + r_center

        self._global_step += 1
        if not hasattr(self, "extras") or self.extras is None:
            self.extras = {}
        # Assign a FRESH dict each step (not setdefault+update on a persisted one) so
        # rsl_rl's per-step ep_infos hold distinct snapshots and the logged values are
        # true rollout means rather than just the last step's values.
        log = {
            "metric/mean_pos_error_mm": (1000.0 * d.mean()).item(),
            "metric/success_rate": within.float().mean().item(),
            "reward/reach": r_reach.mean().item(),
            "reward/fine": r_fine.mean().item(),
            "reward/dist": r_dist.mean().item(),
            "reward/bullseye": r_bull.mean().item(),
            "reward/center": r_center.mean().item(),
        }
        # Per-EPISODE TERMINAL accuracy. metric/success_rate above is a per-step TIME
        # AVERAGE over the whole rollout, so the unavoidable travel phase of every
        # fixed-length episode bounds it well below 1 (empirically ~0.8) even for a
        # perfect policy -> it is the wrong thing to gate "accuracy" on. On the step an
        # episode truncates (episode_length_buf has already been incremented for this
        # step in DirectRLEnv.step, and _reset_idx runs AFTER _get_rewards, so the tip
        # is still at its FINAL pose), record per-env whether the plug tip is within
        # tolerance. rsl_rl concatenates these 1-D tensors across the rollout and means
        # them -> a true per-episode insertion success rate (and terminal error).
        term_mask = self.episode_length_buf >= (self.max_episode_length - 1)
        if bool(term_mask.any()):
            log["metric/ep_terminal_success"] = within[term_mask].detach().float()
            log["metric/ep_terminal_err_mm"] = (1000.0 * d[term_mask]).detach()
        self.extras["log"] = log
        if self.cfg.debug_print and self._global_step % 100 == 0:
            print(f"[reach] step {self._global_step} mean_err "
                  f"{1000*d.mean().item():.1f} mm  succ "
                  f"{100*within.float().mean().item():.0f}%")
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self._init_handles()
        # Fixed-length episodes (like the IsaacLab reach task): never terminate on
        # success, so the policy learns to reach AND hold, and mean episode reward is
        # a clean monotonic signal of reach quality.
        terminated = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        truncated = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, truncated

    # ---------------- reset ----------------
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        self._init_handles()

        env_ids_t = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        n = len(env_ids_t)

        # sample reachable targets (FK of random joint configs -> reachable by construction)
        target_local = self._sample_reachable_targets(n)
        self.target_pos_w[env_ids_t] = target_local + self.scene.env_origins[env_ids_t]

        # reset arm to home (coupled wrist = 0; offset 2*pi => level wrist at home)
        joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        joint_pos[:, self.arm_joint_ids] = 0.0
        if self.passive_joint_ids:
            for jid in self.passive_joint_ids:
                joint_pos[:, jid] = 0.0
        joint_vel = torch.zeros_like(joint_pos)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids)

        self.actions[env_ids_t] = 0.0
        self.prev_actions[env_ids_t] = 0.0

    def _sample_reachable_targets(self, n: int) -> torch.Tensor:
        """(n,3) targets in the base frame, each reachable by construction.

        Samples random joint configs within tightened limits, runs the batched FK,
        and keeps the tips that sit above the table and off the base axis (rejection
        sampling, oversampled so it converges in 1-2 iterations).
        """
        lo = torch.tensor(
            [self.cfg.sample_q_base[0], self.cfg.sample_q_shoulder[0], self.cfg.sample_q_elbow[0]],
            device=self.device)
        hi = torch.tensor(
            [self.cfg.sample_q_base[1], self.cfg.sample_q_shoulder[1], self.cfg.sample_q_elbow[1]],
            device=self.device)
        z_min, z_max = self.cfg.target_z_range
        r_min = self.cfg.target_r_min

        collected = []
        got = 0
        for _ in range(64):  # safety cap; home (q=0) is always valid so this converges fast
            if got >= n:
                break
            m = max(4 * (n - got), 256)
            q = lo + (hi - lo) * torch.rand((m, 3), device=self.device)
            t = kt.forward_kinematics(q, device=self.device)        # (m,3) base frame
            r = torch.linalg.norm(t[:, :2], dim=-1)
            ok = (t[:, 2] >= z_min) & (t[:, 2] <= z_max) & (r >= r_min)
            good = t[ok]
            if good.shape[0] > 0:
                collected.append(good)
                got += good.shape[0]
        out = torch.cat(collected, dim=0) if collected else torch.empty((0, 3), device=self.device)
        if out.shape[0] < n:
            raise RuntimeError(
                f"reachable-target sampler under-filled: {out.shape[0]}/{n} after 64 tries; "
                "loosen target_z_range/target_r_min or the q sample ranges.")
        return out[:n]
