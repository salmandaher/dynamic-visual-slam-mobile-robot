"""RSL-RL PPO runner configuration for the NewArm-Reach task.

Mirrors the maxarm_charger PPO config (same net + algorithm), retuned only in
name/iterations for the simpler 3-DOF reach.
"""

from __future__ import annotations

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class NewArmReachPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 800
    save_interval = 50
    experiment_name = "new_arm_reach"
    empirical_normalization = False

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[128, 128, 64],
        critic_hidden_dims=[128, 128, 64],
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class NewArmInsertPPORunnerCfg(NewArmReachPPORunnerCfg):
    """Insert PPO: same net as reach, but LOWER/MORE CONSERVATIVE updates for stability.

    With the reach default (lr=1e-3, desired_kl=0.01) the deterministic terminal accuracy
    rose to ~94% by iter ~500 then DEGRADED with further updates (PPO over-optimization /
    late collapse), capping the achievable peak below 95%. A lower base LR + tighter KL
    keep the policy improving instead of collapsing, so the rising curve crosses 95%.
    """

    experiment_name = "new_arm_insert"
    max_iterations = 1200

    # The 128/128/64 net + entropy 0.005 plateaued at a robust ~94.5% terminal accuracy
    # with a UNIFORM ~5% rate of total-miss trajectories (not localized, not precision:
    # p90 err 3.8 mm). That signature = capacity/exploration limit. So: BIGGER net (more
    # capacity to avoid committing to a bad attractor) + MORE entropy (visit & correct the
    # failure states during training). Trained fresh with the fast lr=1e-3/kl=0.01 that
    # learns quickly; pick the peak checkpoint, then fine-tune it (lower lr) to lock it in.
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[256, 256, 128],
        critic_hidden_dims=[256, 256, 128],
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,        # was 0.005 -> more exploration to cover failure states
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class NewArmVisionInsertPPORunnerCfg(NewArmReachPPORunnerCfg):
    """End-to-end vision insertion (camera-feature obs) with ASYMMETRIC actor-critic: the actor uses
    the camera obs ('policy'), the critic uses the privileged ground-truth state ('critic')."""

    experiment_name = "new_arm_vision_insert"
    max_iterations = 1500
    obs_groups = {"policy": ["policy"], "critic": ["critic"]}
