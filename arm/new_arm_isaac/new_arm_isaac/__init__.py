"""new_arm Isaac Sim package: analytic kinematics, digital twin, and an RL reach task.

Importing this module registers the gym task `NewArm-Reach-v0`. The gym.register is
wrapped in try/except so the lightweight, ROS-free parts (`new_arm_isaac.kinematics`,
`new_arm_isaac.config`) stay importable in a plain Python where IsaacLab is absent.
"""

try:  # only succeeds inside an IsaacLab python env
    import gymnasium as gym

    from . import agents  # noqa: F401
    from .tasks.new_arm_reach_env import NewArmReachEnv, NewArmReachEnvCfg  # noqa: F401
    from .tasks.new_arm_insert_env import NewArmInsertEnv, NewArmInsertEnvCfg  # noqa: F401
    from .tasks.new_arm_vision_insert_env import (  # noqa: F401
        NewArmVisionInsertEnv, NewArmVisionInsertEnvCfg)

    gym.register(
        id="NewArm-Reach-v0",
        entry_point="new_arm_isaac.tasks.new_arm_reach_env:NewArmReachEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": "new_arm_isaac.tasks.new_arm_reach_env:NewArmReachEnvCfg",
            "rsl_rl_cfg_entry_point": "new_arm_isaac.agents.rsl_rl_ppo_cfg:NewArmReachPPORunnerCfg",
        },
    )

    gym.register(
        id="NewArm-Insert-v0",
        entry_point="new_arm_isaac.tasks.new_arm_insert_env:NewArmInsertEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": "new_arm_isaac.tasks.new_arm_insert_env:NewArmInsertEnvCfg",
            "rsl_rl_cfg_entry_point": "new_arm_isaac.agents.rsl_rl_ppo_cfg:NewArmInsertPPORunnerCfg",
        },
    )

    gym.register(
        id="NewArm-VisionInsert-v0",
        entry_point="new_arm_isaac.tasks.new_arm_vision_insert_env:NewArmVisionInsertEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": "new_arm_isaac.tasks.new_arm_vision_insert_env:NewArmVisionInsertEnvCfg",
            "rsl_rl_cfg_entry_point": "new_arm_isaac.agents.rsl_rl_ppo_cfg:NewArmVisionInsertPPORunnerCfg",
        },
    )
except Exception:  # noqa: BLE001 - kinematics/config still usable without IsaacLab
    pass
