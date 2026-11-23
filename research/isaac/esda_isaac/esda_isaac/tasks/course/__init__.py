"""Course-following task for the ESDA IGVC vehicle."""
import gymnasium as gym

from . import agents
from .course_env import EsdaCourseEnv, EsdaCourseEnvCfg
from .course_env_cfgs import EsdaCoursePlayEnvCfg

gym.register(
    id="Esda-Course-v0",
    entry_point="esda_isaac.tasks.course.course_env:EsdaCourseEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": EsdaCourseEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.EsdaCoursePPORunnerCfg,
    },
)

# Evaluation: few environments, the REAL course, and no observation noise, so
# a number from it measures the policy rather than the randomisation.
gym.register(
    id="Esda-Course-Play-v0",
    entry_point="esda_isaac.tasks.course.course_env:EsdaCourseEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": EsdaCoursePlayEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.EsdaCoursePPORunnerCfg,
    },
)
