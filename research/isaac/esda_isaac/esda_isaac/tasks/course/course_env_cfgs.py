"""Variants of the course environment."""
from isaaclab.utils import configclass

from .course_env import EsdaCourseEnvCfg


@configclass
class EsdaCoursePlayEnvCfg(EsdaCourseEnvCfg):
    """Evaluation on the real course, clean.

    include_igvc flips to True here and ONLY here. The real course is held out
    of training deliberately -- 34 barrel positions are very memorisable, and a
    policy scored on a course it trained on measures recall, not navigation.

    Observation noise goes to zero so the number reflects the policy. The
    robustness sweep raises it again on purpose; that is a separate question
    from "can it drive the course at all".
    """

    def __post_init__(self):
        if hasattr(super(), "__post_init__"):
            super().__post_init__()
        self.scene.num_envs = 32
        self.courses = ("oval",)        # one filler layout; igvc is the subject
        self.courses_per_layout = 1
        self.include_igvc = True
        self.noise_dropout = 0.0
        self.noise_lane_dropout = 0.0
        self.noise_spurious = 0.0
        self.noise_range_bias = 0.0
