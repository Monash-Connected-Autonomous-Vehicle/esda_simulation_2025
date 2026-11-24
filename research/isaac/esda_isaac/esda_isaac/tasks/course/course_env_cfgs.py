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
        self.show_course = True
        # NO generated layouts. An earlier version kept an "oval" filler
        # alongside igvc, which meant half the evaluation episodes ran on a
        # course that is not the one being reported -- the averaged lap length
        # came out at 102.5 m for a 135.7 m course, and every headline number
        # was a blend of two different tasks.
        self.courses = ()
        self.courses_per_layout = 1
        self.include_igvc = True
        self.noise_dropout = 0.0
        self.noise_lane_dropout = 0.0
        self.noise_spurious = 0.0
        self.noise_range_bias = 0.0
