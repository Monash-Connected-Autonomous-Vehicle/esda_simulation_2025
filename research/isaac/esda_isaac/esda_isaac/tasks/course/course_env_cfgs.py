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

        # LONG ENOUGH FOR A LAP. Training inherits 40 s, which is right there
        # -- short episodes mean more resets, more starting positions and a
        # tighter credit horizon. For EVALUATION it made the headline metric
        # impossible to earn: 40 s at the 1.0 m/s cap is 40 m, and the real
        # course is 136.3 m, so the best achievable lap fraction was 0.29.
        # A run measured 0.29 mean against a 0.30 max at 0.99 m/s -- the
        # policy was saturating the clock, and the plan's ">=80% lap
        # completion" target could not be reached by any policy whatsoever.
        #
        # 180 s covers the lap at 0.76 m/s, so a lap left unfinished at this
        # length is the policy's doing and not the timer's. It also makes
        # timeouts meaningful: a timeout now says "too slow or stuck", where
        # before it only said "the lap is longer than 40 seconds".
        self.episode_length_s = 180.0

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
