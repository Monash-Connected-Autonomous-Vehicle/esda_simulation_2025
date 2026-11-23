"""PPO settings for the course task.

Starting point, not a tuned result. The values are rsl_rl's usual defaults for
locomotion with two changes that this task argues for:

  num_steps_per_env 32
      The control rate is 8 Hz, so 32 steps is 4 seconds of driving -- long
      enough for a corner to play out inside one rollout segment.

  a [256, 128, 64] trunk
      The observation is 405 numbers of which 384 are pooled range readings.
      That is a wide, low-information input; depth matters less than enough
      width at the first layer to look at the whole scan at once.
"""
from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class EsdaCoursePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32
    max_iterations = 3000
    save_interval = 100
    experiment_name = "esda_course"
    empirical_normalization = True

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.8,
        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
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
