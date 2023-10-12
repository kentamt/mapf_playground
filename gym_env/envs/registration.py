from gym.envs.registration import register

register(
    id='CarEnv-v0',
    entry_point='gym_env.envs.world:CarEnv',
)
