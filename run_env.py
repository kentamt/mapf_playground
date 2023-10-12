import gym
import gym_env.envs.registration

# There are some changes in the latest version of gym. See this release note:
# https://github.com/openai/gym/releases/tag/0.26.0

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# env_id = 'CarEnv-v0'
# env = make_vec_env(env_id, n_envs=1)  # You can adjust `n_envs` for parallel environments
env = gym.make('CarEnv-v0')
env.reset()
model = PPO('MlpPolicy', env, verbose=1)
model.learn(total_timesteps=100000)
env.close()
exit()


env = gym.make('CarEnv-v0')
env.reset()


# Run the environment for 1000 steps
for i in range(300):
    print(f'{i=}')

    # Sample a random action
    action = env.action_space.sample()

    # Execute the action
    observation, reward, done, truncation, info = env.step(action)

    # Render the environment
    env.render(mode='human')

    # Check if done
    if done:
        print("Episode finished after {} timesteps".format(i + 1))
        env.reset()
        break


print('EOP')


