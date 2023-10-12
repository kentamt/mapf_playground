import gym
import gym_env.envs.registration
from gym_env.envs.world import CarEnv

# see this release note:
# https://github.com/openai/gym/releases/tag/0.26.0

env = gym.make('CarEnv-v0')
env.reset()

# Run the environment for 1000 steps
for i in range(1000):
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


