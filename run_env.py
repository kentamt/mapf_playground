import gym
import gym_env.envs.registration
# There are some changes in the latest version of gym. See this release note:
# https://github.com/openai/gym/releases/tag/0.26.0

from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, BaseCallback, CallbackList


class RenderCallback(BaseCallback):
    def __init__(self, render_freq: int, verbose=0):
        super(RenderCallback, self).__init__(verbose)
        self.render_freq = render_freq
        self.episode_count = 0

    def _on_step(self) -> bool:
        # Check for end of episode
        if 'done' in self.locals.keys() and bool(self.locals['done']):
            self.episode_count += 1

        if self.episode_count % 500 == 0:

            # Check if the environment is vectorized.
            # If it is, choose one env for rendering, e.g., the first one.
            env = self.locals.get('env', None)
            if hasattr(env, 'envs'):
                env = env.envs[0]

            # Try to render the environment.
            # Catching an exception in case rendering is not supported.
            try:
                env.render()
            except AttributeError as e:
                print(f"Error during rendering: {str(e)}")
                pass

            return True

# env_id = 'CarEnv-v0'
# env = make_vec_env(env_id, n_envs=1)  # You can adjust `n_envs` for parallel environments
env = gym.make('CarEnv-v0')
env.reset()

model = PPO('MlpPolicy', env, verbose=1,  tensorboard_log="./ppo_tensorboard/")
eval_env = gym.make('CarEnv-v0')
eval_callback = EvalCallback(eval_env, best_model_save_path='./logs/',
                             log_path='./logs/', eval_freq=500,
                             deterministic=True, render=False)
render_callback = RenderCallback(render_freq=2000)

model.learn(total_timesteps=10000000, callback=CallbackList([eval_callback, render_callback]))

env.close()
exit()


env = gym.make('CarEnv-v0')
env.reset()


# Run the environment for 1000 steps
for i in range(1000):
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


