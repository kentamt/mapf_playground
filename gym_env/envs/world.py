import numpy as np
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym.envs.registration import register

class CarEnv(gym.Env):
    def __init__(self, N=10):
        super(CarEnv, self).__init__()
        metadata = {'render.modes': ['human', 'rbg_array']}

        self.N = N  # Number of vehicles

        # Define action space: [acceleration, steering angle]
        self.action_space = spaces.Box(low=np.array([-1.0, -np.pi / 4]),
                                       high=np.array([1.0, np.pi / 4]),
                                       dtype=np.float64)

        # Define observation space: [x_i, y_i, yaw_i, v_i] for each vehicle
        low_obs = np.array([-np.inf] * 4 * self.N)
        high_obs = np.array([np.inf] * 4 * self.N)
        self.observation_space = spaces.Box(low=low_obs, high=high_obs, dtype=np.float64)

    def step(self, action):
        """
        Execute one time step within the environment.
        """
        # Apply action
        # Your logic here

        # Compute reward
        # Your logic here
        reward = 0

        # Check if done
        # Your logic here
        done = False

        # Update state
        # Your logic here
        observation = np.zeros(self.N * 4)  # Update with your logic

        info = {}

        truncation = False

        return observation, reward, done, truncation, info

    def reset(self, seed=None, options=None):
        """
        Reset the state of the environment to an initial state.
        """
        print('reset is called')

        # Your reset logic here
        initial_observation = np.zeros(self.N * 4)  # Update with your logic

        return initial_observation, {}

    def render(self, mode='human'):
        """
        Render the environment to the screen.
        """
        if mode == 'human':
            pass
            # print(f'rendering for human')


        elif mode == 'rgb_array':
            pass
            # print(f'rendering for rgb_array')

        else:
            raise NotImplementedError(f"Render mode {mode} is not supported. "
                                      f"Supported modes are: {'human', 'rgb_array'}")


        pass

    def close(self):
        """
        Perform any necessary cleanup.
        """
        pass

    # Example usage


#
#
# def main():
#     env = CarEnv(N=5)  # Create environment with 10 vehicles
#     obs = env.reset()  # Reset environment and get initial observation
#     action = env.action_space.sample()  # Get a random action
#     print(action)
#     new_obs, reward, done, info = env.step(action)  # Step the environment with the random action
#     print(f'{new_obs=}')
#     print(f'{reward=}')
#     print(f'{done=}')
#
#
# if __name__ == "__main__":
#     main()
