import numpy as np
import gym
from gym import error, spaces, utils
from gym.utils import seeding

class CarEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        low = [0,0]
        high = [1.0, 1.0]
        self.action_space = spaces.Box(low=low, high=high)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

    def step(self, action):

        observation = self.observe()
        reward = self.reward()
        done = self.is_done()
        info = {}
        return observation, reward, done, info

    def reset(self):
        # self.observation = self.observation
        self.done = False

    def observe(self):
        pass

    def reward(self):
        pass

    def is_done(self):
        pass

    def render(self, mode='human', close=False):
        pass

    def close(self):
        pass

    def seed(self, seed=None):
        pass