import numpy as np

class SimpleReward:
    def compute(self, obs, n_obs):
        """
        Compute the reward based on the observation and the next observation of the robot
        :param obs:
        :param n_obs:
        :return:
        """
        # obs = [x, y, yaw, v, gx, gy, gyaw]

        # position of the robot
        x, y, _, _, gx, gy, gyaw = obs

        # next position of the robot
        n_x, n_y, yaw, _, _, _, _ = n_obs

        # distance between obs and goal
        d = np.sqrt((gx - x)**2 + (gy - y)**2)

        # distance between n_obs and goal
        n_d = np.sqrt((gx - n_x)**2 + (gy - n_y)**2)

        reward = d - n_d

        reward += -1

        if n_d < 2.0 and np.abs(yaw - gyaw) < np.radians(10):
            reward += 100


        return reward

