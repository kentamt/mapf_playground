import numpy as np


class SimpleObservation:
    def compute(self, robots):

        # position of the robot
        x   = robots[0].state.x
        y   = robots[0].state.y
        yaw = robots[0].state.yaw
        v   = robots[0].state.v

        gx   = robots[0].goal_state.x
        gy   = robots[0].goal_state.y
        gyaw = robots[0].goal_state.yaw

        observation = np.array([x, y, yaw, v, gx, gy, gyaw])

        return observation
