import numpy as np


class SimpleObservation:
    def compute(self, robots):

        # position of the robot
        x   = robots[0].state.x
        y   = robots[0].state.x
        yaw = robots[0].state.x
        v   = robots[0].state.v

        gx   = robots[0].goal_state.x
        gy   = robots[0].goal_state.x
        gyaw = robots[0].goal_state.x

        observation = np.array([x, y, yaw, v, gx, gy, gyaw])

        return observation
