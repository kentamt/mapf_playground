import uuid
import numpy as np
import matplotlib.colors as mcolors
import random


class Robot:
    def __init__(self,
                 start,
                 end,
                 speed,
                 radius,
                 max_speed,
                 default_v=None,
                 color=None,
                 label=None):

        # spec
        self._wb = 0.5 * radius
        self._max_speed = max_speed

        # position
        self.start = np.array(start, dtype=np.float64)
        self.end = np.array(end, dtype=np.float64)
        self._position = np.array(start, dtype=np.float64)

        # kinematics
        if default_v is None:
            vec = self.end - self._position
            default_v = (vec / np.linalg.norm(vec)) * speed

        self._speed = np.linalg.norm(default_v)
        self._radius = radius
        self._yaw = np.arctan2(default_v[1], default_v[0])
        self._moving = True

        # self._velocity = default_v

        # viz
        if color is None:
            self.color = random.choice(list(mcolors.cnames.keys()))
        else:
            self.color = color

        if label is None:
            self.label = uuid.uuid1()
        else:
            self.label = label

        # check
        d = np.linalg.norm(self.start - self.end)
        assert speed < d, "speed is too large."

    def __str__(self):
        return self.label

    def __repr__(self):
        return self.label

    def move(self, velocity=None, dt=1):
        """
        move a robot
        velocity is 2d array, [vx, vy]
        """

        if not self.moving:  # If the robot isn't moving, do nothing
            return

        if velocity is None:
            pass
        else:
            self._yaw = np.arctan2(velocity[1], velocity[0])
            self._speed = np.linalg.norm(velocity)

        remaining_distance = np.linalg.norm(self.end - self._position)
        if remaining_distance <= self._speed * dt:  # Check if we're close to the end point
            self._position = self.end
            self._moving = False
            self._speed = 0.0
        else:
            # velocity = self._speed * np.array([np.cos(self._yaw),
            #                                    np.sin(self._yaw)])
            self._position += velocity * dt

        # if not self.moving:  # If the robot isn't moving, do nothing
        #     return
        #
        # if velocity is None:
        #     pass
        # else:
        #     # d = np.linalg.norm(self._position - self.end)
        #     # assert np.linalg.norm(velocity) < d, "speed is too large."
        #     self._yaw = np.arctan2(velocity[1], velocity[0])
        #     self._velocity = velocity
        #
        # remaining_distance = np.linalg.norm(self.end - self._position)
        # if remaining_distance <= self.speed * dt:  # Check if we're close to the end point
        #     self._position = self.end
        #     self._moving = False
        #     self._velocity = np.array([0, 0])
        # else:
        #     self._position += self._velocity * dt
    def move2(self, u=None, dt=1):
        """
         move a robot and update its state
         u[0] : acceleration
         u[1] : yaw rate
        """

        # Check the current state
        if not self.moving:  # If the robot isn't moving, do nothing
            return

        # Check if we're close to the end point
        remaining_distance = np.linalg.norm(self.end - self._position)
        if remaining_distance <= self._speed * dt:
            self._position = self.end
            self._moving = False
            self._speed = 0

        # Move the robot according to the input
        velocity = self._speed * np.array([np.cos(self._yaw),
                                           np.sin(self._yaw)])
        self._position += velocity * dt
        self._yaw += self._speed / self._wb * np.tan(u[1]) * dt

        if u is None:
            pass
        else:
            self._speed += dt * u[0]
            if self._speed > self._max_speed:
                self._speed = self._max_speed

    @property
    def position(self):
        return self._position

    @property
    def radius(self):
        return self._radius

    @property
    def speed(self):
        return self._speed
    @property
    def max_speed(self):
        return self._max_speed


    @property
    def velocity(self):
        return self._speed * np.array([np.cos(self._yaw), np.sin(self._yaw)])

        # return self._velocity

    @property
    def moving(self):
        return self._moving
