import uuid
import numpy as np
import matplotlib.colors as mcolors
import random


class Robot:
    def __init__(self, start, end, speed, radius, color=None, label=None):
        self._position = np.array(start, dtype=np.float64)
        self.start = np.array(start, dtype=np.float64)
        self.end = np.array(end, dtype=np.float64)
        d = np.linalg.norm(self.start - self.end)
        assert speed < d, "speed is too large."

        self.direction = self.end - self._position
        self.default_velocity = (
            self.direction / np.linalg.norm(self.direction)
        ) * speed
        self.current_velocity = self.default_velocity
        self.speed = speed
        self.radius = radius
        if color is None:
            self.color = random.choice(list(mcolors.cnames.keys()))
        else:
            self.color = color

        if label is None:
            self.label = uuid.uuid1()
        else:
            self.label = label

        self.moving = True  # Add a flag to indicate if the robot is moving

    def __str__(self):
        return self.label

    def __repr__(self):
        return self.label
    def move(self, velocity=None, dt=1):
        if not self.moving:  # If the robot isn't moving, do nothing
            return

        if velocity is None:
            self.current_velocity = self.current_velocity
        else:
            # d = np.linalg.norm(self._position - self.end)
            # assert np.linalg.norm(velocity) < d, "speed is too large."

            self.current_velocity = velocity

        remaining_distance = np.linalg.norm(self.end - self._position)
        if remaining_distance <= self.speed * dt:  # Check if we're close to the end point
            self._position = self.end
            self.moving = False
            self.current_velocity = np.array([0,0])
        else:
            self._position += self.current_velocity * dt

    @property
    def position(self):
        return self._position

    @property
    def velocity(self):
        return self.current_velocity
