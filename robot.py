import numpy as np

class Robot:
    def __init__(self, start, end, speed, radius):
        self._position = np.array(start, dtype=np.float64)
        self.start = np.array(start, dtype=np.float64)
        self.end = np.array(end, dtype=np.float64)
        d = np.linalg.norm(self.start - self.end)
        assert speed < d, "speed is too large."

        self.direction = self.end - self._position
        self.default_velocity = (self.direction / np.linalg.norm(self.direction)) * speed
        self.current_velocity = self.default_velocity
        self.speed = speed
        self.radius = radius
        self.moving = True  # Add a flag to indicate if the robot is moving

    def move(self, velocity=None):
        if not self.moving:  # If the robot isn't moving, do nothing
            return

        if velocity is None:
            self.current_velocity = self.current_velocity
        else:
            d = np.linalg.norm(self._position - self.end)
            assert np.linalg.norm(velocity) < d, "speed is too large."

            self.current_velocity = velocity

        remaining_distance = np.linalg.norm(self.end - self._position)
        if remaining_distance <= self.speed:  # Check if we're close to the end point
            self._position = self.end
            self.moving = False
        else:
            self._position += self.current_velocity

    @property
    def position(self):
        return self._position

    @property
    def velocity(self):
        return self.current_velocity
