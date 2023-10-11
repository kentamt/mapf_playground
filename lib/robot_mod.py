import uuid
import math
import numpy as np
import matplotlib.colors as mcolors
import random


class StateBase:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

class State(StateBase):

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, wb=1.0):
        super().__init__(x, y, yaw)
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((wb / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((wb / 2) * math.sin(self.yaw))
        self.wb = wb
        self.steer = 0.0

    def update(self, a, delta, dt):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / self.wb * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((self.wb / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.wb / 2) * math.sin(self.yaw))
        self.steer = delta

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class Robot:
    def __init__(
            self,
            start,
            goal,
            speed,
            radius,
            max_speed,
            default_v=None,
            color=None,
            label=None,
    ):

        # spec
        self._radius = radius
        self._max_speed = max_speed

        # path
        self._path_x = None
        self._path_y = None
        self._path_yaw = None

        # start and goal
        self._start = State(start[0], start[1], 0)
        self._goal = State(goal[0], goal[1], 0)

        # kinematics
        if default_v is None:
            vec = self.goal - self.start
            default_v = (vec / np.linalg.norm(vec)) * speed

        self._speed = np.linalg.norm(default_v)
        yaw = np.arctan2(default_v[1], default_v[0])
        self._state = State(start[0], start[1], yaw)

        #
        self._arrived = True

        # check
        d = np.linalg.norm(self.start - self.goal)
        assert speed < d, "speed is too large."

        # viz
        self.trajectory = []

        if color is None:
            self.color = random.choice(list(mcolors.cnames.keys()))
        else:
            self.color = color

        if label is None:
            self.label = uuid.uuid1()
        else:
            self.label = label

    def __str__(self):
        return self.label

    def __repr__(self):
        return self.label

    @property
    def start(self):
        return np.array([self._start.x, self._start.y], dtype=np.float64)

    @property
    def goal(self):
        return np.array([self._goal.x, self._goal.y], dtype=np.float64)

    @property
    def center(self):
        return np.array([self._state.x, self._state.y], dtype=np.float64)
    @property
    def position(self):
        return np.array([self._state.x, self._state.y], dtype=np.float64)

    @property
    def yaw(self):
        return self._state.yaw

    @property
    def radius(self):
        return self._radius

    @property
    def speed(self):
        return self._state.v

    @property
    def max_speed(self):
        return self._max_speed

    @property
    def velocity(self):
        """
        velocity is 2d array, [vx, vy]
        """
        return self._state.v * np.array([np.cos(self._state.yaw),
                                         np.sin(self._state.yaw)])

    @property
    def arrived(self):
        return self._arrived

    @property
    def path_x(self):
        return self._path_x

    @property
    def path_y(self):
        return self._path_y

    @property
    def path_yaw(self):
        return self._path_yaw

    def set_path(self, path_x, path_y, path_yaw):
        self._path_x = path_x
        self._path_y = path_y
        self._path_yaw = path_yaw

    def move(self, velocity, dt=1):
        """
        move a robot
        velocity is 2d array, [vx, vy]
        """

        if not self.arrived:  # If the robot isn't moving, do nothing
            return

        self._state.yaw = np.arctan2(velocity[1], velocity[0])
        self._speed = np.linalg.norm(velocity)

        # Arrived
        r_dist = np.linalg.norm(self.goal - self.position)
        if r_dist <= self._speed * dt:
            self._state = self._goal
            self._arrived = True
            self._speed = 0.0
        # Not arrived
        else:
            self._state.x += velocity[0] * dt
            self._state.y += velocity[1] * dt


class Car(Robot):
    """
    Nonholonomic car-like robot
    wheelbase : wb

    """

    def __init__(
            self,
            start,
            end,
            speed,
            radius,
            wb,
            max_speed,
            default_v=None,
            color=None,
            label=None,
    ):
        super().__init__(
            start[:2], end[:2], speed, radius, max_speed[0], default_v, color, label
        )

        # robot parameters
        # for vis
        self.length = 4.5  # [m]
        self.width = 2.5  # [m]
        self.backtowheel = 0.6  # [m]
        self.wheel_len = 0.9  # [m]
        self.wheel_width = 0.4  # [m]
        self.tread = 0.8  # [m]

        # for control
        self.wb = wb  # [m]
        self.max_steer = np.deg2rad(45.0)  # maximum steering angle [rad]
        self.max_speed_f = max_speed[0]  # [m/s]
        self.max_speed_b = max_speed[1]  # [m/s]

        # state
        self._state = State(x=start[0],
                            y=start[1],
                            yaw=start[2],
                            v=0.0,
                            wb=wb
                            )

        # start and goal
        self._start = State(x=start[0],
                            y=start[1],
                            yaw=start[2],
                            v=0.0,
                            wb=wb)
        self._goal = State(x=end[0],
                           y=end[1],
                           yaw=end[2],
                           v=0.0,
                           wb=wb)

    @property
    def steer(self):
        return self._state.steer
    @property
    def start(self):
        return np.array([self._start.rear_x, self._start.rear_y], dtype=np.float64)

    @property
    def goal(self):
        return np.array([self._goal.rear_x, self._goal.rear_y], dtype=np.float64)

    @property
    def start_center(self):
        return np.array([self._start.x, self._start.y], dtype=np.float64)

    @property
    def goal_center(self):
        return np.array([self._goal.x, self._goal.y], dtype=np.float64)

    @property
    def position(self):
        return np.array([self._state.rear_x, self._state.rear_y], dtype=np.float64)

    def move(self, u=None, dt=1):
        """
         move a robot and update its state
         u[0] : acceleration
         u[1] : yaw rate
        """

        # Check the current state
        if not self.arrived:  # If the robot isn't moving, do nothing
            return

        # Check if we're close to the end point
        r_d = np.linalg.norm(self.goal - self.position)
        if r_d <= self._state.v * dt:
            self._state = self._goal
            self._state.v = 0
            self._arrived = True

        else:
            self._state.update(u[0], u[1], dt)
            self._limit_speed()

    def _limit_speed(self):
        if self._state.v > self.max_speed_f:
            self._state.v = self.max_speed_f
        if self._state.v < self.max_speed_b:
            self._state.v = self.max_speed_b
