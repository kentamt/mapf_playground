import numpy as np
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym.envs.registration import register

from gym_env.envs.observation_function import SimpleObservation
from gym_env.envs.reward_function import SimpleReward

from lib.robot_mod import Car
from lib.path_planner import DubinsPathPlanner
from lib.motion_controller import *

from pygame_animation import Screen


class CarEnv(gym.Env):
    def __init__(self, N=1):
        super(CarEnv, self).__init__()
        metadata = {'render.modes': ['human', 'rbg_array']}

        # agents
        self.N = N  # Number of vehicles
        self.robots = self.__init_cars()

        # Define action space: [acceleration, steering angle]
        self.action_space = spaces.Box(low=np.array([-1.0, -np.pi / 4]),
                                       high=np.array([5.0, np.pi / 4]),
                                       dtype=np.float64)

        # Define observation space: [x_i, y_i, yaw_i, v_i] for each vehicle
        # + [x_g, y_g, yaw_g] for each goal
        low_obs = np.array([-np.inf] * (4 + 3) * self.N)
        high_obs = np.array([np.inf] * (4 + 3) * self.N)
        self.observation_space = spaces.Box(low=low_obs, high=high_obs, dtype=np.float64)

        self.observation_function = SimpleObservation()
        self.reward_function = SimpleReward()

        self.max_timesteps = 200
        self.timestep = 0

        # for rendering
        self.screen = Screen()

    def step(self, action):
        """ Execute one time step within the environment. """

        self.timestep += 1

        obs = self.observation_function.compute(self.robots)

        # Apply action
        for r in self.robots:
            u = action
            r.move(u, dt=dt)

        n_obs = self.observation_function.compute(self.robots)
        reward = self.reward_function.compute(obs, n_obs)
        done = self.terminate()
        info = {}
        truncation = False

        return n_obs, reward, done, truncation, info

    def reset(self, seed=None, options=None):
        """ Reset the state of the environment to an initial state. """
        self.timestep = 0

        self.robots = self.__init_cars()
        initial_observation = self.observation_function.compute(self.robots)

        return initial_observation, {}

    def render(self, mode='human'):
        """ Render the environment to the screen. """
        if mode == 'human':
            self.screen.is_quit_event()
            self.screen.update_cars(self.robots)
            self.screen.clock.tick(60)

        elif mode == 'rgb_array':
            pass

        else:
            raise NotImplementedError(f"Render mode {mode} is not supported. "
                                      f"Supported modes are: {'human', 'rgb_array'}")

        pass

    def close(self):
        """ Perform any necessary cleanup. """
        self.screen.quit()

    # -------------------------------------
    def terminate(self):
        is_done = False
        is_done += self.__is_all_cars_arrived(self.robots)
        is_done += self.timestep >= self.max_timesteps

        return bool(is_done)

    # -------------------------------------

    def __is_all_cars_arrived(self, robots):
        num_arrived_cars = 0
        for r in robots:
            num_arrived_cars += int(r.arrived)

        is_all_arrived = False
        if len(robots) == num_arrived_cars:
            is_all_arrived = True
        return is_all_arrived

    def __init_cars(self):
        """
        Init cars according to the number of vehicles and initial positions
        :return:
        """
        robot_a = Car(
            start=(25, 25, np.radians(45)),
            end=(40, 40, np.radians(45)),
            speed=10,
            radius=3.6,
            wb=2.3,
            max_speed=[5, -3],
            color="salmon",
            label="RobotA",
        )
        # robot_b = Car(
        #     start=(40, 10, np.radians(180)),
        #     end=(60, 80, np.radians(45)),
        #     speed=10,
        #     radius=3.6,
        #     wb=2.3,
        #     max_speed=[5, -3],
        #     color="teal",
        #     label="RobotB",
        # )
        # robot_c = Car(
        #     start=(30, 80, np.radians(-45)),
        #     end=(80, 30, np.radians(15)),
        #     speed=10,
        #     radius=3.6,
        #     wb=2.3,
        #     max_speed=[5, -3],
        #     color="royalblue",
        #     label="RobotC",
        # )
        robots = [robot_a]  # , robot_b, robot_c]

        # set path from start to goal
        curvature = 1.0 / 10.0
        for r in robots:
            path = DubinsPathPlanner()
            path.generate(r, curvature)
            r.set_path(path.x, path.y, path.yaw)

        return robots
