import sys
import numpy as np
from lib.robot_mod import Car, Robot
from lib.vo import VelocityObstacle
from lib.path_planner import DubinsPathPlanner
from lib.motion_controller import *

from pygame_animation import Screen

def main_robots():
    screen = Screen(scale=20)
    robots = init_robots()

    run = True
    frame = 0
    dt = 0.05

    # Init VOs
    is_rvo = True
    margin = 0.0
    t_h = 1
    vos = {}
    vo = init_vos(is_rvo, robots, t_h, vos)

    trajectories = {}
    for r in robots:
        trajectories[r] = []

    while run:
        run = screen.is_quit_event()

        screen.clock.tick(30)  # 60 frames per second

        # update all VOs
        vo_unions = update_vo_union(margin, robots, vos)

        # create inputs
        for robot_a in robots:
            vo_union = vo_unions[robot_a.label]
            v = vo.desired_velocity_ma(
                robot_a.position,
                robot_a.velocity,
                robot_a.goal,
                vo_union,
                max_speed=robot_a.max_speed,
            )
            robot_a.move(v, dt=dt)

            # update trajectories
            trajectories[robot_a].append((robot_a.position[0], robot_a.position[1]))

        # draw
        screen.update_robots(robots, trajectories)

        frame += 1

    screen.quit()
    sys.exit()


def init_vos(is_rvo, robots, t_h, vos):
    vo = None
    for robot_a in robots:
        other_robots = [x for x in robots if x != robot_a]
        for robot_b in other_robots:
            vo = VelocityObstacle(
                radius_A=robot_a.radius,
                radius_B=robot_b.radius,
                time_horizon=t_h,
                rvo=is_rvo,
            )
            vos[(robot_a.label, robot_b.label)] = vo
    return vo


def update_vo_union(margin, robots, vos):
    vo_unions = {}
    for robot_a in robots:
        vo_unions[robot_a.label] = []
        _pA = robot_a.position
        _vA = robot_a.velocity
        _other_robots = [x for x in robots if x != robot_a]

        for robot_b in _other_robots:
            _pB = robot_b.position
            _vB = robot_b.velocity

            # compute vo
            _vo = vos[(robot_a.label, robot_b.label)]
            _tri = _vo.compute_vo_triangle(margin, _pA, _vA, _pB, _vB)
            if _tri is None:
                _tri = [0, 0, 0]

            # keep vo
            vo_unions[robot_a.label].append(_tri)
    return vo_unions


def init_robots():
    robot_a = Robot(
        start=(10, 10),
        goal=(20, 20),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="red",
        label="RobotA",
    )
    robot_b = Robot(
        start=(20, 12),
        goal=(12, 15),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="green",
        label="RobotB",
    )
    robot_c = Robot(
        start=(10, 14),
        goal=(18, 15),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="blue",
        label="RobotC",
    )
    robot_d = Robot(
        start=(10, 20),
        goal=(15, 10),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="orange",
        label="RobotD",
    )
    robot_e = Robot(
        start=(16, 20),
        goal=(18, 12),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="black",
        label="RobotE",
    )
    robots = [robot_a, robot_b, robot_c, robot_d, robot_e]
    return robots



if __name__ == "__main__":
    main_robots()
