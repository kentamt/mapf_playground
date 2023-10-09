import sys
import numpy as np
from robot import Car, Robot
from vo import VelocityObstacle
from pygame_animation import Screen


def acceleration(frame):
    VELOCITY = 40  # [m/s]
    return VELOCITY * np.cos(np.radians(5 * frame % 360))


def steering_angle(frame):
    STEER_ANG_DEG = 10  # [deg]
    return np.radians(STEER_ANG_DEG) + 0.3 * np.sin(np.radians(3 * frame % 360))


def main_car():
    screen = Screen()
    robots = init_cars()

    run = True
    frame = 0
    dt = 0.01

    trajectories = {}
    for r in robots:
        trajectories[r.label] = []

    while run:
        run = screen.is_quit_event()

        screen.clock.tick(60)  # 60 frames per second

        # create inputs
        for robot_a in robots:
            u = [acceleration(frame), steering_angle(frame)]
            robot_a.move(u, dt=dt)

            # update trajectories
            trajectories[robot_a.label].append(
                (robot_a.position[0], robot_a.position[1])
            )

        # draw
        screen.update_cars(robots, trajectories)

        frame += 1

    pygame.quit()
    sys.exit()


def main_robots():
    screen = Screen()
    robots = init_robots()

    run = True
    frame = 0
    dt = 0.05

    # Init VOs
    is_rvo = True
    margin = 0.0
    t_hori = 1
    vos = {}
    vo = init_vos(is_rvo, robots, t_hori, vos)

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
                robot_a.end,
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


def init_vos(is_rvo, robots, t_hori, vos):
    for robot_a in robots:
        other_robots = [x for x in robots if x != robot_a]
        for robot_b in other_robots:
            vo = VelocityObstacle(
                radius_A=robot_a.radius,
                radius_B=robot_b.radius,
                time_horizon=t_hori,
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
    robotA = Robot(
        start=(10, 10),
        end=(20, 20),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="red",
        label="RobotA",
    )
    robotB = Robot(
        start=(20, 12),
        end=(12, 15),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="green",
        label="RobotB",
    )
    robotC = Robot(
        start=(10, 14),
        end=(18, 15),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="blue",
        label="RobotC",
    )
    robotD = Robot(
        start=(10, 20),
        end=(15, 10),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="orange",
        label="RobotD",
    )
    robotE = Robot(
        start=(16, 20),
        end=(18, 12),
        speed=1,
        max_speed=5.0,
        radius=1.0,
        color="black",
        label="RobotE",
    )
    robots = [robotA, robotB, robotC, robotD, robotE]
    return robots


def init_cars():
    robotA = Car(
        start=(10, 10),
        end=(30, 30),
        speed=10,
        radius=3.6,
        wb=2.3,
        max_speed=[10, -3],
        color="red",
        label="RobotA",
    )
    robotB = Car(
        start=(30, 30),
        end=(10, 10),
        speed=10,
        radius=3.6,
        wb=2.3,
        max_speed=[10, -3],
        color="green",
        label="RobotB",
    )
    robotC = Car(
        start=(10, 30),
        end=(30, 10),
        speed=10,
        radius=3.6,
        wb=2.3,
        max_speed=[10, -3],
        color="blue",
        label="RobotC",
    )
    robots = [robotA, robotB, robotC]
    return robots


if __name__ == "__main__":
    main_car()
    # main_robots()
