import sys
from lib.robot_mod import Car
from lib.path_planner import DubinsPathPlanner
from lib.motion_controller import *

from pygame_animation import Screen


def acceleration(frame):
    velocity = 40  # [m/s]
    return velocity  # * np.cos(np.radians(5 * frame % 360))


def steering_angle(frame):
    steer_ang_deg = 10  # [deg]
    return np.radians(steer_ang_deg) + 0.3 * np.sin(np.radians(3 * frame % 360))


def motion_control(r, target_course_dict):
    """
    :param r:
    :param target_course_dict:
    :return acceleration, steering_angle:
    —"""
    t_course = target_course_dict[r.label]['course']
    t_idx = target_course_dict[r.label]['index']
    a = proportional_control(r.max_speed_f, r.state.v)
    delta, t_idx = pure_pursuit_steer_control(r.state, t_course, t_idx)
    target_course_dict['index'] = t_idx
    return a, delta


def is_all_cars_arrived(robots):
    num_arrived_cars = 0
    for r in robots:
        num_arrived_cars += int(r.arrived)
    is_all_arrived = False
    if len(robots) == num_arrived_cars:
        is_all_arrived = True
    return is_all_arrived


def init_target_course(robots):
    target_course_dict = {}
    for r in robots:
        target_course = TargetCourse(r.path_x, r.path_y)
        target_course_dict[r.label] = {}
        target_idx, _ = target_course.search_target_index(r.state)
        target_course_dict[r.label]["course"] = target_course
        target_course_dict[r.label]["index"] = target_idx
    return target_course_dict


def init_cars():
    """
    Init cars
    :return:
    """
    robot_a = Car(
        start=(25, 25, np.radians(45)),
        end=(90, 90, np.radians(0)),
        speed=10,
        radius=3.6,
        wb=2.3,
        max_speed=[5, -3],
        color="salmon",
        label="RobotA",
    )
    robot_b = Car(
        start=(40, 10, np.radians(180)),
        end=(60, 80, np.radians(45)),
        speed=10,
        radius=3.6,
        wb=2.3,
        max_speed=[5, -3],
        color="teal",
        label="RobotB",
    )
    robot_c = Car(
        start=(30, 80, np.radians(-45)),
        end=(80, 30, np.radians(15)),
        speed=10,
        radius=3.6,
        wb=2.3,
        max_speed=[5, -3],
        color="royalblue",
        label="RobotC",
    )
    robots = [robot_a, robot_b, robot_c]

    # set path from start to goal
    curvature = 1.0 / 10.0
    for r in robots:
        path = DubinsPathPlanner()
        path.generate(r, curvature)
        r.set_path(path.x, path.y, path.yaw)

    return robots


def main_car_gui(dt=0.1):
    robots = init_cars()
    target_course_dict = init_target_course(robots)

    screen = Screen()
    while screen.is_quit_event() and not is_all_cars_arrived(robots):

        for r in robots:
            u = motion_control(r, target_course_dict)
            r.move(u, dt=dt)

        screen.update_cars(robots)
        screen.clock.tick(60)  # 60 frames per second

    screen.quit()


if __name__ == "__main__":
    main_car_gui()
