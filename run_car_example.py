import sys
from lib.robot_mod import Car
from lib.path_planner import DubinsPathPlanner
from lib.motion_controller import *

from pygame_animation import Screen

def acceleration(frame):
    velocity = 40  # [m/s]
    return velocity # * np.cos(np.radians(5 * frame % 360))


def steering_angle(frame):
    steer_ang_deg = 10  # [deg]
    return np.radians(steer_ang_deg) + 0.3 * np.sin(np.radians(3 * frame % 360))

def main_car():
    screen = Screen()
    robots = init_cars()

    run = True
    frame = 0
    dt = 0.01

    target_course_dict = {}
    for r in robots:
        target_course = TargetCourse(r.path_x, r.path_y)
        target_course_dict[r.label] = {}
        target_idx, _ = target_course.search_target_index(r.state)
        target_course_dict[r.label]["course"] = target_course
        target_course_dict[r.label]["index"] = target_idx

    trajectories = {}
    for r in robots:
        trajectories[r.label] = []

    while run:
        run = screen.is_quit_event()

        screen.clock.tick(60)  # 60 frames per second

        # create inputs
        for r in robots:

            # motion planning
            t_course = target_course_dict[r.label]['course']
            t_idx = target_course_dict[r.label]['index']
            ai = proportional_control(r.max_speed_f, r.state.v)
            di, t_idx = pure_pursuit_steer_control(r.state, t_course, t_idx)
            target_course_dict['index'] = t_idx

            # update
            u = [ai, di]
            # u = [acceleration(frame), steering_angle(frame)]
            # u = [0, steering_angle(frame)]
            r.move(u, dt=dt)

            # keep trajectory
            trajectories[r.label].append(
                (r.position[0], r.position[1])
            )

        # draw
        screen.update_cars(robots, trajectories)

        frame += 1

    screen.quit()
    sys.exit()



def init_cars():
    robot_a = Car(
        start=(25, 25, np.radians(45)),
        end=(90, 90, np.radians(0)),
        speed=10,
        radius=3.6,
        wb=2.3,
        max_speed=[30, -3],
        color="salmon",
        label="RobotA",
    )
    robot_b = Car(
        start=(40, 10, np.radians(180)),
        end=(60, 80, np.radians(45)),
        speed=10,
        radius=3.6,
        wb=2.3,
        max_speed=[30, -3],
        color="teal",
        label="RobotB",
    )
    robot_c = Car(
        start=(30, 80, np.radians(-45)),
        end=(80, 30, np.radians(15)),
        speed=10,
        radius=3.6,
        wb=2.3,
        max_speed=[30, -3],
        color="royalblue",
        label="RobotC",
    )
    robots = [robot_a, robot_b, robot_c]

    # set path from start to goal
    curvature = 1.0/5.0
    for r in robots:
        path = DubinsPathPlanner()
        path.generate(r, curvature)
        r.set_path(path.x, path.y, path.yaw)

    return robots


if __name__ == "__main__":
    main_car()
    