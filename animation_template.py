import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from robot import Car
from animation import Animation

matplotlib.use("TkAgg")


def acceleration(frame):
    return 40.0 * np.cos(np.radians(5 * frame % 360))


def steering_angle(frame):
    return np.radians(10.0) + 0.2 * np.sin(np.radians(10 * frame % 360))


def main():
    # Define initial and final positions for both robots
    robots = init_robots()

    # setting
    dt = 0.05
    fps = 14
    save_gif = False

    # init plots
    ani_obj = init_animation_obj(dt, fps, robots, save_gif)

    def init():
        for robot_a in robots:
            plot_animation_obj(ani_obj, robot_a)
        return ani_obj.ax.patches

    def update(frame):
        """"move robots and update plots"""
        for robot_a in robots:
            u = [acceleration(frame), steering_angle(frame)]
            robot_a.move(u, dt=dt)

            update_animation_obj(ani_obj, robot_a, u)

        return ani_obj.ax.patches

    # Start animation
    ani = FuncAnimation(
        ani_obj.fig, update, frames=100, init_func=init, blit=False, interval=1000.0 / fps
    )

    if save_gif:
        write_gif(ani, ani_obj, fps)
    else:
        plt.show()


def init_robots():
    robotA = Car(start=(10, 10), end=(30, 30),
                 speed=10, radius=3.6, wb=2.3, max_speed=[10, -3],
                 color="red", label='RobotA')
    robotB = Car(start=(30, 30), end=(10, 10),
                 speed=10, radius=3.6, wb=2.3, max_speed=[10, -3],
                 color="green", label='RobotB')
    robots = [robotA, robotB]  # , robotC, robotD, robotE]
    return robots


def init_animation_obj(dt, fps, robots, save_gif):
    ani_obj = Animation(dt, fps, save_gif)
    ani_obj.init_points(robots)
    ani_obj.init_trajs(robots)
    ani_obj.init_quivers(robots)
    ani_obj.init_robots(robots)
    ani_obj.init_ax()
    return ani_obj


def plot_animation_obj(ani_obj, robot_a):
    ani_obj.update_points(robot_a)
    ani_obj.update_quivers(robot_a)
    ani_obj.plot_start_point(robot_a)
    ani_obj.plot_end_point(robot_a)
    ani_obj.update_robots(robot_a)


def update_animation_obj(ani_obj, robot_a, u):
    ani_obj.update_points(robot_a)
    ani_obj.update_quivers(robot_a)
    ani_obj.update_trajs(robot_a)
    ani_obj.update_robots(robot_a, steer=u[1])


def write_gif(ani, ani_obj, fps):
    writer_gif = PillowWriter(fps=fps)
    gif_name = "anime/vo.gif"
    writer_gif.setup(ani_obj.fig, gif_name, dpi=300)
    ani.save(gif_name, writer=writer_gif)



if __name__ == "__main__":
    main()
