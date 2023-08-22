import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.patches as patches

from vo import VelocityObstacle
from robot import Robot

from itertools import combinations

matplotlib.use("TkAgg")


def main():
    # Define initial and final positions for both robots
    # robotA = Robot(start=(0, 0), end=(15, 0), speed=1, radius=0.5, color="red", label='RobotA')
    # robotB = Robot(start=(0, 2), end=(15, 2), speed=1, radius=0.5, color="green", label='RobotB')
    # robotC = Robot(start=(0, 4), end=(15, 4), speed=1, radius=0.5, color="blue", label='RobotC')
    # robotD = Robot(start=(15, 0), end=(0, 0), speed=1, radius=0.5, color="orange", label='RobotD')
    # robotE = Robot(start=(15, 2), end=(0, 2), speed=1, radius=0.5, color="black", label='RobotE')
    # robotF = Robot(start=(15, 4), end=(0, 4), speed=1, radius=0.5, color="pink", label='RobotF')
    # robots = [robotA, robotB, robotC, robotD, robotE, robotF]
    robotA = Robot(start=(0, 0), end=(10, 10), speed=3, radius=1.0, color="red", label='RobotA')
    robotB = Robot(start=(10, 2), end=(2, 5), speed=3, radius=1.0, color="green", label='RobotB')
    robotC = Robot(start=(0, 4), end=(8, 5), speed=3, radius=1.0, color="blue", label='RobotC')
    robotD = Robot(start=(0, 10), end=(5, 0), speed=3, radius=1.0, color="orange", label='RobotD')
    robotE = Robot(start=(6, 10), end=(8, 0), speed=3, radius=1.0, color="black", label='RobotE')
    robots = [robotA, robotB] # , robotC, robotD, robotE]


    fig, ax = plt.subplots()
    save_gif = False
    fps = 24
    draw_vo = False
    is_rvo = True
    margin = 0.2
    dt = 0.1

    # robots
    points = {}
    for r in robots:
        point = patches.Circle(
            (r.position[0], r.position[1]), r.radius, fill=True, color=r.color, alpha=0.5, label=r.label)
        points[r.label] = point

    # trajectory
    trajs = {}
    for r in robots:
        (trajA,) = ax.plot([], [], "-", linewidth=0.8, color=r.color)
        trajs[r.label] = {}
        trajs[r.label]['traj'] = trajA
        trajs[r.label]['x'] = []
        trajs[r.label]['y'] = []

    # Arrows
    quivers = {}
    for r in robots:
        quiver = ax.quiver(
            r.position[0], r.position[1], r.velocity[0], r.velocity[1], angles="xy", scale_units="xy", scale=1,
            color="red"
        )
        quivers[r.label] = quiver

    # VO
    t_hori = 1
    vos = {}
    triangles = {}
    for robot_a in robots:
        other_robots = [x for x in robots if x != robot_a]
        for robot_b in other_robots:
            vo = VelocityObstacle(radius_A=robot_a.radius,
                                  radius_B=robot_b.radius,
                                  time_horizon=t_hori, rvo=is_rvo)
            vos[(robot_a.label, robot_b.label)] = vo

            triangle_coords = np.array([[1, 1], [2, 2.5], [3, 1]])
            triangle = patches.Polygon(triangle_coords, alpha=0.1, closed=True, color=robot_a.color)
            triangles[(robot_a.label, robot_b.label)] = triangle

    for point in points.values():
        ax.add_patch(point)

    if draw_vo:
        for triangle in triangles.values():
            ax.add_patch(triangle)

    ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, 1.05),
        ncol=3,
        fancybox=True,
        shadow=True,
    )
    ax.set_aspect("equal")
    ax.set_ylim((-15, 15))
    ax.set_ylim((-5, 15))

    def init():
        print('start init')
        for robot_a in robots:

            # robot
            __update_points(robot_a)
            __update_quivers(robot_a)

            # Plot starting points
            ax.plot(
                robot_a.position[0],
                robot_a.position[1],
                "o",
                color=robot_a.color,
                markersize=15,
                fillstyle="none",
                label=f"Start {robot_a.label}",
            )

            # Plot ending points
            ax.plot(
                robot_a.end[0],
                robot_a.end[1],
                "o",
                color=robot_a.color,
                markersize=15,
                fillstyle="none",
                label=f"End {robot_a.label}",
            )

            # VO
            other_robots = [x for x in robots if x != robot_a]
            for robot_b in other_robots:
                _vo = vos[(robot_a.label, robot_b.label)]
                _tri = _vo.compute_vo_triangle(margin,
                                               robot_a.position, robot_a.velocity,
                                               robot_b.position, robot_b.velocity)
                __update_triangles(_tri, robot_a, robot_b)

        return __plot_list()

    def update(frame):
        print('update robots')


        # all_robots_stop = True
        # for robot_a in robots:
        #     if robot_a.moving:
        #         all_robots_stop *= False
        #
        # if all_robots_stop:
        #     plt.close(fig)


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

                # update triangle
                __update_triangles(_tri, robot_a, robot_b)

            # update plots
            __update_points(robot_a)
            __update_quivers(robot_a)
            __update_trajs(robot_a)

        # choose desired velocity considering all of vos
        for robot_a in robots:
            vo_union = vo_unions[robot_a.label]
            _n_vA = vo.desired_velocity_ma(robot_a.position,
                                           robot_a.velocity,
                                           robot_a.end,
                                           vo_union, max_speed=robot_a.speed)
            robot_a.move(_n_vA, dt=dt)

        return __plot_list()

    def __plot_list():
        ret = []
        for point in points.values():
            ret.append(point)
        for key, tri in triangles.items():
            ret.append(tri)
        for qui in quivers.values():
            ret.append(qui)
        for traj in trajs.values():
            ret.append(traj['traj'])
        return ret

    def __update_triangles(tri, robot_a, robot_b):
        triangle = triangles[(robot_a.label, robot_b.label)]
        triangle.set_xy(tri)

    def __update_trajs(robot):
        trajs[robot.label]['x'].append(robot.position[0])
        trajs[robot.label]['y'].append(robot.position[1])
        trajs[robot.label]['traj'].set_data(trajs[robot.label]['x'],
                                            trajs[robot.label]['y'])

    def __update_quivers(robot):
        quivers[robot.label].set_offsets(robot.position)
        quivers[robot.label].set_UVC(robot.velocity[0], robot.velocity[1])

    def __update_points(robot):
        points[robot.label].center = (robot.position[0], robot.position[1])

    # Animation
    ani = FuncAnimation(
        fig, update, frames=100, init_func=init, blit=False, interval=1000.0 / fps
    )

    if save_gif:
        writergif = PillowWriter(fps=15)
        gif_name = "anime/vo.gif"
        writergif.setup(fig, gif_name, dpi=300)
        ani.save(gif_name, writer=writergif)
    else:
        plt.show()


if __name__ == "__main__":
    main()
