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
    # robotA = Robot(start=(0, 0), end=(15, 0), speed=0.5, radius=0.5, color="red", label='RobotA')
    # robotB = Robot(start=(0, 2), end=(15, 2), speed=0.5, radius=0.5, color="green", label='RobotB')
    # robotC = Robot(start=(0, 4), end=(15, 4), speed=0.5, radius=0.5, color="blue", label='RobotC')
    # robotD = Robot(start=(15, 0), end=(0, 0), speed=0.5, radius=0.5, color="orange", label='RobotD')
    # robotE = Robot(start=(15, 2), end=(0, 2), speed=0.5, radius=0.5, color="black", label='RobotE')
    # robotF = Robot(start=(15, 4), end=(0, 4), speed=0.5, radius=0.5, color="pink", label='RobotF')
    # robots = [robotA, robotB, robotC, robotD, robotE, robotF]
    robotA = Robot(start=(0, 0), end=(10,10), speed=0.2, radius=1.0, color="red", label='RobotA')
    robotB = Robot(start=(10, 2), end=(2, 5), speed=0.2, radius=1.0, color="green", label='RobotB')
    robotC = Robot(start=(0, 4), end=(8, 5), speed=0.2, radius=1.0, color="blue", label='RobotC')
    robotD = Robot(start=(0, 10), end=(5, 0), speed=0.2, radius=1.0, color="orange", label='RobotD')
    robotE = Robot(start=(6, 10), end=(8, 0), speed=0.2, radius=1.0, color="black", label='RobotE')
    robots = [robotA, robotB, robotC, robotD, robotE]

    fig, ax = plt.subplots()
    save_gif = True
    fps = 10

    # robots
    points = {}
    for r in robots:
        point = patches.Circle(
            (r.position[0], r.position[1]), r.radius, fill=True, color=r.color, alpha=0.5, label=r.label)
        points[r.label] = point

    # trajectory
    trajs = {}
    for r in robots:
        (trajA,) = ax.plot([], [], "--", color=r.color)
        trajs[r.label] = {}
        trajs[r.label]['traj'] = trajA
        trajs[r.label]['x'] = []
        trajs[r.label]['y'] = []

    # Arrows
    quivers = {}
    for r in robots:
        quiver = ax.quiver(
            r.position[0], r.position[1], r.velocity[0], r.velocity[1], angles="xy", scale_units="xy", scale=1, color="red"
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
                                  time_horizon=t_hori, rvo=True)
            vos[(robot_a.label, robot_b.label)] = vo

            triangle_coords = np.array([[1, 1], [2, 2.5], [3, 1]])
            triangle = patches.Polygon(triangle_coords, alpha=0.1, closed=True, color=robot_a.color)
            triangles[(robot_a.label, robot_b.label)] = triangle

    for point in points.values():
        ax.add_patch(point)

    # for triangle in triangles.values():
    #     ax.add_patch(triangle)

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
            _p = robot_a.position
            _v = robot_a.velocity

            # robot
            points[robot_a.label].center = (_p[0], _p[1])

            # Plot starting points
            ax.plot(
                _p[0],
                _p[1],
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

            quivers[robot_a.label].set_offsets(_p)
            quivers[robot_a.label].set_UVC(_v[0], _v[1])

            # VO
            other_robots = [x for x in robots if x != robot_a]
            for robot_b in other_robots:
                vo = vos[(robot_a.label, robot_b.label)]
                _pA = robot_a.position
                _pB = robot_b.position
                _vA = robot_a.velocity
                _vB = robot_b.velocity

                tp1, tp2 = vo.tangent_points(_pB, vo.rA + vo.rB, _pA)
                _A = _pA + _vB * t_hori
                _B = tp1 + _vB * t_hori
                _C = tp2 + _vB * t_hori
                _tri = [_A, _B, _C]
                triangle = triangles[(robot_a.label, robot_b.label)]
                triangle.set_xy(_tri)

        ret = []
        for point in points.values():
            ret.append(point)
        for tri in triangles.values():
            ret.append(tri)
        for qui in quivers.values():
            ret.append(qui)
        for traj in trajs.values():
            ret.append(traj['traj'])
        # print('end of init')
        return ret  # points, triangles, quivers, trajs

    def update(frame):
        print('update robots')

        vo_unions = {}
        for robot_a in robots:
            vo_unions[robot_a.label] = []
            _pA = robot_a.position
            _pGA = robot_a.end
            _vA = robot_a.velocity

            points[robot_a.label].center = (_pA[0], _pA[1])
            quivers[robot_a.label].set_offsets(_pA)
            quivers[robot_a.label].set_UVC(_vA[0], _vA[1])

            trajs[robot_a.label]['x'].append(_pA[0])
            trajs[robot_a.label]['y'].append(_pA[1])
            trajs[robot_a.label]['traj'].set_data(trajs[robot_a.label]['x'], trajs[robot_a.label]['y'])

            other_robots = [x for x in robots if x != robot_a]
            for robot_b in other_robots:
                _pB = robot_b.position
                _pGB = robot_b.end
                _vB = robot_b.velocity

                vo = vos[(robot_a.label, robot_b.label)]
                _tri = vo.compute_vo_triangle(0.0, _pA, _vA, _pB, _vB)
                if _tri is None:
                    _tri = [0, 0, 0]
                triangle = triangles[(robot_a.label, robot_b.label)]
                triangle.set_xy(_tri)

                vo_unions[robot_a.label].append(_tri)

        # chose desired velocity considering all of vos
        for robot_a in robots:
            vo_union = vo_unions[robot_a.label]
            _pA = robot_a.position
            _pGA = robot_a.end
            _vA = robot_a.velocity
            _n_vA = vo.desired_velocity_ma(_pA, _vA, _pGA, vo_union, max_speed=robot_a.speed)
            robot_a.move(_n_vA)

        ret = []
        for point in points.values():
            ret.append(point)
        for key, tri in triangles.items():
            ret.append(tri)
        for qui in quivers.values():
            ret.append(qui)
        for traj in trajs.values():
            ret.append(traj['traj'])

        return ret  # points, triangles, quivers, trajs

        # return points, triangles, quivers, trajs

    ani = FuncAnimation(
        fig, update, frames=range(300), init_func=init, blit=False, interval=1000.0 / fps
    )
    if save_gif:
        writergif = PillowWriter(fps=30)
        gif_name = "anime/vo.gif"
        writergif.setup(fig, gif_name, dpi=300)
        ani.save(gif_name, writer=writergif)
    else:
        plt.show()

if __name__ == "__main__":
    main()
