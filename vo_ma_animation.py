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
    robotA = Robot(start=(0, 0), end=(10,10), speed=0.3, radius=1.5, color="red", label='RobotA')
    robotB = Robot(start=(10, 0), end=(0, 5), speed=0.3, radius=1.5, color="green", label='RobotB')
    robotC = Robot(start=(5, 0), end=(10, 5), speed=0.3, radius=1.5, color="blue", label='RobotC')
    robotD = Robot(start=(0, 10), end=(5, 10), speed=0.3, radius=1.5, color="orange", label='RobotD')
    robots = [robotA, robotB, robotC, robotD]

    fig, ax = plt.subplots()
    save_gif = False
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
            triangle = patches.Polygon(triangle_coords, alpha=0.5, closed=True, color="gray")
            triangles[(robot_a.label, robot_b.label)] = triangle

    for point, triangle in zip(points.values(), triangles.values()):
        ax.add_patch(point)
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
    ax.set_ylim((-15, 15))

    def init():
        print('start init')
        for robotA in robots:
            _p = robotA.position
            _v = robotA.velocity

            # robot
            points[robotA.label].center = (_p[0], _p[1])

            # Plot starting points
            ax.plot(
                _p[0],
                _p[1],
                "o",
                color=robotA.color,
                markersize=15,
                fillstyle="none",
                label=f"Start {robotA.label}",
            )

            # Plot ending points
            ax.plot(
                robotA.end[0],
                robotA.end[1],
                "o",
                color=robotA.color,
                markersize=15,
                fillstyle="none",
                label=f"End {robotA.label}",
            )

            quivers[robotA.label].set_offsets(_p)
            quivers[robotA.label].set_UVC(_v[0], _v[1])

            # VO
            other_robots = [x for x in robots if x != robotA]
            for robotB in other_robots:
                print(robotA, robotB)
                vo = vos[(robotA.label, robotB.label)]
                _pA = robotA.position
                _pB = robotB.position
                _vA = robotA.velocity
                _vB = robotB.velocity

                tp1, tp2 = vo.tangent_points(_pB, vo.rA + vo.rB, _pA)
                _A = _pA + _vB * t_hori
                _B = tp1 + _vB * t_hori
                _C = tp2 + _vB * t_hori
                _tri = [_A, _B, _C]
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
        print('end of init')
        return ret  # points, triangles, quivers, trajs

    def update(frame):
        print('update robots')
        for robotA in robots:

            _pA = robotA.position
            _pGA = robotA.end
            _vA = robotA.velocity

            points[robotA.label].center = (_pA[0], _pA[1])
            quivers[robotA.label].set_offsets(_pA)
            quivers[robotA.label].set_UVC(_vA[0], _vA[1])

            trajs[robotA.label]['x'].append(_pA[0])
            trajs[robotA.label]['y'].append(_pA[1])
            trajs[robotA.label]['traj'].set_data(trajs[robotA.label]['x'], trajs[robotA.label]['y'])

            other_robots = [x for x in robots if x != robotA]
            for robotB in other_robots:
                print(robotA, robotB)
                _pB = robotB.position
                _pGB = robotB.end
                _vB = robotB.velocity

                # vo = vos[(robotA.label, robotB.label)]
                # _tri = vo.compute_vo_triangle(0.0, _pA, _vA, _pB, _vB)
                # if _tri is None:
                #     _tri = [0, 0, 0]
                # triangle = triangles[(robotA.label, robotB.label)]
                # triangle.set_xy(_tri)

        # chose desired velocity considering all of vos
        for robotA in robots:
            # move
            robotA.move()
            # _n_vA = vo.desired_velocity(_pA, _vA, _pB, _vB, pG=_pGA, max_speed=robotA.speed)
            # _n_vB = vo.desired_velocity(_pB, _vB, _pA, _vA, pG=_pGB, max_speed=robotB.speed)
            # robotA.move(_n_vA)
            # robotB.move(_n_vB)
        ret = []
        for point in points.values():
            ret.append(point)
        for tri in triangles.values():
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
