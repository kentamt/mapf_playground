import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.patches as patches

from vo import VelocityObstacle
from robot import Robot

matplotlib.use("TkAgg")


def main():
    # Define initial and final positions for both robots
    robotA = Robot(start=(0, 0), end=(10, 0), speed=0.30, radius=1.5, color="red")
    robotB = Robot(start=(10, 0), end=(0, 0), speed=0.3, radius=1.6, color="green")
    # robotA = Robot(start=(0, 10), end=(10, 0), speed=0.30, radius=1.5, color="red")
    # robotB = Robot(start=(0, 0), end=(10, 10), speed=0.30, radius=1.0, color="green")
    # robotA = Robot(start=(0, 5), end=(10, 0), speed=0.20, radius=1.5, color="red")
    # robotB = Robot(start=(0, 0), end=(10, 4), speed=0.20, radius=1.0, color="green")

    fig, ax = plt.subplots()
    save_gif = False
    fps = 10

    # robots
    rA = robotA.radius
    rB = robotB.radius
    pA = robotA.position
    pB = robotB.position
    vA = robotA.velocity
    vB = robotB.velocity

    pointA = patches.Circle(
        (pA[0], pA[1]), rA, fill=True, color=robotA.color, alpha=0.5, label="Robot A"
    )
    pointB = patches.Circle(
        (pB[0], pB[1]), rB, fill=True, color=robotB.color, alpha=0.5, label="Robot B"
    )
    point_minsum = patches.Circle(
        (pB[0], pB[1]),
        rB + rA,
        fill=True,
        color="skyblue",
        alpha=0.5,
        label="Minkowski Sum",
    )

    # trajectory
    (trajA,) = ax.plot([], [], "--", color=robotA.color)
    (trajB,) = ax.plot([], [], "--", color=robotB.color)
    trajA_x = []
    trajA_y = []
    trajB_x = []
    trajB_y = []

    # Arrows
    quiverA = ax.quiver(
        pA[0], pA[1], vA[0], vA[1], angles="xy", scale_units="xy", scale=1, color="red"
    )
    quiverB = ax.quiver(
        pB[0], pB[1], vB[0], vB[1], angles="xy", scale_units="xy", scale=1, color="red"
    )

    # VO
    t_hori = 1
    vo = VelocityObstacle(radius_A=rA, radius_B=rB, time_horizon=t_hori, rvo=False)
    triangle_coords = np.array([[1, 1], [2, 2.5], [3, 1]])
    triangle = patches.Polygon(triangle_coords, alpha=0.5, closed=True, color="gray")

    ax.add_patch(pointA)
    ax.add_patch(pointB)
    ax.add_patch(point_minsum)
    ax.add_patch(triangle)
    ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, 1.05),
        ncol=3,
        fancybox=True,
        shadow=True,
    )
    ax.set_aspect("equal")

    ax.set_ylim((-5, 15))
    ax.set_ylim((-5, 5))

    def init():
        _pA = robotA.position
        _pB = robotB.position
        _vA = robotA.velocity
        _vB = robotB.velocity

        # robot
        pointA.center = (_pA[0], _pB[1])
        pointB.center = (_pB[0], _pB[1])
        point_minsum.center = (_pB[0], _pB[1])

        # Plot starting points
        ax.plot(
            _pA[0],
            _pA[1],
            "o",
            color=robotA.color,
            markersize=15,
            fillstyle="none",
            label="Start A",
        )
        ax.plot(
            _pB[0],
            _pB[1],
            "o",
            color=robotB.color,
            markersize=15,
            fillstyle="none",
            label="Start B",
        )

        # Plot ending points
        ax.plot(
            robotA.end[0],
            robotA.end[1],
            "o",
            color=robotA.color,
            markersize=15,
            fillstyle="none",
            label="End A",
        )
        ax.plot(
            robotB.end[0],
            robotB.end[1],
            "o",
            color=robotB.color,
            markersize=15,
            fillstyle="none",
            label="End B",
        )

        # VO
        quiverA.set_offsets(_pA)
        quiverA.set_UVC(_vA[0], _vA[1])
        quiverB.set_offsets(_pB)
        quiverB.set_UVC(_vA[0], _vB[1])

        tp1, tp2 = vo.tangent_points(_pB, vo.rA + vo.rB, _pA)
        _A = _pA + _vB * t_hori
        _B = tp1 + _vB * t_hori
        _C = tp2 + _vB * t_hori
        _tri = [_A, _B, _C]
        triangle.set_xy(_tri)

        return pointA, pointB, point_minsum, triangle, quiverA, quiverB, trajA, trajB

    def update(frame):

        _pA = robotA.position
        _pB = robotB.position
        _pGA = robotA.end
        _pGB = robotB.end
        _vA = robotA.velocity
        _vB = robotB.velocity

        _tri = vo.compute_vo_triangle(0.0, _pA, _vA, _pB, _vB)
        if _tri is None:
            _tri = [0, 0, 0]
        triangle.set_xy(_tri)

        # update robots
        pointA.center = (_pA[0], _pA[1])
        pointB.center = (_pB[0], _pB[1])
        point_minsum.center = (_pB[0], _pB[1])

        quiverA.set_offsets(_pA)
        quiverA.set_UVC(_vA[0], _vA[1])
        quiverB.set_offsets(_pB)
        quiverB.set_UVC(_vB[0], _vB[1])

        trajA_x.append(_pA[0])
        trajA_y.append(_pA[1])
        trajA.set_data(trajA_x, trajA_y)
        trajB_x.append(_pB[0])
        trajB_y.append(_pB[1])
        trajB.set_data(trajB_x, trajB_y)

        # move
        _n_vA = vo.desired_velocity(
            _pA, _vA, _pB, _vB, pG=_pGA, max_speed=robotA.max_speed
        )
        _n_vB = vo.desired_velocity(
            _pB, _vB, _pA, _vA, pG=_pGB, max_speed=robotB.max_speed
        )
        robotA.move(_n_vA)
        robotB.move(_n_vB)

        return pointA, pointB, point_minsum, triangle, quiverA, quiverB, trajA, trajB

    ani = FuncAnimation(
        fig, update, frames=range(300), init_func=init, blit=True, interval=1000.0 / fps
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
