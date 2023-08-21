import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.patches as patches

from vo import VelocityObstacle
from robot import Robot

matplotlib.use('TkAgg')

# Define Robot class for convenience

def main():

    # Define initial and final positions for both robots
    robotA = Robot(start=(0, 0), end=(10, 0), speed=0.10, radius=1.5)
    robotB = Robot(start=(10, 0), end=(0, 0), speed=0.10, radius=1.6)
    # robotA = Robot(start=(0, 10), end=(10, 0), speed=0.10, radius=1.5)
    # robotB = Robot(start=(0, 0), end=(10, 10), speed=0.10, radius=1.0)
    # robotA = Robot(start=(0, 5), end=(10, 0), speed=0.12, radius=1.5)
    # robotB = Robot(start=(0, 0), end=(10, 4), speed=0.10, radius=1.0)

    fig, ax = plt.subplots()
    ax.set_ylim((-5, 15))
    ax.set_xlim((-5, 15))
    fps = 24

    # robots
    rA = robotA.radius
    rB = robotB.radius
    pA = robotA.position
    pB = robotB.position
    vA = robotA.velocity
    vB = robotB.velocity

    t_hori = 1
    vo = VelocityObstacle(radius_A=rA, radius_B=rB, time_horizon=t_hori)

    # Robots
    pointA = patches.Circle((pA[0], pA[1]), rA, fill=True, color='lightgreen', alpha=0.5, label='Robot A')
    pointB = patches.Circle((pB[0], pB[1]), rB, fill=True, color='blue', alpha=0.5, label='Robot B')
    point_minsum = patches.Circle((pB[0], pB[1]), rB + rA, fill=True, color='skyblue', alpha=0.5, label='Minkowski Sum')

    # trajectory
    trajA, = ax.plot([],[],'--')
    trajA_x = []
    trajA_y = []

    # Arrows
    quiverA = ax.quiver(pA[0], pA[1], vA[0], vA[1], angles='xy', scale_units='xy', scale=1, color='red')
    quiverB = ax.quiver(pB[0], pB[1], vB[0], vB[1], angles='xy', scale_units='xy', scale=1, color='red')

    # VO
    triangle_coords = np.array([[1, 1], [2, 2.5], [3, 1]])
    triangle = patches.Polygon(triangle_coords, alpha=0.5, closed=True, color='gray')

    ax.add_patch(pointA)
    ax.add_patch(pointB)
    ax.add_patch(point_minsum)
    ax.add_patch(triangle)

    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05),
              ncol=3, fancybox=True, shadow=True)

    ax.set_aspect('equal')

    def init():
        # robot
        pointA.center = (robotA.position[0], robotA.position[1])
        pointB.center = (robotB.position[0], robotB.position[1])
        point_minsum.center = (robotB.position[0], robotB.position[1])

        # Plot starting points
        ax.plot(robotA.position[0], robotA.position[1], 'ro', markersize=15, fillstyle='none', label='Start A')
        ax.plot(robotB.position[0], robotB.position[1], 'bo', markersize=15, fillstyle='none', label='Start B')

        # Plot ending points
        ax.plot(robotA.end[0], robotA.end[1], 'ro', markersize=15, fillstyle='none', label='End A')
        ax.plot(robotB.end[0], robotB.end[1], 'bo', markersize=15, fillstyle='none', label='End B')

        # VO
        pA = robotA.position
        pB = robotB.position
        vA = robotA.velocity
        vB = robotB.velocity

        tangent_point1, tangent_point2 = vo.tangent_points(pB, vo.rA + vo.rB, pA)
        triangle_coords = [pA + vB * t_hori, tangent_point1 + vB * t_hori, tangent_point2 + vB * t_hori]
        triangle.set_xy(triangle_coords)

        quiverA.set_offsets(pA)
        quiverA.set_UVC(vA[0], vA[1])
        quiverB.set_offsets(pB)
        quiverB.set_UVC(vA[0], vB[1])


        return pointA, pointB, point_minsum, triangle, quiverA, quiverB, trajA

    def update(frame):

        pA = robotA.position
        pB = robotB.position
        pG = robotA.end
        vA = robotA.velocity
        vB = robotB.velocity

        # update VO
        # _, combined_radius, relative_velocity = vo.compute_vo_parameters(pA, vA, pB, vB)
        combined_radius = vo.rA + vo.rB
        tangent_point1, tangent_point2 = vo.tangent_points(pB, combined_radius, pA)
        if tangent_point1 is None and tangent_point2 is None:
            triangle_coords = [(0,0),(0,0),(0,0)]
        else:
            margin = 0.0
            tangent_point1 += vB * t_hori + margin * (tangent_point1 - pB)
            tangent_point2 += vB * t_hori + margin * (tangent_point2 - pB)

            A = pA + vB * t_hori
            B = tangent_point1 + 1000000 * (tangent_point1 - A)
            C = tangent_point2 + 1000000 * (tangent_point2 - A)
            triangle_coords = [A, B, C]
        triangle.set_xy(triangle_coords)

        n_vA = vo.desired_velocity(pA, vA, pB, vB, pG=pG, max_speed=robotA.speed)
        # vo.draw(pA, vA, pB, vB)
        print(vA, n_vA)

        robotA.move(n_vA)
        # robotA.move()
        robotB.move()
        pointA.center = (robotA.position[0], robotA.position[1])
        pointB.center = (robotB.position[0], robotB.position[1])
        point_minsum.center = (robotB.position[0], robotB.position[1])

        quiverA.set_offsets(pA)
        quiverA.set_UVC(vA[0], vA[1])
        quiverB.set_offsets(pB)
        quiverB.set_UVC(vB[0], vB[1])

        trajA_x.append(pA[0])
        trajA_y.append(pA[1])
        trajA.set_data(trajA_x, trajA_y)


        return pointA, pointB, point_minsum, triangle, quiverA, quiverB, trajA

    ani = FuncAnimation(fig, update, frames=range(300), init_func=init, blit=True, interval=1000./fps)
    plt.show()

    # writergif = PillowWriter(fps=30)
    # gif_name = "anime/vo.gif"
    # writergif.setup(fig, gif_name, dpi=300)
    # ani.save(gif_name, writer=writergif)

if __name__ == '__main__':
    main()