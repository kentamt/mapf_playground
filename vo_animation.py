import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

from vo import VelocityObstacle

matplotlib.use('TkAgg')

# Define Robot class for convenience
class Robot:
    def __init__(self, start, end, speed, radius):
        self._position = np.array(start, dtype=np.float64)
        self.end = np.array(end, dtype=np.float64)
        self.direction = self.end - self._position
        self.default_velocity = (self.direction / np.linalg.norm(self.direction)) * speed
        self.current_velocity = self.default_velocity
        self.speed = speed
        self.radius = radius
        self.moving = True  # Add a flag to indicate if the robot is moving

    def move(self, velocity=None):
        if not self.moving:  # If the robot isn't moving, do nothing
            return

        if velocity is None:
            self.current_velocity = self.current_velocity
        else:
            self.current_velocity = velocity

        remaining_distance = np.linalg.norm(self.end - self._position)
        if remaining_distance <= self.speed:  # Check if we're close to the end point
            self._position = self.end
            self.moving = False
        else:
            self._position += self.current_velocity

    @property
    def position(self):
        return self._position

    @property
    def velocity(self):
        return self.current_velocity

def main():

    # Define initial and final positions for both robots
    robotA = Robot(start=(0, 0), end=(30, 10), speed=0.30, radius=2.0)
    robotB = Robot(start=(0, 10), end=(30, 0), speed=0.20, radius=2.0)

    fig, ax = plt.subplots()

    # robots
    rA = robotA.radius
    rB = robotB.radius
    pA = robotA.position
    pB = robotB.position
    vA = robotA.velocity
    vB = robotB.velocity

    t_hori = 5
    vo = VelocityObstacle(radius_A=rA, radius_B=rB, time_horizon=1)

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
        circle_center, combined_radius, relative_velocity = vo.compute_vo_parameters(pA, vA, pB, vB)
        tangent_point1, tangent_point2 = vo.tangent_points(pB, combined_radius, pA)
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
        circle_center, combined_radius, relative_velocity = vo.compute_vo_parameters(pA, vA, pB, vB)
        tangent_point1, tangent_point2 = vo.tangent_points(pB, combined_radius, pA)
        triangle_coords = [pA + vB * t_hori, tangent_point1 + vB * t_hori, tangent_point2 + vB * t_hori]
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


    fps = 5
    ani = FuncAnimation(fig, update, frames=range(300), init_func=init, blit=True, interval=1000./fps)
    plt.show()
    # ani.save("vo_with_bug.gif", writer='imagemagick', fps=60)

if __name__ == '__main__':
    main()