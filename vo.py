import math
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import matplotlib.patches as patches

# Warning
# matplotlib.use('TkAgg')

LARGE_NUM = 1000000

class VelocityObstacle:
    def __init__(self, radius_A, radius_B, time_horizon):
        self.rA = radius_A
        self.rB = radius_B
        self.t_hori = time_horizon

    @staticmethod
    def do_intersect(P, Q, A, B):
        # Line PQ represented as a1x + b1y = c1
        a1 = Q[1] - P[1]
        b1 = P[0] - Q[0]
        c1 = a1 * P[0] + b1 * P[1]

        # Line AB represented as a2x + b2y = c2
        a2 = B[1] - A[1]
        b2 = A[0] - B[0]
        c2 = a2 * A[0] + b2 * A[1]

        determinant = a1 * b2 - a2 * b1

        if determinant == 0:
            # Lines are parallel
            return False

        # Find intersection point (x, y)
        x = (b2 * c1 - b1 * c2) / determinant
        y = (a1 * c2 - a2 * c1) / determinant

        # Check if the intersection point lies on segment PQ
        if ((x - P[0]) / (Q[0] - P[0]) >= 0 and (x - P[0]) / (Q[0] - P[0]) <= 1) and (
                (y - P[1]) / (Q[1] - P[1]) >= 0 and (y - P[1]) / (Q[1] - P[1]) <= 1):
            # Check if the intersection point is in the direction of the ray
            if (x - A[0]) / (B[0] - A[0]) >= 0 and (y - A[1]) / (B[1] - A[1]) >= 0:
                return True
        return False

    @staticmethod
    def in_triangle(triangle, point):
        """ check if the point is inside the triangle """

        x1 = triangle[0][0]
        y1 = triangle[0][1]
        x2 = triangle[1][0]
        y2 = triangle[1][1]
        x3 = triangle[2][0]
        y3 = triangle[2][1]
        xp = point[0]
        yp = point[1]

        c1 = (x2 - x1) * (yp - y1) - (y2 - y1) * (xp - x1)
        c2 = (x3 - x2) * (yp - y2) - (y3 - y2) * (xp - x2)
        c3 = (x1 - x3) * (yp - y3) - (y1 - y3) * (xp - x3)
        if (c1 < 0 and c2 < 0 and c3 < 0) or (c1 > 0 and c2 > 0 and c3 > 0):
            return True
        else:
            return False

    def desired_velocity(self, pA, vA, pB, vB, pG=None, max_speed=1.0):

        margin = 0.00
        samples = 100
        preferred_v = None

        # generate candidate velocities
        if pG is None:
            v0 = np.array([0.5,0.5])
            sampled_velocities = [v0 * np.array([np.cos(theta), np.sin(theta)])
                                  for theta in np.linspace(0, 2 * np.pi, samples)]
        else:
            # compute preferred velocity so that the robot heads to the goal
            preferred_v = (pG - pA) / np.linalg.norm(pG - pA) * max_speed

            # prepare sample velocity so that the robot can avoid VO
            th = np.linspace(0, 2 * np.pi, 100)
            vel = np.linspace(0, max_speed, 100)
            vv, thth = np.meshgrid(vel, th)
            vx_sample = (vv * np.cos(thth)).flatten()
            vy_sample = (vv * np.sin(thth)).flatten()

            sampled_velocities = np.stack((vx_sample, vy_sample))
            sampled_velocities = np.transpose(sampled_velocities)
            sampled_velocities = np.vstack([sampled_velocities, preferred_v])
            norms = [np.linalg.norm(e - preferred_v) for e in sampled_velocities]
            sorted_indices = np.argsort(norms)
            sampled_velocities = sampled_velocities[sorted_indices]

        for _vA in sampled_velocities:

            # tp1, tp2 are tangent points from pA to the Minkowski sum of rA + rB
            tp1, tp2 = self.tangent_points(pB, self.rA + self.rB, pA)
            if tp1 is None and tp2 is None:
                return preferred_v

            # Add vB to compute actual VO(vB)
            # Add margin to make the Minkowski sum a bit larger
            tp1 += vB * self.t_hori + margin * (tp1 - pB)
            tp2 += vB * self.t_hori + margin * (tp2 - pB)

            # Check if the point P is the inside of the triangle ABC
            # and point Q is the outside of the triangle ABC

            P = pA
            Q = pA + _vA
            A = pA + vB * self.t_hori
            B = tp1 + LARGE_NUM * (tp1 - A)
            C = tp2 + LARGE_NUM * (tp2 - A)
            tri = [A, B, C]

            is_inside = False
            if self.in_triangle(tri, P) and self.in_triangle(tri, Q):
                is_inside = True

            if not is_inside:
                return _vA

        return preferred_v

    def tangent_points(self, circle_center, radius, point):

        Cx, Cy = circle_center
        Px, Py = point
        a = radius

        from math import sqrt, acos, atan2, sin, cos

        b = sqrt((Px - Cx) ** 2 + (Py - Cy) ** 2)  # hypot() also works here
        try:
            th = acos(a / b)  # angle theta
        except:
            return None, None

        d = atan2(Py - Cy, Px - Cx)  # direction angle of point P from C
        d1 = d + th  # direction angle of point T1 from C
        d2 = d - th  # direction angle of point T2 from C

        T1x = Cx + a * cos(d1)
        T1y = Cy + a * sin(d1)
        T2x = Cx + a * cos(d2)
        T2y = Cy + a * sin(d2)

        return np.array([T1x, T1y]), np.array([T2x, T2y])

    def draw(self, pA, vA, pB, vB):
        fig, ax = plt.subplots()

        # Draw robot A and B as circles
        robotA_circle = patches.Circle(pA, self.rA, fill=True, color='lightgreen', alpha=0.5, label='Robot A')
        robotB_circle = patches.Circle(pB, self.rB, fill=True, color='blue', alpha=0.5, label='Robot B')
        ax.add_patch(robotA_circle)
        ax.add_patch(robotB_circle)

        # Draw robot A and B trajectories
        ax.annotate('', xy=pA + vA, xytext=pA, arrowprops=dict(arrowstyle='->', lw=1.0, color='blue'))
        ax.annotate('', xy=pB + vB, xytext=pB, arrowprops=dict(arrowstyle='->', lw=1.0, color='red'))
        ax.annotate('', xy=vB, xytext=pA, arrowprops=dict(arrowstyle='->', lw=1.0, color='red'))

        combined_radius = self.rA + self.rB
        circle_center_org = (pB[0], pB[1])

        # Draw the Minkowski sum circle
        circle = patches.Circle(circle_center_org, combined_radius, fill=True, color='skyblue', alpha=0.5, linestyle='-', label='Minkowski sum')
        ax.add_patch(circle)

        # Calculate the tangent points
        tangent_point1, tangent_point2 = self.tangent_points(circle_center_org, combined_radius, pA)

        # Draw the VO cone using tangent lines
        triangle = patches.Polygon([pA, tangent_point1, tangent_point2], alpha=0.5, closed=True, color='gray')
        ax.add_patch(triangle)
        triangle = patches.Polygon([pA + vB, tangent_point1 + vB, tangent_point2 + vB], alpha=0.5, closed=True, color='gray', label='VO cone')#
        ax.add_patch(triangle)

        ax.set_aspect('equal')
        ax.set_xlim([-2, 4])
        ax.set_ylim([-3, 6])
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        ax.set_title('Velocity Obstacle')
        plt.tight_layout()
        plt.show()

def main():
    import warnings
    warnings.filterwarnings("ignore", category=matplotlib.MatplotlibDeprecationWarning)

    # Usage
    rA = 0.5
    rB = 0.5
    vo = VelocityObstacle(radius_A=rA, radius_B=rB, time_horizon=1)
    pA = np.array([0, 0])
    vA = np.array([2, 0.5])
    pB = np.array([0, 4])
    vB = np.array([2, -0.5])
    vo.draw(pA, vA, pB, vB)

    new_vA = vo.desired_velocity(pA, vA, pB, vB)
    vo.draw(pA, new_vA, pB, vB)


if __name__ == '__main__':
    main()
