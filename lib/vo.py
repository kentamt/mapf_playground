import math
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import matplotlib.patches as patches

# Warning
# matplotlib.use('TkAgg')

LARGE_NUM = 10000


class VelocityObstacle:
    def __init__(self, radius_A, radius_B, time_horizon, rvo=False):
        self.rvo = rvo
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
            (y - P[1]) / (Q[1] - P[1]) >= 0 and (y - P[1]) / (Q[1] - P[1]) <= 1
        ):
            # Check if the intersection point is in the direction of the ray
            if (x - A[0]) / (B[0] - A[0]) >= 0 and (y - A[1]) / (B[1] - A[1]) >= 0:
                return True
        return False

    @staticmethod
    def in_triangle(triangle, point):
        """check if the point is inside the triangle"""

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
        if (c1 <= 0 and c2 <= 0 and c3 <= 0) or (c1 >= 0 and c2 >= 0 and c3 >= 0):
            return True
        else:
            return False

    def desired_velocity(self, pA, vA, pB, vB, pG, max_speed=1.0, verbose=False):

        margin = 0.2

        # compute preferred velocity so that the robot heads to the goal
        preferred_v = (pG - pA) / np.linalg.norm(pG - pA) * max_speed

        # prepare sample velocity so that the robot can avoid VO
        sampled_velocities = self.__sampled_velocities(max_speed, preferred_v, vA)

        for _vA in sampled_velocities:

            tri = self.compute_vo_triangle(margin, pA, vA, pB, vB)
            if tri is None:
                return preferred_v

            # Check if the point P is the inside of the triangle ABC
            # and point Q is the outside of the triangle ABC
            P = pA
            Q = pA + _vA
            is_inside = False
            if self.in_triangle(tri, Q):
                is_inside = True

            if not is_inside:
                if verbose:
                    print(f"tri: {tri}")
                    print(f"outside: {P}, {Q}")
                return _vA

        if verbose:
            print("no candidate")

        return preferred_v

    def desired_velocity_ma(self, pA, vA, pG, vo_union, max_speed=1.0, verbose=False):
        """Choose one velocity"""

        # compute preferred velocity so that the robot heads to the goal
        preferred_v = (pG - pA) / np.linalg.norm(pG - pA) * max_speed
        # preferred_v = 0.2 * preferred_v + 0.8 * vA

        # prepare sample velocity so that the robot can avoid VO
        sampled_velocities = self.__sampled_velocities(max_speed, preferred_v, vA)

        for _vA in sampled_velocities:
            is_outside = True
            for tri in vo_union:

                if tri is None:
                    return preferred_v

                # Check if the point P is the inside of the triangle ABC
                # and point Q is the outside of the triangle ABC
                P = pA
                Q = pA + _vA
                if self.in_triangle(tri, Q):
                    is_outside = False

            if is_outside:
                if verbose:
                    print(f"tri: {tri}")
                    print(f"outside: {P}, {Q}")
                return _vA

        if verbose:
            print("no candidate")

        return preferred_v

    def __sampled_velocities(self, max_speed, preferred_v, vA):
        from scipy.spatial import distance

        direction = np.arctan2(vA[1], vA[0])
        th = np.linspace(direction, direction + np.pi, 100)
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
        return sampled_velocities

    def compute_vo_triangle(self, margin, pA, vA, pB, vB):

        # tp1, tp2 are tangent points from pA to the Minkowski sum of rA + rB
        tp1, tp2 = self.tangent_points(pB, self.rA + self.rB, pA)
        if tp1 is None and tp2 is None:
            tri = None

        if not self.rvo:
            # Add vB to compute actual VO(vB) and margin to make the Minkowski sum a bit larger
            tp1 += vB * self.t_hori + margin * (tp1 - pB)
            tp2 += vB * self.t_hori + margin * (tp2 - pB)

            A = pA + vB * self.t_hori
            B = tp1 + LARGE_NUM * (tp1 - A)
            C = tp2 + LARGE_NUM * (tp2 - A)
        else:
            r = 0.5
            tp1 += (vB - r * (vB - vA)) * self.t_hori + margin * (tp1 - pB)
            tp2 += (vB - r * (vB - vA)) * self.t_hori + margin * (tp2 - pB)
            A = pA + (vB - r * (vB - vA)) * self.t_hori
            B = tp1 + LARGE_NUM * (tp1 - A)
            C = tp2 + LARGE_NUM * (tp2 - A)

        tri = [A, B, C]

        return tri

    def compute_rvo_triangle(self, margin, pA, pB, vA, vB):

        # tp1, tp2 are tangent points from pA to the Minkowski sum of rA + rB
        tp1, tp2 = self.tangent_points(pB, self.rA + self.rB, pA)
        if tp1 is None and tp2 is None:
            tri = None

        # Add vB to compute actual VO(vB)
        # Add margin to make the Minkowski sum a bit larger
        r = 0.5
        tp1 += (vB - r * (vB - vA)) * self.t_hori + margin * (tp1 - pB)
        tp2 += (vB - r * (vB - vA)) * self.t_hori + margin * (tp2 - pB)
        A = pA + (vB - r * (vB - vA)) * self.t_hori
        B = tp1 + LARGE_NUM * (tp1 - A)
        C = tp2 + LARGE_NUM * (tp2 - A)
        tri = [A, B, C]
        return tri

    @staticmethod
    def tangent_points(circle_center, radius, point):

        c_x, c_y = circle_center
        p_x, p_y = point
        a = radius

        from math import sqrt, acos, atan2, sin, cos

        b = sqrt((p_x - c_x) ** 2 + (p_y - c_y) ** 2)  # hypot() also works here
        try:
            th = acos(a / b)  # angle theta
        except:
            return None, None

        d = atan2(p_y - c_y, p_x - c_x)  # direction angle of point P from C
        d1 = d + th  # direction angle of point T1 from C
        d2 = d - th  # direction angle of point T2 from C

        t1_x = c_x + a * cos(d1)
        t1_y = c_y + a * sin(d1)
        t2_x = c_x + a * cos(d2)
        t2_y = c_y + a * sin(d2)

        return np.array([t1_x, t1_y]), np.array([t2_x, t2_y])

    def draw(self, pA, vA, pB, vB, new_vA=None):
        fig, ax = plt.subplots()

        # Draw robot A and B as circles
        robotA_circle = patches.Circle(
            pA, self.rA, fill=True, color="lightgreen", alpha=0.5, label="Robot A"
        )
        robotB_circle = patches.Circle(
            pB, self.rB, fill=True, color="blue", alpha=0.5, label="Robot B"
        )
        ax.add_patch(robotA_circle)
        ax.add_patch(robotB_circle)

        # Draw robot A and B trajectories

        if new_vA is None:
            ax.annotate(
                "",
                xy=pA + vA,
                xytext=pA,
                arrowprops=dict(arrowstyle="->", lw=1.0, color="blue"),
            )
        else:
            ax.annotate(
                "",
                xy=pA + new_vA,
                xytext=pA,
                arrowprops=dict(arrowstyle="->", lw=1.0, color="blue"),
            )

        ax.annotate(
            "",
            xy=pB + vB,
            xytext=pB,
            arrowprops=dict(arrowstyle="->", lw=1.0, color="red"),
        )
        ax.annotate(
            "", xy=vB, xytext=pA, arrowprops=dict(arrowstyle="->", lw=1.0, color="red")
        )

        combined_radius = self.rA + self.rB
        circle_center_org = (pB[0], pB[1])

        # Draw the Minkowski sum circle
        circle = patches.Circle(
            circle_center_org,
            combined_radius,
            fill=True,
            color="skyblue",
            alpha=0.5,
            linestyle="-",
            label="Minkowski sum",
        )
        ax.add_patch(circle)

        # Draw the VO cone using tangent lines from robotA
        # tri = self.compute_vo_triangle(0.0, pA, pB, vB)
        # triangle = patches.Polygon(
        #    [tri[0] - vB, tri[1] - vB, tri[2] - vB], alpha=0.5, closed=True, color="gray"
        # )
        # ax.add_patch(triangle)

        # Draw the VO cone using tangent lines from robotA + vB
        tri = self.compute_vo_triangle(0.0, pA, vA, pB, vB)
        triangle = patches.Polygon(
            tri, alpha=0.5, closed=True, color="gray", label="VO cone"
        )  #
        ax.add_patch(triangle)

        ax.plot(
            [pA[0], pA[0] + vB[0], pA[0] + vA[0], pA[0]],
            [pA[1], pA[1] + vB[1], pA[1] + vA[1], pA[1]],
            "-",
        )

        ax.set_aspect("equal")
        ax.set_xlim([-2, 8])
        ax.set_ylim([-3, 8])
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        ax.set_title("Velocity Obstacle")
        plt.tight_layout()
        plt.show()


def main():
    import warnings
    from robot import Robot

    warnings.filterwarnings("ignore", category=matplotlib.MatplotlibDeprecationWarning)

    # robots
    robotA = Robot(start=(0, 0), end=(5, 0), speed=1.0, radius=1.5, color="red")
    robotB = Robot(start=(5, 0), end=(0, 0), speed=1.0, radius=1.5, color="green")
    rA = robotA.radius
    rB = robotB.radius
    pA = robotA.position
    pB = robotB.position
    vA = robotA.velocity
    vB = robotB.velocity
    vo = VelocityObstacle(radius_A=rA, radius_B=rB, time_horizon=1, rvo=True)
    vo.draw(pA, vA, pB, vB)

    new_vA = vo.desired_velocity(pA, vA, pB, vB, pG=robotA.goal, verbose=True)
    vo.draw(pA, vA, pB, vB, new_vA=new_vA)

    new_vB = vo.desired_velocity(pB, vB, pA, vA, pG=robotB.goal, verbose=True)
    vo.draw(pB, vB, pA, vA, new_vA=new_vB)


if __name__ == "__main__":
    main()
