import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import matplotlib.patches as patches

# Warning
# matplotlib.use('TkAgg')

class VelocityObstacle:
    def __init__(self, radius_A, radius_B, time_horizon):
        self.rA = radius_A
        self.rB = radius_B
        self.τ = time_horizon

    def compute_vo_parameters(self, pA, vA, pB, vB):
        relative_position = pB - pA
        relative_velocity = vB - vA

        # The circle's center for Minkowski sum
        circle_center = relative_position + self.τ * vB
        combined_radius = self.rA + self.rB
        
        return circle_center, combined_radius, relative_velocity

    # def is_velocity_inside_vo(self, velocity, circle_center, combined_radius, relative_velocity):
    #     # Check if the vector from circle_center to velocity passes through the Minkowski sum circle
    #     distance_to_center = np.linalg.norm(velocity - circle_center)
    #
    #     if distance_to_center <= combined_radius:
    #         return True
    #     return False

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

    def get_avoidance_velocity(self, circle_center, combined_radius, relative_velocity):
        # For simplicity, we choose the nearest point on the boundary of the VO
        direction_to_center = relative_velocity - circle_center
        direction_normalized = direction_to_center / np.linalg.norm(direction_to_center)
        new_velocity = circle_center + direction_normalized * combined_radius
        return new_velocity

    def desired_velocity(self, pA, vA, pB, vB, pG=None, max_speed=1.0):

        # generate candidate velocities
        samples = 100
        if pG is None:
            v0 = np.array([0.5,0.5])
            sampled_velocities = [v0 * np.array([np.cos(theta), np.sin(theta)])
                                  for theta in np.linspace(0, 2 * np.pi, samples)]
        else:
            preferred_v = (pG - pA) / np.linalg.norm(pG - pA) * max_speed  # TODO:
            # sampled_velocities = [preferred_v * np.array([np.cos(theta), np.cos(theta)]) for theta in np.linspace(0, 2*np.pi, samples)]
            th = np.linspace(0, 2 * np.pi, 100)
            vel = np.linspace(0, 4.0, 100)
            vv, thth = np.meshgrid(vel, th)
            vx_sample = (vv * np.cos(thth)).flatten()
            vy_sample = (vv * np.sin(thth)).flatten()

            sampled_velocities = np.stack((vx_sample, vy_sample))
            sampled_velocities = np.transpose(sampled_velocities)
            sampled_velocities = np.vstack([sampled_velocities, preferred_v])
            # sort in order of the difference between the length of the current V and sampled V
            # sampled_velocities.append(preferred_v)  # Ensure preferred velocity is in the set
            #
            # Sort velocities based on their distance to the preferred velocity
            norms = [np.linalg.norm(e - preferred_v) for e in sampled_velocities]
            sorted_indices = np.argsort(norms)
            sampled_velocities = sampled_velocities[sorted_indices]

        # sort candidates in order of XXXX
        for _vA in sampled_velocities:

            circle_center, combined_radius, relative_velocity = self.compute_vo_parameters(pA, vA, pB, vB)

            # NOTE: You need to add vB to compute actual VO(vB)
            tangent_point1, tangent_point2 = self.tangent_points(pB, combined_radius, pA)
            tangent_point1 += vB * 5
            tangent_point2 += vB * 5

            is_intersect = False
            for B in [tangent_point1, tangent_point2]:
                P = pA
                Q = pA + _vA
                A = pA + vB * 5

                if False: # if P is in VO, get out the area asap.
                    pass


                if self.do_intersect(P, Q, A, B):
                    print('vA is passing through the VO(vB)')
                    is_intersect += True

            if not is_intersect:
                return _vA

        return preferred_v

    def tangent_points(self, circle_center, radius, point, extension_length=1.0):
        import math

        h, k = circle_center
        x1, y1 = point

        dx = x1 - h
        dy = y1 - k

        # Calculate distance between point and circle center
        d = math.sqrt(dx ** 2 + dy ** 2)

        # Check if point is inside circle
        if d < radius:
            raise ValueError("The point is inside the circle. Tangent points are not defined.")

        distance_to_midpoint = math.sqrt(d ** 2 - radius ** 2)

        # Midpoint M
        Mx = x1 + (distance_to_midpoint / d) * (h - x1)
        My = y1 + (distance_to_midpoint / d) * (k - y1)

        delta_x = radius * dy / d
        delta_y = radius * dx / d

        # Tangent points T1 and T2
        T1x = Mx + delta_x
        T1y = My - delta_y
        T2x = Mx - delta_x
        T2y = My + delta_y

        return (T1x, T1y), (T2x, T2y)

    def draw(self, pA, vA, pB, vB):
        fig, ax = plt.subplots()

        # Draw robot A and B as circles
        robotA_circle = patches.Circle(pA, self.rA, fill=True, color='lightgreen', alpha=0.5, label='Robot A')
        robotB_circle = patches.Circle(pB, self.rB, fill=True, color='blue', alpha=0.5, label='Robot B')
        ax.add_patch(robotA_circle)
        ax.add_patch(robotB_circle)

        # Draw robot A and B trajectories
        # NOTE: it's super weird. You cannot comment out the line below.
        ax.arrow(pA[0], pA[1], vA[0], vA[1], head_width=0.2, head_length=0.3, fc='blue', ec='blue', alpha=0.0)
        ax.annotate('', xy=pA + vA, xytext=pA, arrowprops=dict(arrowstyle='->', lw=1.0, color='blue'))
        ax.annotate('', xy=pB + vB, xytext=pB, arrowprops=dict(arrowstyle='->', lw=1.0, color='red'))
        ax.annotate('', xy=vB, xytext=pA, arrowprops=dict(arrowstyle='->', lw=1.0, color='red'))

        circle_center, combined_radius, relative_velocity = self.compute_vo_parameters(pA, vA, pB, vB)
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
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        ax.set_title('Velocity Obstacle')
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
    print(new_vA)
    vo.draw(pA, new_vA, pB, vB)


if __name__ == '__main__':
    main()
