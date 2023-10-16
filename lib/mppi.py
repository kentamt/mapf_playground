from copy import deepcopy
import numpy as np
from lib.robot_mod import Car


class MPPIControllerBase:
    def __init__(self,
                 start_pose,
                 goal_pose,
                 sigma,
                 time_horizon=100,
                 k=300,
                 l=2,
                 nominal_input=None):

        self.time_horizon = time_horizon
        self.k = k
        self.l = l

        # noise parameters
        self.sigma_acc = sigma[0]
        self.sigma_del = sigma[1]

        if nominal_input is None:
            self.p_u = self.nominal_input()
        else:
            self.p_u = nominal_input

        # init car model
        x, y, yaw, v = start_pose
        gx, gy, gyaw, _ = goal_pose
        self.car_model = Car(
            start=(x, y, yaw),
            end=(gx, gy, gyaw),
            speed=v,
            radius=3.6,
            wb=2.3,
            max_speed=[20, -3],
            color="salmon",
            label="CarModel",
        )
        self.tmp_car_model = deepcopy(self.car_model)

        self.Q = None
        self.w = None
        self.U = None


    def init_car_model(self):
        self.car_model = deepcopy(self.tmp_car_model)

    def nominal_input(self):
        """
        nominal input is a 2d array with shape (2, time_horizon)
        first row is acceleration
        second row is delta
        """
        nominal = np.zeros((2, self.time_horizon))
        return nominal

    def compute_input(self, dt=0.1, timestamp=0, prefix='') -> np.ndarray:
        """
        S: score array
        W: weight array
        Q: trajectory array, first row is x, second row is y, third row is yaw
        U: input array, first row is acceleration, second row is delta
        """
        S = np.zeros(self.k)
        w = np.zeros(self.k)
        Q = np.zeros((self.k, 3, self.time_horizon))
        U = np.zeros((self.k, 2, self.time_horizon))

        for k in range(self.k):

            # noise array
            e_acc = np.random.normal(0, self.sigma_acc, self.time_horizon)
            e_del = np.random.normal(0, self.sigma_del, self.time_horizon)
            acc = self.p_u[0, :] + e_acc
            delta = self.p_u[1, :] + e_del

            # keep the input in the range
            U[k, 0, :] = acc
            U[k, 1, :] = delta

            self.init_car_model()
            for t in range(self.time_horizon):
                # keep the state
                Q[k, 0, t] = self.car_model.position[0]  # x
                Q[k, 1, t] = self.car_model.position[1]  # y
                Q[k, 2, t] = self.car_model.yaw  # yaw

                # move the car model
                self.car_model.move([acc[t], delta[t]], dt=dt, ignore_goal=True)

            # keep the state
            Q[k, 0, t] = self.car_model.position[0]  # x
            Q[k, 1, t] = self.car_model.position[1]  # y
            Q[k, 2, t] = self.car_model.yaw  # yaw

            # compute score for k
            S[k] = self.compute_cost(Q, k)

        # compute weight for k
        for k in range(self.k):
            w[k] = np.exp(-1 / self.l * (S[k] - np.min(S)))
        w = w / np.sum(w)

        # compute u_out
        u_out = np.zeros((2, self.time_horizon))
        u_out[0] = w.dot(U[:, 0, :])
        u_out[1] = w.dot(U[:, 1, :])

        self.Q = deepcopy(Q)
        self.w = deepcopy(w)
        self.U = deepcopy(U)

        self.plot(Q, timestamp, prefix, u_out, w, dt)

        return u_out

    def plot(self, Q, timestamp, prefix, u_out, w, dt):

        from matplotlib import pyplot as plt
        import matplotlib.cm as cm

        norm = plt.Normalize(0, 0.01)
        cmap = cm.jet

        fig, ax = plt.subplots(1, 1)
        for k in range(self.k):
            color = cmap(norm(w[k]))
            ax.plot(Q[k, 0, :], Q[k, 1, :], '-', alpha=0.1, color=color)
        self.init_car_model()
        q_result = np.zeros((2, self.time_horizon))
        for t in range(self.time_horizon):
            self.car_model.move([u_out[0, t], u_out[1, t]], dt=dt, ignore_goal=True)
            q_result[0, t] = self.car_model.position[0]
            q_result[1, t] = self.car_model.position[1]
        ax.plot(q_result[0, :], q_result[1, :], '--', color='red', label='result')
        self.init_car_model()
        ax.plot(self.car_model.position[0], self.car_model.position[1], 'o', label='start')
        ax.plot(self.car_model.goal[0], self.car_model.goal[1], 'o', label='goal')
        ax.arrow(self.car_model.start_state.x,
                 self.car_model.start_state.y,
                 10 * np.cos(self.car_model.start_state.yaw),
                 10 * np.sin(self.car_model.start_state.yaw), width=0.5, color='black')
        ax.arrow(self.car_model.goal_state.x,
                 self.car_model.goal_state.y,
                 10 * np.cos(self.car_model.goal_state.yaw),
                 10 * np.sin(self.car_model.goal_state.yaw), width=0.5, color='black')
        ax.title.set_text('MPPI')
        ax.legend()
        ax.axis('equal')
        ax.set_xlim(0, 80)
        ax.set_ylim(0, 80)
        sm = cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        plt.colorbar(sm, label='w')
        # plt.show()
        plt.savefig(f'{prefix}_mppi_{timestamp:03d}.png')

    def compute_cost(self, Q, k):
        return 1.0

class SampleMPPIController(MPPIControllerBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def compute_cost(self, Q, k):
        closest_point = Q[k, :, -1]
        goal_state = np.array(
            [self.car_model.goal_state.x,
             self.car_model.goal_state.y,
             self.car_model.goal_state.yaw])

        # find the closest point to goal
        closest_idx = 0
        for i, q in enumerate(Q[k, :, :].T):
            if np.linalg.norm(q - goal_state) < np.linalg.norm(closest_point - goal_state):
                closest_point = q
                closest_idx = i

        c_distance = np.sqrt((closest_point[0] - goal_state[0]) ** 2 + (closest_point[1] - goal_state[1]) ** 2)
        c_angle = np.sqrt((closest_point[2] - goal_state[2]) ** 2)
        c_time = float(closest_idx / self.time_horizon)
        cost = c_distance + 10 * c_angle + c_time
        return cost


class MultiAgentMPPIController(SampleMPPIController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)



def main():
    x, y, yaw, v = 30, -10, np.radians(45), 5
    gx, gy, gyaw, gv = 55, 20, np.radians(20), 0

    car_model = Car(
        start=(x, y, yaw),
        end=(gx, gy, gyaw),
        speed=v,
        radius=3.6,
        wb=2.3,
        max_speed=[20, -3],
        color="salmon",
        label="CarModel",
    )

    c = 0
    u_out = None
    dt = 0.1
    for t in range(200):

        if t % 2 == 0:
            print(t)

            mppi = SampleMPPIController(time_horizon=60,
                                        k=500,
                                        start_pose=[x, y, yaw, v],
                                        goal_pose=[gx, gy, gyaw, gv],
                                        sigma=[0.1, np.radians(12.0)],
                                        nominal_input=u_out)
            u_out = mppi.compute_input(dt=dt, timestamp=t)
            c = 0

        # move the car model according to the input
        if car_model.arrived:
            break
        u = u_out[:, c]
        car_model.move(u, dt=dt)
        x = car_model.state.x
        y = car_model.state.y
        yaw = car_model.state.yaw
        v = car_model.state.v
        c += 1


if __name__ == "__main__":
    main()
