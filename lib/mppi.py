from copy import deepcopy
import numpy as np
from lib.robot_mod import Car

from matplotlib import pyplot as plt
import matplotlib.cm as cm


norm = plt.Normalize(0, 0.1)
cmap = cm.jet
dt = 0.1

class MPPIController:
    def __init__(self, time_horizon,
                 x, y, yaw, v,
                 gx, gy, gyaw,
                 nominal_input=None):

        self.time_horizon = time_horizon

        # TODO: it should be a np.array for better performance
        if nominal_input is None:
            self.p_u = self.nominal_input()
        else:
            self.p_u = nominal_input

        self.sigma_acc = 0.5
        self.sigma_del = np.radians(10.0)

        self.k =300
        self.l = 2
        # lambda

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

        self.timestamp = 0

    def init_car_model(self):
        self.car_model = deepcopy(self.tmp_car_model)

    def nominal_input(self):
        # nominal input is a 2d array with shape (2, time_horizon)
        # first row is acceleration
        # second row is delta
        nominal = np.zeros((2, self.time_horizon))
        return nominal

    def compute_input(self, timestamp=0) -> list[tuple[float, float]]:

        # init matplotplot ax object
        fig, ax = plt.subplots(1, 1)

        # score array
        S = np.zeros(self.k)

        # weihgt array
        w = np.zeros(self.k)

        # trajectory array, first row is x, second row is y, third row is yaw
        Q = np.zeros((self.k, 3, self.time_horizon))

        # input array, first row is acceleration, second row is delta
        U = np.zeros((self.k, 2, self.time_horizon))

        for k in range(self.k):

            self.init_car_model()

            # noise array
            e_acc = np.random.normal(0, self.sigma_acc, self.time_horizon)
            e_del = np.random.normal(0, self.sigma_del, self.time_horizon)
            acc = self.p_u[0, :] + e_acc
            delta = self.p_u[1, :] + e_del

            U[k, 0, :] = acc
            U[k, 1, :] = delta

            # Q is a 2d array with shape (2, time_horizon)
            # the first row is x, the second row is y
            # Q = np.zeros((2, self.time_horizon))

            for t in range(self.time_horizon):
                Q[k, 0, t] = self.car_model.position[0]  # x
                Q[k, 1, t] = self.car_model.position[1]  # y
                Q[k, 2, t] = self.car_model.yaw  # yaw
                self.car_model.move([acc[t], delta[t]], dt=dt, ignore_goal=True)

            Q[k, 0, t] = self.car_model.position[0]  # x
            Q[k, 1, t] = self.car_model.position[1]  # y
            Q[k, 2, t] = self.car_model.yaw  # yaw

            # compute score for k
            # score is the distance between the closest point in Q and the goal
            # first, find the closest point to the goal in Q
            # second, compute the distance between the closest point and the goal
            closest_point = Q[k, :, -1]
            goal_state = np.array(
                [self.car_model.goal_state.x, self.car_model.goal_state.y, self.car_model.goal_state.yaw])

            # cost for x-y
            closest_idx = 0
            for i, q in enumerate(Q[k, :, :].T):
                # calc 3d-distance
                if np.linalg.norm(q - goal_state) < np.linalg.norm(closest_point - goal_state):
                    closest_point = q
                    closest_idx = i
            # include yaw
            goal_state = np.array([self.car_model.goal_state.x, self.car_model.goal_state.y, self.car_model.goal_state.yaw])
            # S[k] = np.linalg.norm(closest_point - goal_state)
            S[k] = (closest_point[0] - goal_state[0])**2 + (closest_point[1] - goal_state[1])**2 + 10* (closest_point[2] - goal_state[2])**2
            S[k] += float(closest_idx / self.time_horizon)
            # plot xy trajectory in Q
            # ax.plot(Q[0, :], Q[1, :], '-')

            # plot xy trajectory in Q with the color of S[k]
            # color = cmap(norm(S[k]))
            # ax.plot(Q[0, :], Q[1, :], '-', color=color)

            # plot xy trajectory in Q with the color of w[k]
            # color = cmap(norm(w[k]))
            # ax.plot(Q[0, :], Q[1, :], '-', color=color)

            # plot the closest point in Q
            if k == 0:
                ax.plot(closest_point[0], closest_point[1], '+', c='red', label='closest point')
            else:
                ax.plot(closest_point[0], closest_point[1], '+', c='red')

        # compute weight for k
        for k in range(self.k):
            w[k] = np.exp(-1/self.l * (S[k] - np.min(S)))
        w = w / np.sum(w)

        # compute u_out
        u_out = np.zeros((2, self.time_horizon))
        for k in range(self.k):
            for t in range(self.time_horizon):
                u_out[0, t] += w[k] * U[k, 0, t]
                u_out[1, t] += w[k] * U[k, 1, t]

        # _u_out = np.zeros((2, self.time_horizon))
        # _u_out[0] = w.dot(U[:,0,:])
        # _u_out[1] = w.dot(U[:, 1, :])

        for k in range(self.k):
            # color = cmap(norm(S[k]))
            color = cmap(norm(w[k]))
            ax.plot(Q[k, 0, :], Q[k, 1, :], '-', alpha=0.1, color=color)

        # plot car movement with u_out
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

        # draw arrows on both start and goal

        ax.arrow(self.car_model.start_state.x,
                 self.car_model.start_state.y,
                 10* np.cos(self.car_model.start_state.yaw),
                 10* np.sin(self.car_model.start_state.yaw), width=0.5, color='black')
        ax.arrow(self.car_model.goal_state.x,
                 self.car_model.goal_state.y,
                 10* np.cos(self.car_model.goal_state.yaw),
                 10* np.sin(self.car_model.goal_state.yaw), width=0.5, color='black')


        ax.title.set_text('MPPI')
        ax.legend()
        ax.axis('equal')
        ax.set_xlim(0, 80)
        ax.set_ylim(-20, 40)

        sm = cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        plt.colorbar(sm, label='w')
        # plt.show()
        # save as png in the format 3 digits number
        plt.savefig(f'mppi_{timestamp:03d}.png')


        return u_out


def main():

    x, y, yaw, v = 30, -10, np.radians(45), 5
    gx, gy, gyaw = 30, 20, np.radians(135)

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
    for t in range(200):

        if t % 2 == 0:
            mppi = MPPIController(time_horizon=100,
                                  x=x, y=y, yaw=yaw, v=v,
                                  gx=gx, gy=gy, gyaw=gyaw,
                                  nominal_input=u_out)
            u_out = mppi.compute_input(t)
            c = 0
            print(t)

        u = u_out[:, c]
        car_model.move(u, dt=dt)
        x = car_model.state.x
        y = car_model.state.y
        yaw = car_model.state.yaw
        v = car_model.state.v
        c += 1


if __name__ == "__main__":
    main()
