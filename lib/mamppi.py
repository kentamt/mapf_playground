import numpy as np
from mppi import MPPIControllerBase, SampleMPPIController
from robot_mod import Car

def main():

    xa, ya, yawa, va = 20, 20, np.radians(45), 5
    gxa, gya, gyawa, gva = 50, 50, np.radians(45), 5


    car = {}
    car['a'] = Car(
        start=(xa, ya, yawa),
        end=(gxa, gya, gyawa),
        speed=va,
        radius=3.6,
        wb=2.3,
        max_speed=[20, -3],
        color="salmon",
        label="Car_A",
    )

    xb, yb, yawb, vb = 50, 50, np.radians(45+180), 5
    gxb, gyb, gyawb, gvb = 20, 20, np.radians(45+180), 0

    car['b'] = Car(
        start=(xb, yb, yawb),
        end=(gxb, gyb, gyawb),
        speed=vb,
        radius=3.6,
        wb=2.3,
        max_speed=[20, -3],
        color="green",
        label="Car_B",
    )


    c = 0
    u_out = {}
    u_out['a'] = None
    u_out['b'] = None
    dt = 0.1
    K = 100

    mppi = {}
    Q = {}
    w = {}
    U = {}

    for t in range(200):
        print(f'{t=}')
        if t % 2 == 0:
            mppi['a'] = SampleMPPIController(time_horizon=120,
                                        k=K, l=1,
                                        start_pose=[xa, ya, yawa, va],
                                        goal_pose=[gxa, gya, gyawa, gva],
                                        sigma=[0.1, np.radians(7.0)],
                                        nominal_input=u_out['a'])
            mppi['a'].compute_input(dt=dt, timestamp=t, prefix='a')
            Q['a'] = mppi['a'].Q
            w['a'] = mppi['a'].w
            U['a'] = mppi['a'].U

            mppi['b'] = SampleMPPIController(time_horizon=120,
                                        k=K,l=1,
                                        start_pose=[xb, yb, yawb, vb],
                                        goal_pose=[gxb, gyb, gyawb, gvb],
                                        sigma=[0.1, np.radians(7.0)],
                                        nominal_input=u_out['b'])
            mppi['b'].compute_input(dt=dt, timestamp=t, prefix='b')
            Q['b'] = mppi['b'].Q
            w['b'] = mppi['b'].w
            U['b'] = mppi['b'].U

            # check if the two cars are going to collide
            for k in range(K):
                for t in range(mppi['a'].time_horizon):
                    if np.linalg.norm(Q['a'][k,0:2,t] - Q['b'][k,0:2,t]) < 2.0:
                        w['a'][k] = -0.5
                        w['b'][k] = -0.5

            w['a'] = w['a'] / np.sum(w['a'])
            w['b'] = w['b'] / np.sum(w['b'])

            # update the u_out
            u_out = {}
            for name in ['a', 'b']:
                u_out[name] = np.zeros((2, mppi[name].time_horizon))
                u_out[name][0] = w[name].dot(U[name][:, 0, :])
                u_out[name][1] = w[name].dot(U[name][:, 1, :])

            cnt = 0

        # move the car model according to the input
        for name in ['a', 'b']:
            if car[name].arrived:
                break
            u = u_out[name][:, c]
            car[name].move(u, dt=dt)

            if name == 'a':
                xa = car[name].state.x
                ya = car[name].state.y
                yawa = car[name].state.yaw
                va = car[name].state.v
            else:
                xb = car[name].state.x
                yb = car[name].state.y
                yawb = car[name].state.yaw
                vb = car[name].state.v

        c += 1


if __name__ == '__main__':
    main()
