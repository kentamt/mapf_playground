import numpy as np
import matplotlib.pyplot as plt
import math


class Vehicle:
    def __init__(self, car_robot):

        self.patches = []

        # Vehicle parameters
        self.LENGTH = car_robot.length
        self.WIDTH = car_robot.width
        self.BACKTOWHEEL = car_robot.backtowheel
        self.WHEEL_LEN = car_robot.wheel_len
        self.WHEEL_WIDTH = car_robot.wheel_width
        self.TREAD = car_robot.tread
        self.WB = car_robot.wb
        self.MAX_STEER = car_robot.max_steer

    def plot(self, x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):
        outline = np.array(
            [
                [
                    -self.BACKTOWHEEL,
                    (self.LENGTH - self.BACKTOWHEEL),
                    (self.LENGTH - self.BACKTOWHEEL),
                    -self.BACKTOWHEEL,
                    -self.BACKTOWHEEL,
                ],
                [
                    self.WIDTH / 2,
                    self.WIDTH / 2,
                    -self.WIDTH / 2,
                    -self.WIDTH / 2,
                    self.WIDTH / 2,
                ],
            ]
        )
        fr_wheel = np.array(
            [
                [
                    self.WHEEL_LEN,
                    -self.WHEEL_LEN,
                    -self.WHEEL_LEN,
                    self.WHEEL_LEN,
                    self.WHEEL_LEN,
                ],
                [
                    -self.WHEEL_WIDTH - self.TREAD,
                    -self.WHEEL_WIDTH - self.TREAD,
                    self.WHEEL_WIDTH - self.TREAD,
                    self.WHEEL_WIDTH - self.TREAD,
                    -self.WHEEL_WIDTH - self.TREAD,
                ],
            ]
        )
        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        # Rotate front wheels
        Rot1 = np.array(
            [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]]
        )
        Rot2 = np.array(
            [[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]]
        )
        fr_wheel[1, :] += self.TREAD
        fl_wheel[1, :] -= self.TREAD
        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[1, :] -= self.TREAD
        fl_wheel[1, :] += self.TREAD

        #
        fr_wheel[0, :] += self.WB
        fl_wheel[0, :] += self.WB

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T
        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y

        if self.patches:
            self.patches[0].set_data(outline[0, :], outline[1, :])
            self.patches[1].set_data(fr_wheel[0, :], fr_wheel[1, :])
            self.patches[2].set_data(rr_wheel[0, :], rr_wheel[1, :])
            self.patches[3].set_data(fl_wheel[0, :], fl_wheel[1, :])
            self.patches[4].set_data(rl_wheel[0, :], rl_wheel[1, :])
            self.patches[5].set_data(x, y)
        else:
            p_outline, = plt.plot(
                outline[0, :], outline[1, :], truckcolor, linewidth=0.5
            )
            p_fr_wheel, = plt.plot(
                fr_wheel[0, :], fr_wheel[1, :], truckcolor, linewidth=0.5
            )
            p_rr_wheel, = plt.plot(
                rr_wheel[0, :], rr_wheel[1, :], truckcolor, linewidth=0.5
            )
            p_fl_wheel, = plt.plot(
                fl_wheel[0, :], fl_wheel[1, :], truckcolor, linewidth=0.5
            )
            p_rl_wheel, = plt.plot(
                rl_wheel[0, :], rl_wheel[1, :], truckcolor, linewidth=0.5
            )
            p_center, = plt.plot(x, y, "+")

            self.patches = [
                p_outline,
                p_fr_wheel,
                p_rr_wheel,
                p_fl_wheel,
                p_rl_wheel,
                p_center,
            ]


# Example Usage:
if __name__ == "__main__":
    import matplotlib.animation as animation

    fig, ax = plt.subplots()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.grid(True)

    vehicle = Vehicle()

    def init():
        vehicle.plot(0, 0, 0)
        return vehicle.patches

    def update(frame):
        x = frame * 0.1
        y = 0.0
        yaw = 0.0
        steer = 0.0
        vehicle.plot(x, y, yaw, steer)
        return vehicle.patches

    ani = animation.FuncAnimation(
        fig, update, frames=np.arange(0, 100), init_func=init, blit=False, repeat=False
    )

    plt.axis("equal")
    plt.show()