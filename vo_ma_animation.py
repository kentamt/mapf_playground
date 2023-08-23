import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.patches as patches

from robot import Robot
from vo import VelocityObstacle

matplotlib.use("TkAgg")

class Animation:
    def __init__(self, dt, fps, save_gif=False):
        # plots
        fig, ax = plt.subplots()
        self._fig = fig
        self._ax = ax
        self._dt = dt
        self._fps = fps
        self._save_gif = save_gif
        self._points = None
        self._trajs = None
        self._quivers = None

    @property
    def fig(self):
        return self._fig

    def init_points(self, robots):
        self._points = {}
        for r in robots:
            point = patches.Circle(
                (r.position[0], r.position[1]), r.radius, fill=True, color=r.color, alpha=0.5, label=r.label)
            self._points[r.label] = point
        for point in self._points.values():
            self._ax.add_patch(point)

    def init_trajs(self, robots):
        self._trajs = {}
        for r in robots:
            (trajA,) = self._ax.plot([], [], "-", linewidth=0.8, color=r.color)
            self._trajs[r.label] = {}
            self._trajs[r.label]['traj'] = trajA
            self._trajs[r.label]['x'] = []
            self._trajs[r.label]['y'] = []

    def init_ax(self):
        self._ax.legend(
            loc="upper center",
            bbox_to_anchor=(0.5, 1.05),
            ncol=3,
            fancybox=True,
            shadow=True,
        )
        self._ax.set_aspect("equal")
        self._ax.set_ylim((-15, 15))
        self._ax.set_ylim((-5, 15))

    def init_quivers(self, robots):
        self._quivers = {}
        for r in robots:
            quiver = self._ax.quiver(
                r.position[0], r.position[1], r.velocity[0], r.velocity[1], angles="xy", scale_units="xy", scale=1,
                color="red"
            )
            self._quivers[r.label] = quiver

    def plot_list(self):
        ret = []
        for point in self._points.values():
            ret.append(point)
        for qui in self._quivers.values():
            ret.append(qui)
        for traj in self._trajs.values():
            ret.append(traj['traj'])
        return ret

    def plot_start_point(self, robot_a):
        print(robot_a.position)
        self._ax.plot(
            robot_a.start[0],
            robot_a.start[1],
            "s",
            color=robot_a.color,
            markersize=15,
            fillstyle="none",
            label=f"Start {robot_a.label}",
        )

    def plot_end_point(self, robot_a):
        self._ax.plot(
            robot_a.end[0],
            robot_a.end[1],
            "o",
            color=robot_a.color,
            markersize=15,
            fillstyle="none",
            label=f"End {robot_a.label}",
        )

    def update_trajs(self, robot):
        self._trajs[robot.label]['x'].append(robot.position[0])
        self._trajs[robot.label]['y'].append(robot.position[1])
        self._trajs[robot.label]['traj'].set_data(self._trajs[robot.label]['x'],
                                                  self._trajs[robot.label]['y'])

    def update_quivers(self, robot):
        self._quivers[robot.label].set_offsets(robot.position)
        self._quivers[robot.label].set_UVC(robot.velocity[0], robot.velocity[1])

    def update_points(self, robot):
        self._points[robot.label].center = (robot.position[0], robot.position[1])


def main():
    # Define initial and final positions for both robots
    robotA = Robot(start=(0, 0), end=(10, 10), speed=1, max_speed=2.0, radius=1.0, color="red", label='RobotA')
    robotB = Robot(start=(10, 2), end=(2, 5), speed=1, max_speed=2.0, radius=1.0, color="green", label='RobotB')
    robotC = Robot(start=(0, 4), end=(8, 5), speed=1, max_speed=2.0, radius=1.0, color="blue", label='RobotC')
    robotD = Robot(start=(0, 10), end=(5, 0), speed=1, max_speed=2.0, radius=1.0, color="orange", label='RobotD')
    robotE = Robot(start=(6, 10), end=(8, 0), speed=1, max_speed=2.0, radius=1.0, color="black", label='RobotE')
    robots = [robotA, robotB, robotC, robotD, robotE]

    # setting
    dt = 0.1
    save_gif = False
    fps = 24
    is_rvo = False
    margin = 0.2

    # init plots
    ani_obj = Animation(dt, fps, save_gif)
    ani_obj.init_points(robots)
    ani_obj.init_trajs(robots)
    ani_obj.init_quivers(robots)
    ani_obj.init_ax()

    # VO
    t_hori = 1
    vos = {}
    triangles = {}
    for robot_a in robots:
        other_robots = [x for x in robots if x != robot_a]
        for robot_b in other_robots:
            vo = VelocityObstacle(radius_A=robot_a.radius,
                                  radius_B=robot_b.radius,
                                  time_horizon=t_hori, rvo=is_rvo)
            vos[(robot_a.label, robot_b.label)] = vo


    def init():
        print('start init')
        for robot_a in robots:
            ani_obj.update_points(robot_a)
            ani_obj.update_quivers(robot_a)
            ani_obj.plot_start_point(robot_a)
            ani_obj.plot_end_point(robot_a)

        return ani_obj.plot_list()

    def update(frame):
        # print('update robots')
        vo_unions = {}
        for robot_a in robots:
            # update plots
            ani_obj.update_points(robot_a)
            ani_obj.update_quivers(robot_a)
            ani_obj.update_trajs(robot_a)

            vo_unions[robot_a.label] = []
            _pA = robot_a.position
            _vA = robot_a.velocity

            _other_robots = [x for x in robots if x != robot_a]

            for robot_b in _other_robots:
                _pB = robot_b.position
                _vB = robot_b.velocity

                # compute vo
                _vo = vos[(robot_a.label, robot_b.label)]
                _tri = _vo.compute_vo_triangle(margin, _pA, _vA, _pB, _vB)
                if _tri is None:
                    _tri = [0, 0, 0]

                # keep vo
                vo_unions[robot_a.label].append(_tri)

        # choose desired velocity considering all of vos
        for robot_a in robots:
            vo_union = vo_unions[robot_a.label]
            _n_vA = vo.desired_velocity_ma(robot_a.position,
                                           robot_a.velocity,
                                           robot_a.end,
                                           vo_union, max_speed=robot_a.max_speed)
            print(robot_a, _n_vA)
            robot_a.move(_n_vA, dt=dt)


        return ani_obj.plot_list()

    # Animation
    ani = FuncAnimation(
        ani_obj.fig, update, frames=100, init_func=init, blit=False, interval=1000.0 / fps
    )

    if save_gif:
        writergif = PillowWriter(fps=fps)
        gif_name = "anime/vo.gif"
        writergif.setup(ani_obj.fig, gif_name, dpi=300)
        ani.save(gif_name, writer=writergif)
    else:
        plt.show()



if __name__ == "__main__":
    main()
