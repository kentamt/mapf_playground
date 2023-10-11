from matplotlib import pyplot as plt
import matplotlib.patches as patches
from lib.plt_vehicle import Vehicle


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
        self._robots = None

    @property
    def fig(self):
        return self._fig

    @property
    def ax(self):
        return self._ax

    def init_points(self, robots):
        self._points = {}
        for r in robots:
            point = patches.Circle(
                (r.position[0], r.position[1]),
                r.radius,
                fill=True,
                color=r.color,
                alpha=0.5,
                label=r.label,
            )
            self._points[r.label] = point
        for point in self._points.values():
            self._ax.add_patch(point)

    def init_robots(self, robots):
        self._robots = {}
        for r in robots:
            robot = Vehicle(r)
            robot.plot(
                r.position[0],
                r.position[1],
                r.yaw,
                steer=r.yaw,
                cabcolor="-r",
                truckcolor="-k",
            )
            self._robots[r.label] = robot
        # for robot in self._robots.values():
        #     for p in robot.patches:
        #         self._ax.add_patch(p)

    def init_trajs(self, robots):
        self._trajs = {}
        for r in robots:
            (trajA,) = self._ax.plot([], [], "-", linewidth=0.8, color=r.color)
            self._trajs[r.label] = {}
            self._trajs[r.label]["traj"] = trajA
            self._trajs[r.label]["x"] = []
            self._trajs[r.label]["y"] = []

    def init_ax(self):
        self._ax.legend(
            loc="upper center",
            bbox_to_anchor=(0.5, 1.05),
            ncol=3,
            fancybox=True,
            shadow=True,
        )
        self._ax.set_aspect("equal")
        self._ax.set_ylim((30, 100))
        self._ax.set_xlim((30, 100))

    def init_quivers(self, robots):
        self._quivers = {}
        for r in robots:
            quiver = self._ax.quiver(
                r.position[0],
                r.position[1],
                r.velocity[0],
                r.velocity[1],
                angles="xy",
                scale_units="xy",
                scale=1,
                color="red",
            )
            self._quivers[r.label] = quiver

    def plot_list(self):
        ret = []
        for point in self._points.values():
            ret.append(point)
        for qui in self._quivers.values():
            ret.append(qui)
        for traj in self._trajs.values():
            ret.append(traj["traj"])
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
            robot_a.goal[0],
            robot_a.goal[1],
            "o",
            color=robot_a.color,
            markersize=15,
            fillstyle="none",
            label=f"End {robot_a.label}",
        )

    def update_trajs(self, robot):
        self._trajs[robot.label]["x"].append(robot.position[0])
        self._trajs[robot.label]["y"].append(robot.position[1])
        self._trajs[robot.label]["traj"].set_data(
            self._trajs[robot.label]["x"], self._trajs[robot.label]["y"]
        )

    def update_quivers(self, robot):
        self._quivers[robot.label].set_offsets(robot.position)
        self._quivers[robot.label].set_UVC(robot.velocity[0], robot.velocity[1])
        if robot.speed < 0:
            self._quivers[robot.label].set_color("blue")
        else:
            self._quivers[robot.label].set_color("red")

    def update_points(self, robot):
        self._points[robot.label].center = (robot.position[0], robot.position[1])

    def update_robots(self, robot, steer=0.0):
        self._robots[robot.label].plot(
            robot.position[0],
            robot.position[1],
            robot.yaw,
            steer=steer,
            cabcolor="-r",
            truckcolor="-k",
        )