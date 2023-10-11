from lib.dubins import plan_dubins_path
from lib.robot_mod import Car, Robot

class PathPlanner:
    def __init__(self):
        self.x = None
        self.y = None
        self.yaw = None

    # def generate(self):
    #     """ should be implemented in a child class"""
    #     pass

class DubinsPathPlanner(PathPlanner):
    def __init__(self):
        super(DubinsPathPlanner, self).__init__()

    def generate(self, car_robot, curvature):
        sx = car_robot._start.rear_x
        sy = car_robot._start.rear_y
        syaw = car_robot._start.yaw
        ex = car_robot._goal.rear_x
        ey = car_robot._goal.rear_y
        eyaw = car_robot._goal.yaw
        path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(sx,
                                                                   sy,
                                                                   syaw,
                                                                   ex,
                                                                   ey,
                                                                   eyaw,
                                                                   curvature)

        self.x = path_x
        self.y = path_y
        self.yaw = path_yaw
