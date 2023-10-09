import pygame
import math
import numpy as np
import sys
from robot import Robot, Car

STEER_ANG_DEG = 10

from matplotlib import colors as mcolors
colors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)
by_hsv = sorted((tuple(mcolors.rgb_to_hsv(mcolors.to_rgba(color)[:3])), name) for name, color in colors.items())
sorted_names = [name for hsv, name in by_hsv]

color_dict: dict = {}
for name in sorted_names:
    rgb = [0, 0, 0]
    for i, e in enumerate(mcolors.to_rgb(name)):
        rgb[i] = int(e * 255)

    color_dict[name] = tuple(rgb)


def get_color(name: str) -> tuple:
    return color_dict[name]

class Screen:
    def __init__(self):
        pygame.init()

        # Set up display
        self.WIDTH, self.HEIGHT = 640, 480
        self.win = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Animation")

        # Clock to control frame rate
        self.clock = pygame.time.Clock()

        # Colors
        self.color = color_dict


    def update_robots(self, robots, trajectories=None):
        self.win.fill(self.color['gray'])

        for r in robots:
            rr = self.meter_to_pixel(r.radius)
            sx = self.meter_to_pixel(r.start[0])
            sy = self.meter_to_pixel(r.start[1])
            gx = self.meter_to_pixel(r.end[0])
            gy = self.meter_to_pixel(r.end[1])

            pygame.draw.circle(self.win, self.color[r.color], (sx, sy), rr, width=1)
            pygame.draw.circle(self.win, self.color[r.color], (gx, gy), rr, width=1)


        for r in robots:
            self.draw_robot(r)

        for r, traj in trajectories.items():
            for i, (x1, y1) in enumerate(traj):
                if i == len(traj) - 1:
                    break
                x1 = self.meter_to_pixel(x1)
                y1 = self.meter_to_pixel(y1)
                x2 = self.meter_to_pixel(traj[i + 1][0])
                y2 = self.meter_to_pixel(traj[i + 1][1])
                pygame.draw.line(self.win, self.color[r.color], (x1, y1), (x2, y2), 1)

        pygame.display.flip()

    def update_cars(self, robots, trajectories=None):
        self.win.fill(self.color['gray'])


        for r in robots:
            self.draw_car(r)

        for traj in trajectories.values():
            for i, (x1, y1) in enumerate(traj):
                if i == len(traj) - 1:
                    break
                x1 = self.meter_to_pixel(x1)
                y1 = self.meter_to_pixel(y1)
                x2 = self.meter_to_pixel(traj[i + 1][0])
                y2 = self.meter_to_pixel(traj[i + 1][1])
                pygame.draw.line(self.win, self.color['lightgreen'], (x1, y1), (x2, y2), 1)

        pygame.display.flip()

    def meter_to_pixel(self, x):
        scale = 20
        return x * scale

    def draw_robot(self, r: Robot):
        radius = self.meter_to_pixel(r.radius)
        x = self.meter_to_pixel(r.position[0])
        y = self.meter_to_pixel(r.position[1])
        pygame.draw.circle(self.win, self.color[r.color], (x, y), radius, width=1)

    def draw_car(self, r: Car):
        CAR_LENGTH = self.meter_to_pixel(r.length)
        CAR_WIDTH = self.meter_to_pixel(r.width)
        WHEEL_LEN = self.meter_to_pixel(r.wheel_len)
        WHEEL_WIDTH = self.meter_to_pixel(r.wheel_width)
        BACKTOWHEEL = self.meter_to_pixel(r.backtowheel)
        TREAD = self.meter_to_pixel(r.tread)
        WB = self.meter_to_pixel(r.wb)

        x = self.meter_to_pixel(r.position[0])
        y = self.meter_to_pixel(r.position[1])
        yaw = -np.degrees(r._yaw)
        car_rect = pygame.Rect(0, 0, CAR_LENGTH, CAR_WIDTH)
        car_image = pygame.Surface((CAR_LENGTH, CAR_WIDTH), pygame.SRCALPHA)
        pygame.draw.rect(car_image, self.color[r.color], car_rect, width=2)

        # REAR
        rr_wheel = pygame.Rect(BACKTOWHEEL,
                               WHEEL_WIDTH / 2.0,
                               WHEEL_LEN,
                               WHEEL_WIDTH)
        rl_wheel = pygame.Rect(BACKTOWHEEL,
                               CAR_WIDTH - WHEEL_WIDTH - WHEEL_WIDTH / 2.0,
                               WHEEL_LEN,
                               WHEEL_WIDTH)
        pygame.draw.rect(car_image, self.color['black'], rr_wheel, width=1)
        pygame.draw.rect(car_image, self.color['black'], rl_wheel, width=1)

        # FRONT
        fr_wheel_rect = pygame.Rect(BACKTOWHEEL + WB,
                                    WHEEL_WIDTH / 2.0,
                                    WHEEL_LEN,
                                    WHEEL_WIDTH)
        fl_wheel_rect = pygame.Rect(BACKTOWHEEL + WB,
                                    CAR_WIDTH - WHEEL_WIDTH - WHEEL_WIDTH / 2.0,
                                    WHEEL_LEN,
                                    WHEEL_WIDTH)
        # Rotating front wheels
        fr_wheel_image = pygame.Surface((WHEEL_LEN, WHEEL_WIDTH), pygame.SRCALPHA)
        fl_wheel_image = pygame.Surface((WHEEL_LEN, WHEEL_WIDTH), pygame.SRCALPHA)
        pygame.draw.rect(fr_wheel_image, self.color['black'], (0, 0, WHEEL_LEN, WHEEL_WIDTH), width=1)
        pygame.draw.rect(fl_wheel_image, self.color['black'], (0, 0, WHEEL_LEN, WHEEL_WIDTH), width=1)
        rotated_fr_wheel = pygame.transform.rotate(fr_wheel_image, -math.degrees(r.steer))
        rotated_fl_wheel = pygame.transform.rotate(fl_wheel_image, -math.degrees(r.steer))
        car_image.blit(rotated_fr_wheel, fr_wheel_rect.topleft)
        car_image.blit(rotated_fl_wheel, fl_wheel_rect.topleft)
        rotated_image = pygame.transform.rotate(car_image, yaw)
        rect = rotated_image.get_rect(center=(x, y))

        self.win.blit(rotated_image, rect.topleft)


# --------------------------------------------------------------


def acceleration(frame):
    return 40.0 * np.cos(np.radians(5 * frame % 360))


def steering_angle(frame):
    return np.radians(STEER_ANG_DEG) + 0.3 * np.sin(np.radians(3 * frame % 360))


def main_car():
    screen = Screen()
    robots = init_cars()

    run = True
    frame = 0
    dt = 0.01

    trajectories = {}
    for r in robots:
        trajectories[r.label] = []

    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        screen.clock.tick(60)  # 60 frames per second

        # create inputs
        for robot_a in robots:
            u = [acceleration(frame),
                 steering_angle(frame)]
            robot_a.move(u, dt=dt)

            # update trajectories
            trajectories[robot_a.label].append((robot_a.position[0],
                                                robot_a.position[1]))

        # draw
        screen.update_cars(robots, trajectories)

        frame += 1

    pygame.quit()
    sys.exit()


def main_robots():
    from vo import VelocityObstacle

    screen = Screen()
    robots = init_robots()

    run = True
    frame = 0
    dt = 0.05

    is_rvo = True
    margin = 0.0


    # Init VOs
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


    trajectories = {}
    for r in robots:
        trajectories[r] = []


    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        screen.clock.tick(30)  # 60 frames per second

        # update VOs
        vo_unions = {}
        for robot_a in robots:
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


        # create inputs
        for robot_a in robots:
            # v = robot_a.velocity
            vo_union = vo_unions[robot_a.label]
            v = vo.desired_velocity_ma(robot_a.position,
                                       robot_a.velocity,
                                       robot_a.end,
                                       vo_union, max_speed=robot_a.max_speed)
            robot_a.move(v, dt=dt)

            # update trajectories
            trajectories[robot_a].append((robot_a.position[0],
                                                robot_a.position[1]))


        # draw
        screen.update_robots(robots, trajectories)

        frame += 1

    pygame.quit()
    sys.exit()




def init_robots():
    robotA = Robot(start=(10, 10), end=(20, 20), speed=1, max_speed=5.0, radius=1.0, color="red", label='RobotA')
    robotB = Robot(start=(20, 12), end=(12, 15), speed=1, max_speed=5.0, radius=1.0, color="green", label='RobotB')
    robotC = Robot(start=(10, 14), end=(18, 15), speed=1, max_speed=5.0, radius=1.0, color="blue", label='RobotC')
    robotD = Robot(start=(10, 20), end=(15, 10), speed=1, max_speed=5.0, radius=1.0, color="orange", label='RobotD')
    robotE = Robot(start=(16, 20), end=(18, 12), speed=1, max_speed=5.0, radius=1.0, color="black", label='RobotE')
    robots = [robotA, robotB, robotC, robotD, robotE]
    return robots


def init_cars():
    robotA = Car(start=(10, 10), end=(30, 30),
                 speed=10, radius=3.6, wb=2.3, max_speed=[10, -3],
                 color="red", label='RobotA')
    robotB = Car(start=(30, 30), end=(10, 10),
                 speed=10, radius=3.6, wb=2.3, max_speed=[10, -3],
                 color="green", label='RobotB')
    robotC = Car(start=(10, 30), end=(30, 10),
                 speed=10, radius=3.6, wb=2.3, max_speed=[10, -3],
                 color="blue", label='RobotC')
    robots = [robotA, robotB, robotC]  # , robotD, robotE]
    return robots


if __name__ == '__main__':
    # main_car()
    main_robots()
