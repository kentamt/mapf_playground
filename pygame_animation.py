import pygame
import math
import numpy as np
from robot import Robot, Car
from matplotlib import colors as mcolors

colors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)
by_hsv = sorted(
    (tuple(mcolors.rgb_to_hsv(mcolors.to_rgba(color)[:3])), name)
    for name, color in colors.items()
)
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

        self.scale = 10

    def quit(self):
        pygame.quit()

    def is_quit_event(self):
        run = True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
        return run

    def update_robots(self, robots, trajectories=None):
        self.win.fill(self.color["gray"])

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
        self.win.fill(self.color["gray"])

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
                pygame.draw.line(
                    self.win, self.color["lightgreen"], (x1, y1), (x2, y2), 1
                )

        pygame.display.flip()

    def meter_to_pixel(self, x):
        return x * self.scale

    def draw_robot(self, r: Robot):
        radius = self.meter_to_pixel(r.radius)
        x = self.meter_to_pixel(r.position[0])
        y = self.meter_to_pixel(r.position[1])
        pygame.draw.circle(self.win, self.color[r.color], (x, y), radius, width=0)

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
        rr_wheel = pygame.Rect(BACKTOWHEEL, WHEEL_WIDTH / 2.0, WHEEL_LEN, WHEEL_WIDTH)
        rl_wheel = pygame.Rect(
            BACKTOWHEEL,
            CAR_WIDTH - WHEEL_WIDTH - WHEEL_WIDTH / 2.0,
            WHEEL_LEN,
            WHEEL_WIDTH,
        )
        pygame.draw.rect(car_image, self.color["black"], rr_wheel, width=1)
        pygame.draw.rect(car_image, self.color["black"], rl_wheel, width=1)

        # FRONT
        fr_wheel_rect = pygame.Rect(
            BACKTOWHEEL + WB, WHEEL_WIDTH / 2.0, WHEEL_LEN, WHEEL_WIDTH
        )
        fl_wheel_rect = pygame.Rect(
            BACKTOWHEEL + WB,
            CAR_WIDTH - WHEEL_WIDTH - WHEEL_WIDTH / 2.0,
            WHEEL_LEN,
            WHEEL_WIDTH,
        )
        # Rotating front wheels
        fr_wheel_image = pygame.Surface((WHEEL_LEN, WHEEL_WIDTH), pygame.SRCALPHA)
        fl_wheel_image = pygame.Surface((WHEEL_LEN, WHEEL_WIDTH), pygame.SRCALPHA)
        pygame.draw.rect(
            fr_wheel_image, self.color["black"], (0, 0, WHEEL_LEN, WHEEL_WIDTH), width=1
        )
        pygame.draw.rect(
            fl_wheel_image, self.color["black"], (0, 0, WHEEL_LEN, WHEEL_WIDTH), width=1
        )
        rotated_fr_wheel = pygame.transform.rotate(
            fr_wheel_image, -math.degrees(r.steer)
        )
        rotated_fl_wheel = pygame.transform.rotate(
            fl_wheel_image, -math.degrees(r.steer)
        )
        car_image.blit(rotated_fr_wheel, fr_wheel_rect.topleft)
        car_image.blit(rotated_fl_wheel, fl_wheel_rect.topleft)
        rotated_image = pygame.transform.rotate(car_image, yaw)
        rect = rotated_image.get_rect(center=(x, y))

        self.win.blit(rotated_image, rect.topleft)
