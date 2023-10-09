import pygame
import math
import numpy as np
import sys
from robot import Car

STEER_ANG_DEG = 10


def acceleration(frame):
    return 40.0  # * np.cos(np.radians(5 * frame % 360))


def steering_angle(frame):
    return np.radians(STEER_ANG_DEG) + 0.3 * np.sin(np.radians(3 * frame % 360))


class Screen:
    def __init__(self):
        # Initialize Pygame
        pygame.init()

        # Set up display
        self.WIDTH, self.HEIGHT = 640, 480
        self.win = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Animation")

        # Clock to control frame rate
        self.clock = pygame.time.Clock()

        # Colors
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.BLACK = (0, 0, 0)
        self.GRAY = (128, 128, 128)
        self.GOLD = (255, 215, 0)


    def update(self, robots, trajectories=None):
        self.win.fill(self.GRAY)

        for r in robots:
            self.draw_robot(r)

        for traj in trajectories.values():
            for i, (x1, y1) in enumerate(traj):
                if i == len(traj) - 1:
                    break
                x1 = self.meter_to_pixel(x1)
                y1 = self.meter_to_pixel(y1)
                x2 = self.meter_to_pixel(traj[i + 1][0])
                y2 = self.meter_to_pixel(traj[i + 1][1])

                pygame.draw.line(self.win, self.BLACK, (x1, y1), (x2, y2), 1)


        pygame.display.flip()

    def meter_to_pixel(self, x):
        scale = 10
        return x * scale

    def draw_robot(self, r):
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
        pygame.draw.rect(car_image, self.GOLD, car_rect, width=1)

        rr_wheel = pygame.Rect(BACKTOWHEEL, WHEEL_WIDTH/2.0, WHEEL_LEN, WHEEL_WIDTH)
        rl_wheel = pygame.Rect(BACKTOWHEEL, CAR_WIDTH - WHEEL_WIDTH - WHEEL_WIDTH/2.0, WHEEL_LEN, WHEEL_WIDTH)
        pygame.draw.rect(car_image, self.BLACK, rr_wheel, width=1)
        pygame.draw.rect(car_image, self.BLACK, rl_wheel, width=1)

        fr_wheel_rect = pygame.Rect(BACKTOWHEEL + WB, WHEEL_WIDTH/2.0, WHEEL_LEN, WHEEL_WIDTH)
        fl_wheel_rect = pygame.Rect(BACKTOWHEEL + WB, CAR_WIDTH - WHEEL_WIDTH - WHEEL_WIDTH/2.0, WHEEL_LEN, WHEEL_WIDTH)

        # Rotating front wheels
        fr_wheel_image = pygame.Surface((WHEEL_LEN, WHEEL_WIDTH), pygame.SRCALPHA)
        fl_wheel_image = pygame.Surface((WHEEL_LEN, WHEEL_WIDTH), pygame.SRCALPHA)
        pygame.draw.rect(fr_wheel_image, self.BLACK, (0, 0, WHEEL_LEN, WHEEL_WIDTH), width=1)
        pygame.draw.rect(fl_wheel_image, self.BLACK, (0, 0, WHEEL_LEN, WHEEL_WIDTH), width=1)
        rotated_fr_wheel = pygame.transform.rotate(fr_wheel_image, -math.degrees(r.steer))
        rotated_fl_wheel = pygame.transform.rotate(fl_wheel_image, -math.degrees(r.steer))
        car_image.blit(rotated_fr_wheel, fr_wheel_rect.topleft)
        car_image.blit(rotated_fl_wheel, fl_wheel_rect.topleft)
        rotated_image = pygame.transform.rotate(car_image, yaw)
        rect = rotated_image.get_rect(center=(x, y))
        self.win.blit(rotated_image, rect.topleft)


def main():
    screen = Screen()
    robots = init_robots()

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
        screen.update(robots, trajectories)

        frame += 1

    pygame.quit()
    sys.exit()


def init_robots():
    robotA = Car(start=(10, 10), end=(30, 30),
                 speed=10, radius=3.6, wb=2.3, max_speed=[10, -3],
                 color="red", label='RobotA')
    robotB = Car(start=(30, 30), end=(10, 10),
                 speed=10, radius=3.6, wb=2.3, max_speed=[10, -3],
                 color="green", label='RobotB')
    robots = [robotA, robotB]  # , robotC, robotD, robotE]
    return robots


if __name__ == '__main__':
    main()
