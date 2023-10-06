import pygame
import math
import numpy as np
import sys
from robot import Car

def acceleration(frame):
    return 40.0 * np.cos(np.radians(5 * frame % 360))


def steering_angle(frame):
    return np.radians(10.0)#  + 0.2 * np.sin(np.radians(10 * frame % 360))


class Screen:
    def __init__(self):
        # Initialize Pygame
        pygame.init()

        # Set up display
        self.WIDTH, self.HEIGHT = 800, 600
        self.win = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Animation")
        # Clock to control frame rate
        self.clock = pygame.time.Clock()

        # Colors
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.BLACK = (0, 0, 0)

        # # Car parameters in meters
        # self.CAR_LENGTH_M = 4.0  # meters
        # self.CAR_WIDTH_M = 1.8  # meters
        # self.WHEEL_LEN_M = 0.6  # meters
        # self.WHEEL_WIDTH_M = 0.2  # meters
        # self.TREAD_M = 1.5  # meters
        #
        # # Conversion scale: 1 meter is equal to SCALE pixels
        # self.SCALE = 50  # e.g., 1 meter is 50 pixels
        #
        # # Convert to pixels for rendering
        # self.CAR_LENGTH = self.CAR_LENGTH_M * self.SCALE
        # self.CAR_WIDTH = self.CAR_WIDTH_M * self.SCALE
        # self.WHEEL_LEN = self.WHEEL_LEN_M * self.SCALE
        # self.WHEEL_WIDTH = self.WHEEL_WIDTH_M * self.CALE
        # self.TREAD = self.TREAD_M * self.SCALE

        # Initial position and yaw
        # x, y, yaw = self.WIDTH // 2, self.HEIGHT // 2, 0

    def update(self, robots):
        self.win.fill(self.WHITE)

        for r in robots:
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
            steer = 0

            car_rect = pygame.Rect(0, 0, CAR_LENGTH, CAR_WIDTH)
            car_image = pygame.Surface((CAR_LENGTH, CAR_WIDTH), pygame.SRCALPHA)
            pygame.draw.rect(car_image, self.BLACK, car_rect)

            rr_wheel = pygame.Rect(BACKTOWHEEL, TREAD, WHEEL_LEN, WHEEL_WIDTH)
            rl_wheel = pygame.Rect(BACKTOWHEEL, CAR_WIDTH - TREAD - WHEEL_WIDTH, WHEEL_LEN, WHEEL_WIDTH)
            fr_wheel = pygame.Rect(BACKTOWHEEL + WB, TREAD, WHEEL_LEN, WHEEL_WIDTH)
            fl_wheel = pygame.Rect(BACKTOWHEEL + WB, CAR_WIDTH - TREAD - WHEEL_WIDTH, WHEEL_LEN, WHEEL_WIDTH)

            pygame.draw.rect(car_image, self.BLACK, fr_wheel)
            pygame.draw.rect(car_image, self.BLACK, fl_wheel)
            pygame.draw.rect(car_image, self.BLACK, rr_wheel)
            pygame.draw.rect(car_image, BLACK, rl_wheel)

            rotated_image = pygame.transform.rotate(car_image, math.degrees(self.yaw))
            rect = rotated_image.get_rect(center=(self.x, self.y))
            screen.blit(rotated_image, rect.topleft)

            # Create a rotated car surface
            # car_surf = pygame.Surface((CAR_LENGTH, CAR_WIDTH), pygame.SRCALPHA)
            # pygame.draw.rect(car_surf, self.RED, car_surf.get_rect())
            # rotated_car = pygame.transform.rotate(car_surf, yaw)
            #
            # # Create rotated wheels
            # wheel_surf = pygame.Surface((WHEEL_LEN, WHEEL_WIDTH), pygame.SRCALPHA)
            # pygame.draw.rect(wheel_surf, self.BLACK, wheel_surf.get_rect())
            #
            # rotated_fr_wheel = pygame.transform.rotate(wheel_surf, yaw + steer)  # Front right wheel
            # rotated_fl_wheel = pygame.transform.rotate(wheel_surf, yaw + steer)  # Front left wheel
            # rotated_rr_wheel = pygame.transform.rotate(wheel_surf, yaw)  # Rear right wheel
            # rotated_rl_wheel = pygame.transform.rotate(wheel_surf, yaw)  # Rear left wheel
            #
            # # Blit the surfaces
            # new_rect = rotated_car.get_rect(center=(x, y))
            # self.win.blit(rotated_car, new_rect.topleft)
            #
            # # Calculate wheel positions
            # offset_y = 2
            # offset_x = 4
            #
            # self.win.blit(rotated_fr_wheel, (x + math.cos(math.radians(yaw)) * offset_x - math.sin(math.radians(yaw)) * (
            #             TREAD / 2) - rotated_fr_wheel.get_width() / 2,
            #                             y - math.sin(math.radians(yaw)) * offset_x - math.cos(math.radians(yaw)) * (
            #                                         TREAD / 2) - rotated_fr_wheel.get_height() / 2))
            #
            # self.win.blit(rotated_fl_wheel, (x + math.cos(math.radians(yaw)) * offset_x + math.sin(math.radians(yaw)) * (
            #             TREAD / 2) - rotated_fl_wheel.get_width() / 2,
            #                             y - math.sin(math.radians(yaw)) * offset_x + math.cos(math.radians(yaw)) * (
            #                                         TREAD / 2) - rotated_fl_wheel.get_height() / 2))
            #
            # self.win.blit(rotated_rr_wheel, (x - math.cos(math.radians(yaw)) * offset_x - math.sin(math.radians(yaw)) * (
            #             TREAD / 2) - rotated_rr_wheel.get_width() / 2,
            #                             y + math.sin(math.radians(yaw)) * offset_x - math.cos(math.radians(yaw)) * (
            #                                         TREAD / 2) - rotated_rr_wheel.get_height() / 2))
            #
            # self.win.blit(rotated_rl_wheel, (x - math.cos(math.radians(yaw)) * offset_x + math.sin(math.radians(yaw)) * (
            #             TREAD / 2) - rotated_rl_wheel.get_width() / 2,
            #                             y + math.sin(math.radians(yaw)) * offset_x + math.cos(math.radians(yaw)) * (
            #                                         TREAD / 2) - rotated_rl_wheel.get_height() / 2))

        pygame.display.flip()

    def meter_to_pixel(self, x):
        scale = 10
        return x * scale


def main():

    screen = Screen()
    robotA = Car(start=(10, 10), end=(30, 30),
                 speed=10, radius=3.6, wb=2.3, max_speed=[10, -3],
                 color="red", label='RobotA')
    robotB = Car(start=(30, 30), end=(10, 10),
                 speed=10, radius=3.6, wb=2.3, max_speed=[10, -3],
                 color="green", label='RobotB')
    robots = [robotA, robotB]  # , robotC, robotD, robotE]

    run = True
    frame = 0
    dt = 0.01
    while run:
        screen.clock.tick(60)  # 60 frames per second

        # create inputs
        for robot_a in robots:
            u = [acceleration(frame), steering_angle(frame)]
            robot_a.move(u, dt=dt)

        # Draw car
        screen.update(robots)

        frame += 1

    pygame.quit()
    sys.exit()

if __name__ == '__main__':
    main()