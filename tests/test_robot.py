import pytest
import numpy as np
from lib.robot import Robot


class TestRobot:
    def setup_method(self):
        self.robot = Robot(start=(0, 0), end=(10, 0), speed=1.00, radius=1.0)

    def test_move_default(self):
        self.robot.move()
        assert np.all(self.robot.position == np.array([1.0, 0]))

    def test_move_input(self):
        self.robot.move(velocity=[1, 1])
        assert np.all(self.robot.position == np.array([1.0, 1.0]))

    def test_current_velocity(self):
        assert np.all(self.robot._velocity == [1.0, 0.0])
        self.robot.move(velocity=[1, 1])
        assert np.all(self.robot._velocity == [1, 1])

    def test_remaining_distance(self):
        for _ in range(10):
            self.robot.move()

        assert self.robot.moving == False

    def test_remaining_distance_over_run(self):
        for _ in range(100):
            self.robot.move()

        assert self.robot.moving == False

    def test_init_speed_too_large(self):
        with pytest.raises(Exception) as e:
            self.robot.move(velocity=[100, 100])

        assert str(e.value) == "speed is too large."

    # def test_move_speed_too_large(self):
    #     with pytest.raises(Exception) as e:
    #         Robot(start=(0, 0), end=(10, 0), speed=100.00, radius=1.0)
    #
    #     assert str(e.value) == 'speed is too large.'
