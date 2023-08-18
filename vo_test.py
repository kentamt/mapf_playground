import numpy as np
from vo_animation import Robot  # Assuming your main code is in robots.py

def test_initialization():
    robot = Robot([0, 0], [2, 2], 1)
    np.testing.assert_array_equal(robot._position, [0, 0])
    np.testing.assert_array_equal(robot.end, [2, 2])

def test_default_movement():
    robot = Robot([0, 0], [2, 2], 1)
    robot.move()
    expected_position = [0 + np.sqrt(2)/2, 0 + np.sqrt(2)/2]
    np.testing.assert_array_almost_equal(robot._position, expected_position)

def test_external_velocity():
    robot = Robot([0, 0], [2, 2], 1)
    robot.move(velocity=np.array([0.5, 0]))
    expected_position = [0.5, 0]
    np.testing.assert_array_almost_equal(robot._position, expected_position)

def test_stop_at_end():
    robot = Robot([0, 0], [2, 2], 1)
    for _ in range(10):
        robot.move()
    np.testing.assert_array_equal(robot._position, [2, 2])