import math
import pytest
from pi5_slam_bringup.demo_sensors import ray_distance


def test_room_and_obstacle_are_visible():
    assert ray_distance(0., 0., 0.) == pytest.approx(5.)
    assert ray_distance(0., 0., math.pi/2) == pytest.approx(4.)
    assert ray_distance(0., 0., math.pi) == pytest.approx(3.)
    assert ray_distance(0., 1., 0.) == pytest.approx(2.5)


def test_parallel_and_out_of_range_segments():
    assert ray_distance(0., 0., 0., [(1., 1., 2., 1.)]) == 12.
    assert ray_distance(0., 0., 0., [(-1., -1., -1., 1.)]) == 12.
