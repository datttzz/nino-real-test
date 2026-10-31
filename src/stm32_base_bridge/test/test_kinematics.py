import math

import pytest

from stm32_base_bridge.kinematics import (
    Pose2D,
    integrate_wheel_positions,
    wheel_velocities_to_twist,
)


RADIUS = 0.060
SEPARATION = 0.345


def test_straight_motion():
    pose = integrate_wheel_positions(Pose2D(), 1.0, 1.0, RADIUS, SEPARATION)
    assert pose.x == pytest.approx(RADIUS)
    assert pose.y == pytest.approx(0.0)
    assert pose.yaw == pytest.approx(0.0)


def test_rotation_in_place():
    pose = integrate_wheel_positions(Pose2D(), -1.0, 1.0, RADIUS, SEPARATION)
    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.yaw == pytest.approx(2.0 * RADIUS / SEPARATION)


def test_velocity_conversion():
    linear_x, angular_z = wheel_velocities_to_twist(2.0, 2.0, RADIUS, SEPARATION)
    assert linear_x == pytest.approx(2.0 * RADIUS)
    assert angular_z == pytest.approx(0.0)

    linear_x, angular_z = wheel_velocities_to_twist(-1.0, 1.0, RADIUS, SEPARATION)
    assert linear_x == pytest.approx(0.0)
    assert angular_z == pytest.approx(2.0 * RADIUS / SEPARATION)


def test_invalid_geometry():
    with pytest.raises(ValueError):
        wheel_velocities_to_twist(0.0, 0.0, 0.0, SEPARATION)
    with pytest.raises(ValueError):
        integrate_wheel_positions(Pose2D(), 0.0, 0.0, RADIUS, 0.0)
