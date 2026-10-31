"""Differential-drive kinematics shared by the bridge and its tests."""

from dataclasses import dataclass
import math


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate_wheel_positions(
    pose: Pose2D,
    left_delta_rad: float,
    right_delta_rad: float,
    wheel_radius: float,
    wheel_separation: float,
) -> Pose2D:
    """Integrate one differential-drive encoder increment."""
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be positive')
    if wheel_separation <= 0.0:
        raise ValueError('wheel_separation must be positive')

    left_distance = left_delta_rad * wheel_radius
    right_distance = right_delta_rad * wheel_radius
    distance = 0.5 * (left_distance + right_distance)
    delta_yaw = (right_distance - left_distance) / wheel_separation

    if abs(delta_yaw) < 1.0e-9:
        new_x = pose.x + distance * math.cos(pose.yaw)
        new_y = pose.y + distance * math.sin(pose.yaw)
    else:
        turn_radius = distance / delta_yaw
        new_yaw_unwrapped = pose.yaw + delta_yaw
        new_x = pose.x + turn_radius * (
            math.sin(new_yaw_unwrapped) - math.sin(pose.yaw)
        )
        new_y = pose.y - turn_radius * (
            math.cos(new_yaw_unwrapped) - math.cos(pose.yaw)
        )

    return Pose2D(new_x, new_y, normalize_angle(pose.yaw + delta_yaw))


def wheel_velocities_to_twist(
    left_rad_s: float,
    right_rad_s: float,
    wheel_radius: float,
    wheel_separation: float,
) -> tuple[float, float]:
    """Return forward velocity and yaw rate from wheel angular velocities."""
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be positive')
    if wheel_separation <= 0.0:
        raise ValueError('wheel_separation must be positive')

    left_velocity = left_rad_s * wheel_radius
    right_velocity = right_rad_s * wheel_radius
    linear_x = 0.5 * (left_velocity + right_velocity)
    angular_z = (right_velocity - left_velocity) / wheel_separation
    return linear_x, angular_z
