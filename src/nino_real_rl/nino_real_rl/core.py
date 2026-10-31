"""ROS-independent adapter. PI gains/observation scales follow the supplied trainer.

Changes for hardware: feedback-clocked PI, immediate zero on stop/fault,
integral clamping retained from training. Deployment differences are documented.
"""
from dataclasses import dataclass
from math import isfinite
import numpy as np

from .features import path_features, imu_features
from .kinematics import clamp, wheel_angular_targets, limit_effort_commands

OBSERVATIONS = [
    'cross_track', 'heading_error', 'remaining', 'roll', 'pitch',
    'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z',
    'speed', 'yaw_rate', 'cmd_speed', 'cmd_yaw_rate', 'wheel_left', 'wheel_right',
    'base_left', 'base_right', 'previous_action_left', 'previous_action_right',
    'residual_left', 'residual_right',
]


def pair(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (2,) or not np.all(np.isfinite(values)):
        raise ValueError('Expected two finite values')
    return values


@dataclass(frozen=True)
class Contract:
    max_path_deviation: float = .20
    residual_limit: float = .30
    step_seconds: float = .10

    def __post_init__(self):
        if not all(isfinite(v) and v > 0 for v in (
                self.max_path_deviation, self.residual_limit, self.step_seconds)):
            raise ValueError('Invalid observation contract')
        if self.residual_limit > .30:
            raise ValueError('Residual limit exceeds supported training contract')


def observation(contract, pose, points, acceleration, quaternion, gyro,
                speed, yaw_rate, command, wheels, base, previous_action, accepted):
    ey, heading, progress, remaining = path_features(*pose, points)
    accel, roll, pitch = imu_features(acceleration, quaternion)
    wx, wy, wz = gyro
    left, right = pair(wheels)
    base, previous_action, accepted = map(pair, (base, previous_action, accepted))
    v, w = pair(command)
    values = np.asarray([
        ey/contract.max_path_deviation, heading/np.pi, remaining/10,
        roll/.5, pitch/.5, *(accel/10), wx/3, wy/3, wz/3,
        speed/.5, yaw_rate, v/.5, w, left/12, right/12,
        *(base/2), *previous_action, *(accepted/contract.residual_limit),
    ], dtype=np.float32)
    if values.shape != (23,) or not np.all(np.isfinite(values)):
        raise ValueError('Invalid observation; no zero-fill for missing inputs')
    return np.clip(values, -10, 10), dict(cross_track=ey, heading_error=heading,
                                       progress=progress, remaining=remaining,
                                       acceleration=accel.tolist())


class WheelPI:
    KP, KI = .30, .10  # Frozen training baseline; Kd = 0.
    TORQUE, RATE, SPEED, ACCEL, INTEGRAL = 2.0, 10.0, 12.0, 12.0, 4.0

    def __init__(self, radius, separation):
        if not all(isfinite(v) and v > 0 for v in (radius, separation)):
            raise ValueError('Invalid real robot geometry')
        self.radius, self.separation = radius, separation
        self.reset()

    def reset(self):
        self.target = np.zeros(2)
        self.integral = np.zeros(2)
        self.total = np.zeros(2)
        self.base = np.zeros(2)
        self.accepted = np.zeros(2)

    def update(self, command, measured, residual, dt):
        command, measured, residual = map(pair, (command, measured, residual))
        if not isfinite(dt) or not 0 < dt <= .25:
            self.reset()
            raise ValueError('Invalid wheel feedback dt')
        if np.max(np.abs(residual)) > .3000001:
            self.reset()
            raise ValueError('Residual outside contract')
        if np.max(np.abs(command)) < 1e-9:
            self.reset()  # Stop takes priority over PI, SAC and slew shaping.
            return self.total.copy()
        requested = np.clip(wheel_angular_targets(*command, self.radius, self.separation),
                            -self.SPEED, self.SPEED)
        self.target += np.clip(requested-self.target, -self.ACCEL*dt, self.ACCEL*dt)
        error = self.target-measured
        self.integral = np.clip(self.integral+error*dt, -self.INTEGRAL, self.INTEGRAL)
        self.base = np.clip(self.KP*error+self.KI*self.integral, -self.TORQUE, self.TORQUE)
        requested_total = np.clip(self.base+residual, -self.TORQUE, self.TORQUE)
        # This is the pre-slew accepted residual, exactly as observed during training.
        self.accepted = requested_total-self.base
        self.total = np.asarray(limit_effort_commands(requested_total, self.total, measured,
                               dt, self.TORQUE, self.RATE, self.SPEED))
        return self.total.copy()


def source_stamp(seconds, now, previous=None, timeout=.25):
    if not all(isfinite(x) for x in (seconds, now)) or seconds <= 0:
        raise ValueError('Missing/invalid source timestamp')
    if not -.05 <= now-seconds <= timeout:
        raise ValueError('Stale/future source timestamp')
    if previous is not None and seconds <= previous:
        raise ValueError('Duplicate/backwards source timestamp')
    return seconds


def hardware_contract(data):
    for key in ('firmware_torque_verified', 'wheel_torque_nm_verified',
                'watchdog_verified', 'feedback_signs_verified',
                'accept_feedback_rate_difference'):
        if data.get(key) is not True:
            raise ValueError('Hardware not verified: '+key)
    if data.get('command_topic') != '/cmd_torque':
        raise ValueError('Only verified /cmd_torque Float64MultiArray [left,right] supported')
    for name in ('left_command_sign', 'right_command_sign'):
        if type(data.get(name)) not in (int, float) or data[name] not in (-1, 1):
            raise ValueError('Verify '+name+' as +1 or -1')
    limit = data.get('verified_wheel_torque_limit_nm', 0)
    watchdog = data.get('verified_watchdog_seconds', 0)
    if not isfinite(limit) or limit < WheelPI.TORQUE:
        raise ValueError('Verified wheel torque limit must cover software limit 2 N.m')
    if not isfinite(watchdog) or not 0 < watchdog <= .5:
        raise ValueError('Verify a firmware torque watchdog <= 0.5s')
    return data
