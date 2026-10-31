import math

import numpy as np

from pi5_slam_bringup.scan_deskew import beam_times, deskew_ranges, relative_motion

N = 720
ANGLE_MIN = math.pi - math.radians(359.0)
INC = 2.0 * math.pi / N
SCAN_TIME = 0.144


def _wall_scan(laser_x, wz, vx, wall_x=2.0):
    """Ranges an RPLIDAR on a moving base would report for the wall x=wall_x (base frame at t_ref)."""
    t_ref = 10.0
    times = beam_times(N, t_ref, SCAN_TIME, reverse=True)
    ranges = np.full(N, np.inf)
    for k, t in enumerate(times):
        yaw = wz * (t - t_ref)
        bx = vx * (t - t_ref)                  # straight-line motion is enough for small yaw
        ox = bx + math.cos(yaw) * laser_x
        oy = math.sin(yaw) * laser_x
        direction = ANGLE_MIN + k * INC + yaw
        if math.cos(direction) > 0.3:
            ranges[k] = (wall_x - ox) / math.cos(direction)
    return times, t_ref, ranges


def _deskew(times, t_ref, ranges, laser_x, wz, vx):
    rate_t = np.array([t_ref - 1.0, t_ref + 1.0])
    motion = relative_motion(times, t_ref, rate_t, np.array([wz, wz]), rate_t, np.array([vx, vx]))
    out, _ = deskew_ranges(ranges, None, ANGLE_MIN, INC, 0.05, 12.0, motion, (laser_x, 0.0, 0.0))
    return out


def _wall_error(ranges, laser_x, wall_x=2.0):
    angles = ANGLE_MIN + np.arange(N) * INC
    valid = np.isfinite(ranges)
    x = ranges[valid] * np.cos(angles[valid])
    return np.max(np.abs(x - (wall_x - laser_x)))


def test_beam_times_reverse_order_newest_first():
    times = beam_times(4, 1.0, 0.3, reverse=True)
    assert np.allclose(times, [1.0, 0.9, 0.8, 0.7])


def test_static_scan_is_unchanged():
    times, t_ref, ranges = _wall_scan(0.223, 0.0, 0.0)
    out = _deskew(times, t_ref, ranges, 0.223, 0.0, 0.0)
    valid = np.isfinite(ranges)
    assert np.allclose(out[valid], ranges[valid], atol=1e-6)


def test_rotation_smear_is_removed():
    times, t_ref, ranges = _wall_scan(0.223, 1.5, 0.0)
    assert _wall_error(ranges, 0.223) > 0.1      # raw scan bends the wall
    out = _deskew(times, t_ref, ranges, 0.223, 1.5, 0.0)
    assert _wall_error(out, 0.223) < 0.02


def test_forward_motion_is_removed():
    times, t_ref, ranges = _wall_scan(0.223, 0.0, 0.5)
    out = _deskew(times, t_ref, ranges, 0.223, 0.0, 0.5)
    assert _wall_error(out, 0.223) < 0.02
