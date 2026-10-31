"""Motion-compensate RPLIDAR scans and restamp them to the time of the last beam.

rplidar_ros stamps a revolution when it has been read out, so the header is
about one scan period (~0.14 s at 7 Hz) later than the first beam. SLAM Toolbox
looks up a single odometry pose at the header time and does not deskew, so a
turning robot smears every scan. This node rotates and translates each beam
into the laser pose at the last beam using the gyro yaw rate and the wheel
speed, then republishes the scan stamped at that time.

Beam timing was fitted on RPBot bags (2026-10-17) against the gyro: with
angle_compensate the ranges array is in reverse acquisition order, so index 0
is the newest beam (`beam_order: reverse`); the newest beam is 10-30 ms before
the header stamp (two sessions gave -0.010 s and -0.032 s).
"""

from collections import deque
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def beam_times(count: int, t_last: float, scan_time: float, reverse: bool) -> np.ndarray:
    """Acquisition time of every beam; `reverse` means index 0 is the newest."""
    if count < 2 or scan_time <= 0.0:
        return np.full(count, t_last)
    step = scan_time / (count - 1)
    age = np.arange(count) if reverse else np.arange(count)[::-1]
    return t_last - age * step


def relative_motion(times, t_ref, rate_t, rate_wz, speed_t, speed_vx):
    """Base pose at each time expressed in the base frame at t_ref.

    Integrates yaw rate and forward speed (held at the ends of the buffers).
    Returns (x, y, yaw) arrays aligned with `times`.
    """
    grid = np.unique(np.concatenate([times, [t_ref]]))
    wz = np.interp(grid, rate_t, rate_wz)
    vx = np.interp(grid, speed_t, speed_vx)
    dt = np.diff(grid)
    yaw = np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * dt)])
    c, s = np.cos(yaw), np.sin(yaw)
    x = np.concatenate([[0.0], np.cumsum(0.5 * (vx[1:] * c[1:] + vx[:-1] * c[:-1]) * dt)])
    y = np.concatenate([[0.0], np.cumsum(0.5 * (vx[1:] * s[1:] + vx[:-1] * s[:-1]) * dt)])
    ref = np.searchsorted(grid, t_ref)
    yaw0, x0, y0 = yaw[ref], x[ref], y[ref]
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    dx, dy = x - x0, y - y0
    rx, ry, ryaw = c0 * dx + s0 * dy, -s0 * dx + c0 * dy, yaw - yaw0
    idx = np.searchsorted(grid, times)
    return rx[idx], ry[idx], ryaw[idx]


def deskew_ranges(ranges, intensities, angle_min, angle_increment, range_min, range_max,
                  motion, laser_pose):
    """Move every valid beam into the laser frame at the reference time and re-bin it.

    `motion` is (x, y, yaw) of the base at each beam time relative to the reference;
    `laser_pose` is (x, y, yaw) of the laser in the base frame.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    count = ranges.size
    out = np.full(count, np.inf, dtype=np.float32)
    out_i = np.zeros(count, dtype=np.float32) if intensities is not None else None
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    if not valid.any():
        return out, out_i
    k = np.nonzero(valid)[0]
    lx, ly, lyaw = laser_pose
    a = angle_min + k * angle_increment + lyaw
    bx = lx + ranges[k] * np.cos(a)            # beam end point in base frame at its time
    by = ly + ranges[k] * np.sin(a)
    mx, my, myaw = motion[0][k], motion[1][k], motion[2][k]
    c, s = np.cos(myaw), np.sin(myaw)
    px = c * bx - s * by + mx - lx             # same point in base frame at t_ref, minus laser offset
    py = s * bx + c * by + my - ly
    cl, sl = math.cos(lyaw), math.sin(lyaw)
    qx, qy = cl * px + sl * py, -sl * px + cl * py
    r = np.hypot(qx, qy)
    b = np.rint((np.arctan2(qy, qx) - angle_min) / angle_increment).astype(np.int64) % count
    order = np.argsort(-r)                     # write far first so the nearest return wins a bin
    out[b[order]] = r[order]
    if out_i is not None:
        out_i[b[order]] = np.asarray(intensities, dtype=np.float32)[k][order]
    return out, out_i


class ScanDeskew(Node):
    def __init__(self) -> None:
        super().__init__('scan_deskew')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/bridge')
        self.declare_parameter('odom_topic', '/odom/unfiltered')
        self.declare_parameter('laser_x', 0.0)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw', 0.0)
        self.declare_parameter('last_beam_offset_s', -0.03)
        self.declare_parameter('beam_order', 'reverse')
        self.declare_parameter('deskew', True)
        self.declare_parameter('max_wait_s', 0.06)

        p = lambda name: self.get_parameter(name).value
        self.laser_pose = (float(p('laser_x')), float(p('laser_y')), float(p('laser_yaw')))
        self.offset = float(p('last_beam_offset_s'))
        self.reverse = str(p('beam_order')) == 'reverse'
        self.enabled = bool(p('deskew'))
        self.max_wait = float(p('max_wait_s'))
        self.rates = deque(maxlen=400)     # (t, wz)
        self.speeds = deque(maxlen=400)    # (t, vx)
        self.pending = deque()

        self.publisher = self.create_publisher(LaserScan, str(p('output_topic')), qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(p('input_topic')), self._scan, qos_profile_sensor_data)
        self.create_subscription(Imu, str(p('imu_topic')), self._imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(p('odom_topic')), self._odom, 10)
        self.create_timer(0.01, self._flush)
        self.get_logger().info(
            f'scan_deskew {p("input_topic")} -> {p("output_topic")} deskew={self.enabled} '
            f'order={p("beam_order")} last_beam_offset={self.offset:+.3f}s laser={self.laser_pose}')

    @staticmethod
    def _sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _imu(self, msg: Imu) -> None:
        self.rates.append((self._sec(msg.header.stamp), msg.angular_velocity.z))

    def _odom(self, msg: Odometry) -> None:
        self.speeds.append((self._sec(msg.header.stamp), msg.twist.twist.linear.x))

    def _scan(self, msg: LaserScan) -> None:
        self.pending.append((self.get_clock().now().nanoseconds * 1e-9, msg))

    def _flush(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.pending:
            arrived, msg = self.pending[0]
            t_last = self._sec(msg.header.stamp) + self.offset
            have_rate = bool(self.rates) and self.rates[-1][0] >= t_last
            if not have_rate and now - arrived < self.max_wait:
                return
            self.pending.popleft()
            self.publisher.publish(self._process(msg, t_last))

    def _process(self, msg: LaserScan, t_last: float) -> LaserScan:
        out = LaserScan()
        out.header.frame_id = msg.header.frame_id
        sec = math.floor(t_last)
        out.header.stamp.sec, out.header.stamp.nanosec = int(sec), int((t_last - sec) * 1e9)
        out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = 0.0
        out.scan_time = msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        if not self.enabled or len(self.rates) < 2:
            out.ranges, out.intensities = msg.ranges, msg.intensities
            return out
        times = beam_times(len(msg.ranges), t_last, msg.scan_time, self.reverse)
        rate = np.array(self.rates)
        rate = rate[np.argsort(rate[:, 0], kind='stable')]
        speed = np.array(self.speeds) if len(self.speeds) >= 2 else np.array([[t_last, 0.0], [t_last + 1, 0.0]])
        speed = speed[np.argsort(speed[:, 0], kind='stable')]
        motion = relative_motion(times, t_last, rate[:, 0], rate[:, 1], speed[:, 0], speed[:, 1])
        ranges, intensities = deskew_ranges(
            msg.ranges, msg.intensities if len(msg.intensities) == len(msg.ranges) else None,
            msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, motion, self.laser_pose)
        out.ranges = ranges.tolist()
        out.intensities = intensities.tolist() if intensities is not None else []
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanDeskew()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
