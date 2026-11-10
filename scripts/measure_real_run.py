#!/usr/bin/env python3
"""Logger thụ động dùng cùng phép đo cho Nav-only, PI và SAC; không phát lệnh."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/nino_real_rl'))
from nino_real_rl.features import imu_features, path_features


class Mission:
    """Giữ riêng từng goal; dữ liệu bê xe/reset nằm ngoài cửa sổ nhiệm vụ."""
    def __init__(self, created):
        self.created = created
        self.goals = {}

    def status(self, key, start, status, received):
        if (not math.isfinite(start) or not math.isfinite(received) or
                start < self.created-.5 or start <= 0 or received < start):
            return False
        if status not in (1, 2, 3, 4, 5, 6):
            return False
        goal = self.goals.setdefault(key, dict(start=start, terminal=None, status=status))
        goal['status'] = status
        if status in (4, 5, 6) and goal['terminal'] is None:
            goal['terminal'] = received
        return True

    def at(self, stamp):
        candidates = [(key, g) for key, g in self.goals.items()
                      if g['start'] <= stamp and (g['terminal'] is None or stamp <= g['terminal'])]
        return candidates[0][0] if len(candidates) == 1 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-dir', required=True)
    parser.add_argument('--group', required=True, choices=['nav_only', 'pi', 'sac80k'])
    parser.add_argument('--robot-config', help='Cùng robot.yaml đã cài mà launch dùng')
    args = parser.parse_args()
    # Import ROS tại entry point; Mission vẫn kiểm tra offline được.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float64MultiArray
    from tf2_ros import Buffer, TransformListener
    import yaml

    out = Path(args.session_dir).resolve()
    folder = out/'measurements'
    folder.mkdir(exist_ok=False)
    robot_path = Path(args.robot_config) if args.robot_config else (
        Path(__file__).resolve().parents[1]/'src/pi5_slam_bringup/config/robot.yaml')
    robot = yaml.safe_load(robot_path.read_text())['robot']
    radius, separation = float(robot['wheel_radius']), float(robot['wheel_separation'])
    stamp_s = lambda s: s.sec+s.nanosec*1e-9

    class Logger(Node):
        def __init__(self):
            super().__init__('real_measurement_logger')
            self.mission = Mission(self.now())
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.files = {k: (folder/(k+'.jsonl')).open('x') for k in ('pose', 'imu', 'events')}
            self.references = {}
            self.path = None
            self.odom = None
            self.wheels = None
            self.command = None
            self.torque = None
            self.previous_pose = self.previous_imu = None
            self.rejected = {}
            self.last_flush = time.monotonic()
            self.counts = dict(pose=0, imu=0)
            reference_topic = '/cmd_vel' if args.group == 'nav_only' else '/rl/cmd_vel_reference'
            self.subs = [
                self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status, 10),
                self.create_subscription(RosPath, '/plan', self.on_path, 10),
                self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), qos_profile_sensor_data),
                self.create_subscription(Imu, '/imu/bridge', self.on_imu, qos_profile_sensor_data),
                self.create_subscription(JointState, '/motor_fb', lambda m: setattr(self, 'wheels', m), qos_profile_sensor_data),
                self.create_subscription(Twist, reference_topic, lambda m: setattr(self, 'command', (time.monotonic(), m)), 10),
                self.create_subscription(Float64MultiArray, '/rl/torque_preview', lambda m: setattr(self, 'torque', (time.monotonic(), m)), 10),
            ]
            self.timer = self.create_timer(.1, self.tick)
            (folder/'READY').write_text('passive logger ready; no actuator publishers\n')

        def now(self):
            return self.get_clock().now().nanoseconds*1e-9

        def write(self, kind, row):
            self.files[kind].write(json.dumps(row, allow_nan=False)+'\n')

        def reject(self, reason):
            # Lỗi TF thường chứa timestamp mới mỗi mẫu; tránh vô hạn khóa JSON.
            category = ': '.join(reason.split(': ', 2)[:2])
            self.rejected[category] = self.rejected.get(category, 0)+1
            if self.rejected[category] == 1:
                self.write('events', dict(event='measurement_rejected', reason=reason,
                                          received_ros_s=self.now()))

        def source_time(self, stamp):
            value = stamp_s(stamp)
            if value <= 0 or not -.05 <= self.now()-value <= .25:
                raise ValueError('stale/future/empty source stamp')
            return value

        def status(self, msg):
            for item in msg.status_list:
                key = bytes(item.goal_info.goal_id.uuid).hex()
                if self.mission.status(key, stamp_s(item.goal_info.stamp), item.status, self.now()):
                    self.write('events', dict(event='goal_status', goal_id=key, status=item.status,
                                             received_ros_s=self.now(), goal_start_s=stamp_s(item.goal_info.stamp)))
            self.save_summary()

        def on_path(self, msg):
            if msg.header.frame_id != 'map' or len(msg.poses) < 2 or any(
                    p.header.frame_id not in ('', 'map') for p in msg.poses):
                self.reject('invalid_path_frame_or_length')
                return
            points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if not all(math.isfinite(x) for pair in points for x in pair):
                self.reject('nonfinite_path')
                return
            self.path = (stamp_s(msg.header.stamp), points)

        def on_imu(self, msg):
            try:
                t = self.source_time(msg.header.stamp)
                if self.previous_imu is not None and t <= self.previous_imu:
                    raise ValueError('duplicate/backwards IMU stamp')
                key = self.mission.at(t)
                if key is None:
                    return
                if msg.header.frame_id != 'imu_link' or any(c[0] < 0 for c in (
                        msg.orientation_covariance, msg.angular_velocity_covariance, msg.linear_acceleration_covariance)):
                    raise ValueError('IMU fields/frame unavailable')
                tf = self.tf.lookup_transform('base_footprint', 'imu_link', Time())
                q = tf.transform.rotation
                # Đúng phạm vi adapter hiện tại: không tự đổi mounting/observation.
                if max(abs(q.x), abs(q.y), abs(q.z)) > 1e-5 or abs(abs(q.w)-1) > 1e-5:
                    raise ValueError('nonidentity IMU mounting unsupported')
                a, q = msg.linear_acceleration, msg.orientation
                values = [a.x, a.y, a.z, q.x, q.y, q.z, q.w]
                if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1) > .02:
                    raise ValueError('invalid IMU quaternion/data')
                acceleration, _, _ = imu_features(values[:3], values[3:])
                self.write('imu', dict(ros_stamp=t, goal_id=key, acceleration=acceleration.tolist()))
                self.previous_imu = t
                self.counts['imu'] += 1
            except Exception as exc:
                self.reject('imu: '+type(exc).__name__+': '+str(exc))

        def tick(self):
            try:
                if self.odom is None:
                    return
                odom = self.odom
                t = self.source_time(odom.header.stamp)
                if self.previous_pose is not None and t <= self.previous_pose:
                    return
                key = self.mission.at(t)
                if key is None:
                    return
                if odom.child_frame_id != 'base_footprint':
                    raise ValueError('wrong odometry body frame')
                if key not in self.references:
                    if self.path is None or self.path[0] < self.mission.goals[key]['start']:
                        raise ValueError('no new reference path for this goal')
                    points = self.path[1]
                    digest = hashlib.sha256(json.dumps(points, separators=(',', ':')).encode()).hexdigest()
                    self.references[key] = dict(frame='map', points=points, sha256=digest,
                                                policy='first valid plan for this goal, frozen')
                    self.save_summary()
                tf = self.tf.lookup_transform('map', 'base_footprint', Time.from_msg(odom.header.stamp))
                self.source_time(tf.header.stamp)
                p, q = tf.transform.translation, tf.transform.rotation
                if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
                    raise ValueError('nonfinite AMCL/odometry TF')
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                ey, heading, progress, remaining = path_features(p.x, p.y, yaw, self.references[key]['points'])
                row = dict(ros_stamp=t, goal_id=key, group=args.group, x=p.x, y=p.y, yaw=yaw,
                           cross_track=ey, heading_error=heading, progress=progress, remaining=remaining,
                           estimated_body_speed_mps=odom.twist.twist.linear.x)
                if self.wheels is not None and abs(stamp_s(self.wheels.header.stamp)-t) <= .1:
                    indices = [next((self.wheels.name.index(n) for n in names if n in self.wheels.name), None)
                               for names in [('left_wheel', 'left_wheel_joint'), ('right_wheel', 'right_wheel_joint')]]
                    if all(i is not None and i < len(self.wheels.velocity) for i in indices):
                        row['wheel_velocity'] = [self.wheels.velocity[i] for i in indices]
                if self.command and time.monotonic()-self.command[0] <= .25:
                    v, w = self.command[1].linear.x, self.command[1].angular.z
                    row['wheel_target'] = [(v-w*separation/2)/radius, (v+w*separation/2)/radius]
                if args.group != 'nav_only' and self.torque and time.monotonic()-self.torque[0] <= .25:
                    row['total'] = list(self.torque[1].data)
                self.write('pose', row)
                self.previous_pose = t
                self.counts['pose'] += 1
            except Exception as exc:
                self.reject('pose: '+type(exc).__name__+': '+str(exc))
            if time.monotonic()-self.last_flush >= 1:
                for f in self.files.values():
                    f.flush()
                self.last_flush = time.monotonic()

        def save_summary(self, closed=False):
            data = dict(format_version=1, group=args.group, closed=closed, goals=self.mission.goals,
                        references=self.references, counts=self.counts, rejected=self.rejected,
                        pose_source='AMCL + odometry TF map->base_footprint, estimated',
                        imu_source='/imu/bridge at source rate; gravity removed; identity mounting checked',
                        pose_rate_hz=10, wheel_target_source='unshaped kinematic safe velocity reference',
                        robot_config=str(robot_path.resolve()),
                        torque_source='controller torque preview snapshot; N/A for Nav-only')
            temp = folder/'summary.tmp'
            temp.write_text(json.dumps(data, ensure_ascii=False, indent=2))
            temp.replace(folder/'summary.json')

        def close(self):
            self.save_summary(closed=True)
            for f in self.files.values():
                f.close()

    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Logger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
