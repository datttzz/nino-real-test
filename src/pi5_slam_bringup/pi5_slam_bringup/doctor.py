"""Read-only health report. Bench never requires unconnected wheel or laser data."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import UInt8
from tf2_ros import Buffer, TransformListener


class Doctor(Node):
    def __init__(self):
        super().__init__('pi5_slam_doctor')
        self.samples = {}
        self.bad_imu = 0
        self.motor_status = None
        self.map_known_cells = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions_kept = []
        for topic, cls in [('/imu/data', Imu), ('/imu/bridge', Imu), ('/motor_status', UInt8),
                           ('/scan', LaserScan), ('/odom/unfiltered', Odometry),
                           ('/odom', Odometry), ('/map', OccupancyGrid)]:
            self.subscriptions_kept.append(self.create_subscription(
                cls, topic, lambda msg, name=topic: self.receive(name, msg),
                map_qos if topic == '/map' else qos_profile_sensor_data))

    def receive(self, topic, msg):
        item = self.samples.setdefault(topic, {'count': 0, 'first': time.monotonic()})
        item['count'] += 1
        item['last'] = time.monotonic()
        if topic == '/imu/data':
            q = msg.orientation
            values = [q.x, q.y, q.z, q.w, msg.angular_velocity.x,
                      msg.angular_velocity.y, msg.angular_velocity.z,
                      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[:4])-1) > .01:
                self.bad_imu += 1
        elif topic == '/motor_status':
            self.motor_status = msg.data
        elif topic == '/map':
            self.map_known_cells = sum(v >= 0 for v in msg.data)

    def report(self, mode):
        now = time.monotonic()
        required = ['/imu/data', '/imu/bridge', '/motor_status']
        if mode != 'bench':
            required += ['/scan', '/odom/unfiltered', '/odom', '/map']
        errors = []
        topics = {}
        for topic, item in self.samples.items():
            span = item['last'] - item['first']
            topics[topic] = {'count': item['count'], 'age_s': now-item['last'],
                             'rate_hz': (item['count']-1)/span if span > 0 else 0.0}
        for topic in required:
            if topic not in topics or topics[topic]['age_s'] > (10. if topic == '/map' else 1.):
                errors.append(f'Missing or stale {topic}')
        if self.bad_imu:
            errors.append('IMU has non-finite values or invalid quaternion norm')
        if mode != 'bench':
            if self.motor_status is None or self.motor_status & 0x03 != 3 or self.motor_status & 0x0C:
                errors.append('Both wheel CAN feedback flags must be fresh without bus-off')
            if self.map_known_cells == 0:
                errors.append('No observed cells in the map')
            for parent, child in [('odom', 'base_footprint'), ('map', 'odom')]:
                if not self.tf_buffer.can_transform(parent, child, rclpy.time.Time()):
                    errors.append(f'Missing transform {parent} -> {child}')
        return {'mode': mode, 'synthetic_data': mode == 'demo', 'ready': not errors,
                'errors': errors, 'motor_status': self.motor_status,
                'motors_enabled': bool((self.motor_status or 0) & 16),
                'map_known_cells': self.map_known_cells, 'invalid_imu_messages': self.bad_imu,
                'topics': topics}


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['bench', 'demo', 'mapping'], default='bench')
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--output')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Doctor()
    start = time.monotonic()
    try:
        while time.monotonic()-start < parsed.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        report = node.report(parsed.mode)
        output = json.dumps(report, indent=2)
        print(output)
        if parsed.output:
            Path(parsed.output).write_text(output+'\n')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if report['ready'] else 1)
