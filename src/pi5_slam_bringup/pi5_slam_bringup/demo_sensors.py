"""Synthetic room and encoders for testing SLAM without opening any hardware."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import UInt8

WALLS = [(-3., -3., 5., -3.), (5., -3., 5., 4.), (5., 4., -3., 4.),
         (-3., 4., -3., -3.), (2.5, 0.5, 3.0, 0.5), (3., .5, 3., 1.5),
         (3., 1.5, 2.5, 1.5), (2.5, 1.5, 2.5, .5)]


def ray_distance(x, y, angle, segments=WALLS, maximum=12.0):
    dx, dy = math.cos(angle), math.sin(angle)
    distance = maximum
    for x1, y1, x2, y2 in segments:
        sx, sy = x2-x1, y2-y1
        denominator = dx*sy - dy*sx
        if abs(denominator) < 1e-10:
            continue
        qx, qy = x1-x, y1-y
        t = (qx*sy - qy*sx) / denominator
        u = (qx*dy - qy*dx) / denominator
        if t >= 0 and 0 <= u <= 1:
            distance = min(distance, t)
    return distance


class DemoSensors(Node):
    def __init__(self):
        super().__init__('synthetic_demo_sensors')
        for key, default in [('wheel_radius', .062455), ('wheel_separation', .345),
                             ('laser_x', .223), ('laser_y', 0.), ('laser_yaw', 0.)]:
            self.declare_parameter(key, default)
        self.radius = self.get_parameter('wheel_radius').value
        self.separation = self.get_parameter('wheel_separation').value
        self.start = self.get_clock().now().nanoseconds * 1e-9
        self.fb = self.create_publisher(JointState, '/motor_fb', qos_profile_sensor_data)
        self.imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.status = self.create_publisher(UInt8, '/motor_status', 10)
        self.create_timer(.02, self.publish_sensors)
        self.create_timer(.1, self.publish_scan)
        self.create_timer(.2, lambda: self.status.publish(UInt8(data=3)))
        self.get_logger().info('SYNTHETIC sensor data in an artificial room; no motor commands are published.')

    def pose(self):
        # A small circle remains inside the synthetic room. Wall time is shared
        # by encoders and ray casting so the two inputs describe the same motion.
        t = self.get_clock().now().nanoseconds * 1e-9 - self.start
        velocity, omega = .12, .16
        yaw = omega*t
        return t, velocity/omega*math.sin(yaw), velocity/omega*(1-math.cos(yaw)), yaw, velocity, omega

    def publish_sensors(self):
        t, _, _, yaw, velocity, omega = self.pose()
        stamp = self.get_clock().now().to_msg()
        left = (velocity - omega*self.separation/2)/self.radius
        right = (velocity + omega*self.separation/2)/self.radius
        fb = JointState()
        fb.header.stamp, fb.header.frame_id = stamp, 'base_link'
        fb.name = ['left_wheel', 'right_wheel']
        fb.position, fb.velocity, fb.effort = [left*t, right*t], [left, right], [0., 0.]
        self.fb.publish(fb)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
        imu.orientation.z, imu.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        imu.angular_velocity.z = omega
        imu.linear_acceleration.z = 9.81
        imu.linear_acceleration.y = velocity*omega
        imu.angular_velocity_covariance = [.00025, 0., 0., 0., .00025, 0., 0., 0., .00025]
        imu.linear_acceleration_covariance = [.01, 0., 0., 0., .01, 0., 0., 0., .01]
        imu.orientation_covariance = [.001, 0., 0., 0., .001, 0., 0., 0., 1e6]
        self.imu.publish(imu)

    def publish_scan(self):
        _, x, y, yaw, _, _ = self.pose()
        lx, ly, lyaw = [self.get_parameter(k).value for k in ('laser_x', 'laser_y', 'laser_yaw')]
        sx, sy = x+math.cos(yaw)*lx-math.sin(yaw)*ly, y+math.sin(yaw)*lx+math.cos(yaw)*ly
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = self.get_clock().now().to_msg(), 'laser'
        scan.angle_min, scan.angle_increment = -math.pi, math.tau/360
        scan.angle_max = scan.angle_min + 359*scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .15, 12., .1
        # Ideal simultaneous rays; do not invent per-ray acquisition delays.
        scan.time_increment = 0.
        scan.ranges = [ray_distance(sx, sy, yaw+lyaw+scan.angle_min+i*scan.angle_increment)
                       for i in range(360)]
        self.scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DemoSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
