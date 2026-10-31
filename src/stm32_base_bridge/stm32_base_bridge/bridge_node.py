"""Adapt STM32H723 feedback. Modified for pi5-stm32-slam: freshness and enable gates."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, UInt8
from std_srvs.srv import SetBool

from .kinematics import Pose2D, integrate_wheel_positions, wheel_velocities_to_twist
from .feedback_gate import FeedbackGate


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class Stm32BaseBridge(Node):
    """Convert STM32 wheel/IMU feedback into topics consumed by Linorobot2."""

    def __init__(self) -> None:
        super().__init__('stm32_base_bridge')

        self.declare_parameter('wheel_radius', 0.060)
        self.declare_parameter('wheel_separation', 0.345)
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)
        self.declare_parameter('max_position_jump_rad', 4.0)
        self.declare_parameter('motor_feedback_topic', '/motor_fb')
        self.declare_parameter('imu_input_topic', '/imu/data')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('odom_topic', '/odom/unfiltered')
        self.declare_parameter('imu_output_topic', '/imu/bridge')
        self.declare_parameter('require_motor_feedback', True)
        self.declare_parameter('motor_status_timeout_s', 0.6)
        self.declare_parameter('allow_motor_enable', False)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('imu_frame_id', 'imu_link')
        # STM32 stamps are kept while within this window of the Pi clock.
        self.declare_parameter('stamp_tolerance_s', 0.05)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.left_sign = float(self.get_parameter('left_wheel_sign').value)
        self.right_sign = float(self.get_parameter('right_wheel_sign').value)
        self.max_position_jump = float(self.get_parameter('max_position_jump_rad').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.stamp_tolerance = float(self.get_parameter('stamp_tolerance_s').value)
        self.last_stamp = {}
        self.feedback_gate = FeedbackGate(float(self.get_parameter('motor_status_timeout_s').value))
        if self.feedback_gate.timeout_s <= 0:
            raise ValueError('motor_status_timeout_s must be positive')
        if self.get_parameter('imu_input_topic').value == self.get_parameter('imu_output_topic').value:
            raise ValueError('IMU input and output topics must differ')

        if self.wheel_radius <= 0.0 or self.wheel_separation <= 0.0:
            raise ValueError('wheel_radius and wheel_separation must be positive')

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.joint_state_publisher = self.create_publisher(JointState,str(self.get_parameter('joint_states_topic').value),SENSOR_QOS,)
        self.odom_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            odom_qos,
        )
        self.imu_publisher = self.create_publisher(
            Imu,
            str(self.get_parameter('imu_output_topic').value),
            SENSOR_QOS,
        )
        self.motor_enable_publisher = self.create_publisher(
            Bool, '/motor_enable', command_qos
        )

        self.create_subscription(
            JointState,
            str(self.get_parameter('motor_feedback_topic').value),
            self._motor_feedback_callback,
            SENSOR_QOS,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('imu_input_topic').value),
            self._imu_callback,
            SENSOR_QOS,
        )
        self.create_service(SetBool, '/set_motor_enable', self._set_motor_enable)
        self.create_subscription(UInt8, '/motor_status', self._motor_status_callback, 10)

        self.pose = Pose2D()
        self.last_left_position = None
        self.last_right_position = None
        self.warned_about_joint_names = False

        self.get_logger().info(
            'STM32 bridge ready: /motor_fb -> /joint_states + '
            f"/odom/unfiltered, {self.get_parameter('imu_input_topic').value} -> "
            f"{self.get_parameter('imu_output_topic').value} "
            f'(R={self.wheel_radius:.6f} m, W={self.wheel_separation:.3f} m)'
        )

    def _motor_status_callback(self, message: UInt8) -> None:
        self.feedback_gate.update(message.data, self.get_clock().now().nanoseconds * 1e-9)

    @staticmethod
    def _find_wheel_indices(names: list[str]) -> tuple[int, int] | None:
        left_aliases = ('left_wheel_joint', 'left_wheel')
        right_aliases = ('right_wheel_joint', 'right_wheel')
        left_index = next((names.index(name) for name in left_aliases if name in names), None)
        right_index = next((names.index(name) for name in right_aliases if name in names), None)
        if left_index is None or right_index is None:
            return None
        return left_index, right_index

    def _motor_feedback_callback(self, feedback: JointState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.get_parameter('require_motor_feedback').value and not self.feedback_gate.ready(now):
            self.last_left_position = self.last_right_position = None
            return
        indices = self._find_wheel_indices(list(feedback.name))
        if indices is None:
            if len(feedback.position) >= 2 and len(feedback.velocity) >= 2:
                indices = (0, 1)
                if not self.warned_about_joint_names:
                    self.get_logger().warning(
                        'Unknown motor joint names; using feedback array indices 0 and 1'
                    )
                    self.warned_about_joint_names = True
            else:
                self.get_logger().error(
                    '/motor_fb does not contain two wheel positions and velocities'
                )
                return

        left_index, right_index = indices
        if (
            left_index >= len(feedback.position)
            or right_index >= len(feedback.position)
            or left_index >= len(feedback.velocity)
            or right_index >= len(feedback.velocity)
        ):
            self.get_logger().error('/motor_fb arrays are shorter than the name array')
            return

        left_position = float(feedback.position[left_index]) * self.left_sign
        right_position = float(feedback.position[right_index]) * self.right_sign
        left_velocity = float(feedback.velocity[left_index]) * self.left_sign
        right_velocity = float(feedback.velocity[right_index]) * self.right_sign
        if not all(math.isfinite(v) for v in (left_position, right_position, left_velocity, right_velocity)):
            self.get_logger().error('Discarding non-finite wheel feedback', throttle_duration_sec=5.0)
            self.last_left_position = self.last_right_position = None
            return
        left_effort = (
            float(feedback.effort[left_index]) * self.left_sign
            if left_index < len(feedback.effort)
            else 0.0
        )
        right_effort = (
            float(feedback.effort[right_index]) * self.right_sign
            if right_index < len(feedback.effort)
            else 0.0
        )

        stamp = self._sanitize_stamp(feedback.header.stamp, 'motor')

        if self.last_left_position is not None and self.last_right_position is not None:
            left_delta = left_position - self.last_left_position
            right_delta = right_position - self.last_right_position
            if (
                abs(left_delta) <= self.max_position_jump
                and abs(right_delta) <= self.max_position_jump
            ):
                self.pose = integrate_wheel_positions(
                    self.pose,
                    left_delta,
                    right_delta,
                    self.wheel_radius,
                    self.wheel_separation,
                )
            else:
                self.get_logger().warning(
                    'Ignoring encoder position jump '
                    f'(left={left_delta:.3f}, right={right_delta:.3f} rad)'
                )

        self.last_left_position = left_position
        self.last_right_position = right_position

        self._publish_joint_states(
            stamp,
            left_position,
            right_position,
            left_velocity,
            right_velocity,
            left_effort,
            right_effort,
        )
        self._publish_odometry(stamp, left_velocity, right_velocity)

    def _publish_joint_states(
        self,
        stamp,
        left_position: float,
        right_position: float,
        left_velocity: float,
        right_velocity: float,
        left_effort: float,
        right_effort: float,
    ) -> None:
        output = JointState()
        output.header.stamp = stamp
        output.name = [
            'left_wheel_joint',
            'right_wheel_joint',
        ]
        output.position = [left_position, right_position]
        output.velocity = [left_velocity, right_velocity]
        output.effort = [left_effort, right_effort]
        self.joint_state_publisher.publish(output)

    def _publish_odometry(
        self, stamp, left_velocity: float, right_velocity: float
    ) -> None:
        linear_x, angular_z = wheel_velocities_to_twist(
            left_velocity,
            right_velocity,
            self.wheel_radius,
            self.wheel_separation,
        )

        output = Odometry()
        output.header.stamp = stamp
        output.header.frame_id = self.odom_frame_id
        output.child_frame_id = self.base_frame_id
        output.pose.pose.position.x = self.pose.x
        output.pose.pose.position.y = self.pose.y
        output.pose.pose.orientation.z = math.sin(0.5 * self.pose.yaw)
        output.pose.pose.orientation.w = math.cos(0.5 * self.pose.yaw)
        output.twist.twist.linear.x = linear_x
        output.twist.twist.angular.z = angular_z

        output.pose.covariance[0] = 0.02
        output.pose.covariance[7] = 0.02
        output.pose.covariance[14] = 1.0e6
        output.pose.covariance[21] = 1.0e6
        output.pose.covariance[28] = 1.0e6
        output.pose.covariance[35] = 0.05
        output.twist.covariance[0] = 0.02
        output.twist.covariance[7] = 0.02
        output.twist.covariance[14] = 1.0e6
        output.twist.covariance[21] = 1.0e6
        output.twist.covariance[28] = 1.0e6
        output.twist.covariance[35] = 0.05
        self.odom_publisher.publish(output)

    def _sanitize_stamp(self, stamp, stream: str):
        """Keep the STM32 sample time when it is plausible, and never go backwards.

        The micro-ROS clock sync can jump; a stamp in the future or older than
        stamp_tolerance_s is replaced by the receive time. Stamps on one stream are
        forced to increase so TF and the EKF never see data from the past.
        """
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns == 0 or stamp_ns > now_ns or now_ns - stamp_ns > self.stamp_tolerance * 1e9:
            stamp_ns = now_ns
        last_ns = self.last_stamp.get(stream)
        if last_ns is not None and stamp_ns <= last_ns:
            stamp_ns = last_ns + 1000
        self.last_stamp[stream] = stamp_ns
        return Time(nanoseconds=stamp_ns, clock_type=now.clock_type).to_msg()

    def _imu_callback(self, message: Imu) -> None:
        message.header.stamp = self._sanitize_stamp(message.header.stamp, 'imu')
        if not message.header.frame_id:
            message.header.frame_id = self.imu_frame_id
        self.imu_publisher.publish(message)

    def _set_motor_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data and not self.get_parameter('allow_motor_enable').value:
            response.success = False
            response.message = 'Motor enable is disabled by this launch configuration'
            return response
        command = Bool()
        command.data = bool(request.data)
        self.motor_enable_publisher.publish(command)
        response.success = True
        response.message = 'Motor enable command sent' if command.data else 'Motor disable command sent'
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Stm32BaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
