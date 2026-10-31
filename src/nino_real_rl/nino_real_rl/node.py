"""Hardware adapter; owns no odometry/TF and never learns online."""
import json
from math import atan2
from pathlib import Path
from time import monotonic

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, UInt8, String, Float64MultiArray
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from .core import Contract, WheelPI, observation, source_stamp, hardware_contract
from .policy import Policy


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class RealController(Node):
    def __init__(self):
        super().__init__('real_controller')
        defaults = dict(mode='shadow', checkpoint='', hardware_file='',
                        sensor_timeout=.25, command_timeout=.25, path_timeout=2.,
                        max_sample_skew=.10, joint_min_rate_hz=30.,
                        reference_topic='/rl/cmd_vel_reference', path_topic='/plan',
                        map_frame='map', base_frame='base_footprint', imu_frame='imu_link',
                        wheel_radius=.060, wheel_separation=.345,
                        left_feedback_sign=1., right_feedback_sign=1., log_dir='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        self.mode = self.cfg['mode']
        if self.mode not in ('shadow', 'pi', 'pi_sac'):
            raise ValueError('mode must be shadow, pi or pi_sac')
        if self.cfg['reference_topic'] in ('/cmd_vel', '/cmd_torque', '/cmd_mit'):
            raise ValueError('Reference must not be a firmware command topic')
        for k in ('sensor_timeout','command_timeout','path_timeout','max_sample_skew','joint_min_rate_hz'):
            if not np.isfinite(self.cfg[k]) or self.cfg[k] <= 0:
                raise ValueError('Invalid '+k)
        for k in ('left_feedback_sign','right_feedback_sign'):
            if self.cfg[k] not in (-1.,1.):
                raise ValueError('Invalid '+k)
        self.policy = Policy(self.cfg['checkpoint']) if self.cfg['checkpoint'] else None
        if self.mode == 'pi_sac' and self.policy is None:
            raise ValueError('PI+SAC requires imported verified 50k checkpoint bundle')
        self.contract = self.policy.contract if self.policy else Contract()
        self.hardware = None
        if self.mode != 'shadow':
            self.hardware = hardware_contract(json.loads(Path(self.cfg['hardware_file']).expanduser().read_text()))
        self.pi = WheelPI(self.cfg['wheel_radius'], self.cfg['wheel_separation'])
        self.armed = False
        self.fault = ''
        self.action = np.zeros(2)
        self.action_at = 0.
        self.command = np.zeros(2)
        self.command_at = 0.
        self.samples = {}
        self.status = 0
        self.status_at = 0.
        self.joint_stamp = None
        self.joint_at = 0.
        self.joint_intervals = []
        self.path = None
        self.path_at = 0.
        self.last_actor = 0.
        self.last_log = 0.
        self.last_diag = 0.
        self.wheel_velocity = np.zeros(2)
        self.moving_since = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        qos = qos_profile_sensor_data
        self.create_subscription(JointState, '/motor_fb', self.on_joint, qos)
        self.create_subscription(Imu, '/imu/bridge', lambda m: self.on_sample('imu', m), qos)
        self.create_subscription(Odometry, '/odom', lambda m: self.on_sample('odom', m), qos)
        self.create_subscription(UInt8, '/motor_status', self.on_status, 10)
        self.create_subscription(Twist, self.cfg['reference_topic'], self.on_command, 1)
        self.create_subscription(RosPath, self.cfg['path_topic'], self.on_path, 1)
        # Shadow does not create publishers on ANY firmware command topic.
        self.output = self.create_publisher(Float64MultiArray, '/cmd_torque', 1) if self.mode != 'shadow' else None
        self.disable = self.create_publisher(Bool, '/motor_enable', 1) if self.mode != 'shadow' else None
        self.diag = self.create_publisher(String, '/rl/status', 1)
        self.preview = self.create_publisher(Float64MultiArray, '/rl/torque_preview', 1)
        self.create_service(SetBool, '/rl/arm', self.on_arm)
        self.timer = self.create_timer(.02, self.tick)
        self.log = None
        if self.cfg['log_dir']:
            folder = Path(self.cfg['log_dir']).expanduser(); folder.mkdir(parents=True, exist_ok=True)
            self.log = (folder/f'real_{self.mode}_{self.get_clock().now().nanoseconds}.jsonl').open('x')
        self.get_logger().info(f'{self.mode}: DISARMED; feedback-clocked PI, actor {1/self.contract.step_seconds:g} Hz; '
                               + ('checkpoint loaded' if self.policy else 'NO CHECKPOINT: data/PI only'))

    def ros_now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_status(self, msg):
        self.status, self.status_at = msg.data, monotonic()
        if self.armed and (msg.data & 0x0C or not msg.data & 0x10):
            self.trip('Motor disabled or CAN bus-off')

    def on_sample(self, key, msg):
        try:
            old = self.samples.get(key)
            source_stamp(seconds(msg.header.stamp), self.ros_now(),
                         seconds(old[0].header.stamp) if old else None, self.cfg['sensor_timeout'])
            self.samples[key] = (msg, monotonic())
        except ValueError as exc:
            self.samples.pop(key, None)
            self.trip(key+': '+str(exc))

    def on_command(self, msg):
        values = np.array([msg.linear.x, msg.angular.z])
        if not np.all(np.isfinite(values)):
            self.trip('Invalid velocity reference'); return
        self.command = values
        self.command_at = monotonic()
        if np.max(np.abs(values)) < 1e-9:
            self.reset_control()
            self.send_zero()

    def on_path(self, msg):
        if msg.header.frame_id != self.cfg['map_frame'] or len(msg.poses) < 2:
            self.path = None; self.trip('Path must contain >=2 points in map'); return
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not np.all(np.isfinite(points)):
            self.path = None; self.trip('Nonfinite path'); return
        if any(p.header.frame_id not in ('', msg.header.frame_id) for p in msg.poses):
            self.path = None; self.trip('Mixed path frames'); return
        self.path, self.path_at = points, monotonic()

    def graph_error(self):
        if self.count_publishers('/cmd_vel'):
            return 'Conflicting /cmd_vel publisher (firmware velocity mode)'
        if self.count_publishers('/cmd_mit'):
            return 'Conflicting /cmd_mit publisher'
        expected = 1 if self.output is not None else 0
        if self.count_publishers('/cmd_torque') != expected:
            return 'Conflicting or undiscovered /cmd_torque publisher'
        if self.output is not None and self.count_subscribers('/cmd_torque') == 0:
            return 'Firmware /cmd_torque subscriber not discovered'
        return ''

    def basic_ready(self, now, enabled=False):
        if now-self.status_at > self.cfg['sensor_timeout'] or self.status & 0x03 != 0x03 or self.status & 0x0C:
            raise ValueError('Missing/faulted motor status or CAN feedback')
        if enabled and not self.status & 0x10:
            raise ValueError('Motor is disabled; controller never enables it automatically')
        if self.joint_stamp is None or now-self.joint_at > self.cfg['sensor_timeout']:
            raise ValueError('Missing/stale wheel feedback')
        source_stamp(self.joint_stamp, self.ros_now(), timeout=self.cfg['sensor_timeout'])
        if len(self.joint_intervals) < 10 or 1/np.mean(self.joint_intervals) < self.cfg['joint_min_rate_hz']:
            raise ValueError('Insufficient measured feedback rate')

    def on_arm(self, request, response):
        if not request.data:
            self.trip('Operator disarmed')
            response.success = True; response.message = 'Disarmed; motor disable sent in live modes'
            return response
        try:
            if self.mode == 'shadow':
                raise ValueError('Shadow cannot arm or actuate')
            hardware_contract(self.hardware)
            error = self.graph_error()
            if error: raise ValueError(error)
            self.basic_ready(monotonic(), enabled=True)
            # Validate IMU mounting for all modes; essential to future SAC comparison.
            self.imu_in_base()
            self.reset_control()
            self.armed = True; self.fault = ''
            response.success = True
            response.message = 'Armed. Motion still requires fresh safe reference; PI+SAC also requires valid observation.'
        except (ValueError, KeyError, RuntimeError) as exc:
            response.success = False; response.message = str(exc)
        return response

    def reset_control(self):
        self.pi.reset(); self.action[:] = 0
        self.action_at = 0.; self.moving_since = None

    def send_zero(self):
        if self.output is not None and self.armed:
            self.output.publish(Float64MultiArray(data=[0.,0.]))

    def trip(self, reason):
        was_armed = self.armed
        self.send_zero()
        self.armed = False
        self.reset_control()
        self.fault = reason
        if was_armed:
            self.disable.publish(Bool(data=False))
            self.get_logger().error('DISARMED: '+reason)

    def imu_in_base(self):
        if 'imu' not in self.samples:
            raise ValueError('Missing IMU')
        imu, received = self.samples['imu']
        if monotonic()-received > self.cfg['sensor_timeout']:
            raise ValueError('Stale IMU')
        source_stamp(seconds(imu.header.stamp), self.ros_now(), timeout=self.cfg['sensor_timeout'])
        if imu.header.frame_id != self.cfg['imu_frame'] or any(
                c[0] < 0 for c in (imu.orientation_covariance, imu.angular_velocity_covariance,
                                   imu.linear_acceleration_covariance)):
            raise ValueError('IMU frame or orientation/gyro/acceleration unavailable')
        # Supplied robot has identity IMU rotation. Reject changed mounting rather
        # than silently feeding sensor-frame features into a body-frame policy.
        try:
            t = self.tf.lookup_transform(self.cfg['base_frame'], self.cfg['imu_frame'], Time())
        except Exception as exc:
            raise ValueError('Missing IMU mounting TF: '+str(exc)) from exc
        q = t.transform.rotation
        if max(abs(q.x),abs(q.y),abs(q.z)) > 1e-5 or abs(abs(q.w)-1) > 1e-5:
            raise ValueError('Nonidentity IMU mounting needs an explicit feature transform; unsupported')
        a, q, w = imu.linear_acceleration, imu.orientation, imu.angular_velocity
        return [a.x,a.y,a.z], [q.x,q.y,q.z,q.w], [w.x,w.y,w.z]

    def observe(self):
        now = monotonic()
        if self.path is None or now-self.path_at > self.cfg['path_timeout']:
            raise ValueError('Missing/stale Nav2 global path')
        if now-self.command_at > self.cfg['command_timeout']:
            raise ValueError('Missing/stale safe velocity reference')
        if 'odom' not in self.samples:
            raise ValueError('Missing odometry')
        odom, received = self.samples['odom']
        if now-received > self.cfg['sensor_timeout'] or odom.child_frame_id != self.cfg['base_frame']:
            raise ValueError('Stale odometry or wrong body frame')
        source_stamp(seconds(odom.header.stamp), self.ros_now(), timeout=self.cfg['sensor_timeout'])
        acceleration, quaternion, gyro = self.imu_in_base()
        stamps = [seconds(self.samples['imu'][0].header.stamp), seconds(odom.header.stamp), self.joint_stamp]
        if max(stamps)-min(stamps) > self.cfg['max_sample_skew']:
            raise ValueError('Sensor timestamp skew exceeds configured limit')
        # Pose at odometry sample time, not an unrelated latest AMCL pose.
        try:
            tf = self.tf.lookup_transform(self.cfg['map_frame'], self.cfg['base_frame'],
                                          Time.from_msg(odom.header.stamp))
        except Exception as exc:
            raise ValueError('Map pose TF unavailable at sensor time: '+str(exc)) from exc
        source_stamp(seconds(tf.header.stamp), self.ros_now(), timeout=self.cfg['sensor_timeout'])
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        v = odom.twist.twist
        return observation(self.contract, (p.x,p.y,yaw), self.path, acceleration,
                           quaternion, gyro, v.linear.x, v.angular.z, self.command,
                           self.wheel_velocity, self.pi.base, self.action, self.pi.accepted)

    def on_joint(self, msg):
        now = monotonic()
        try:
            stamp = source_stamp(seconds(msg.header.stamp), self.ros_now(), self.joint_stamp,
                                 self.cfg['sensor_timeout'])
            names = list(msg.name)
            left = next((names.index(n) for n in ('left_wheel_joint','left_wheel') if n in names), None)
            right = next((names.index(n) for n in ('right_wheel_joint','right_wheel') if n in names), None)
            if left is None or right is None or max(left,right) >= len(msg.velocity):
                raise ValueError('Motor feedback must identify both wheels')
            wheels = np.array([msg.velocity[left]*self.cfg['left_feedback_sign'],
                               msg.velocity[right]*self.cfg['right_feedback_sign']])
            if not np.all(np.isfinite(wheels)):
                raise ValueError('Nonfinite wheel feedback')
            dt = stamp-self.joint_stamp if self.joint_stamp is not None else None
            self.joint_stamp, self.joint_at, self.wheel_velocity = stamp, now, wheels
            if dt is None: return
            if not 0 < dt <= self.cfg['sensor_timeout']:
                self.joint_intervals.clear(); raise ValueError('Wheel feedback gap')
            self.joint_intervals = (self.joint_intervals+[dt])[-50:]
            self.basic_ready(now, enabled=self.armed)
            if self.mode != 'shadow' and not self.armed:
                self.reset_control(); return
            error = self.graph_error()
            if error: raise ValueError(error)
            if now-self.command_at > self.cfg['command_timeout']:
                if self.armed and self.moving_since is not None:
                    raise ValueError('Safe velocity reference timed out during motion')
                self.reset_control(); self.send_zero(); return
            residual = np.zeros(2)
            if self.policy is not None and self.mode in ('shadow','pi_sac') and np.max(np.abs(self.command)) > 1e-9:
                if self.action_at == 0.:
                    self.pi.reset(); self.send_zero(); return  # Wait for first valid actor output.
                if now-self.action_at > max(.2, 2*self.contract.step_seconds):
                    raise ValueError('Missing/stale policy action')
                residual = self.action*self.contract.residual_limit
            total = self.pi.update(self.command, wheels, residual, dt)
            self.preview.publish(Float64MultiArray(data=total.tolist()))
            if self.armed:
                signs = np.array([self.hardware['left_command_sign'], self.hardware['right_command_sign']])
                if np.max(np.abs(total)) > 1e-9 and self.moving_since is None:
                    self.moving_since = now
                self.output.publish(Float64MultiArray(data=(total*signs).tolist()))
        except (ValueError, KeyError, IndexError) as exc:
            self.trip(str(exc))

    def tick(self):
        now = monotonic()
        if self.armed:
            try:
                self.basic_ready(now, enabled=True)
                error = self.graph_error()
                if error: raise ValueError(error)
                if now-self.command_at > self.cfg['command_timeout']:
                    if self.moving_since is not None:
                        raise ValueError('Safe velocity reference timed out during motion')
                    self.reset_control(); self.send_zero()
            except ValueError as exc:
                self.trip(str(exc))
        try:
            if now-self.last_actor >= self.contract.step_seconds:
                self.last_actor = now
                standby = self.armed and self.mode == 'pi_sac' and (
                    now-self.command_at > self.cfg['command_timeout'] or np.max(np.abs(self.command)) < 1e-9)
                if standby:
                    # Arming before the first goal is valid. Never generate torque
                    # while waiting, and do not demand a path before a goal exists.
                    self.reset_control(); self.send_zero()
                    self.fault = 'Waiting for a fresh nonzero navigation reference'
                else:
                    self.basic_ready(now, enabled=self.armed)
                    obs, metrics = self.observe()
                    if self.mode == 'pi_sac' and abs(metrics['cross_track']) > self.contract.max_path_deviation:
                        raise ValueError('AMCL cross-track exceeds checkpoint corridor')
                    if self.policy is not None and self.mode in ('shadow','pi_sac'):
                        self.action = self.policy.predict(obs)
                        self.action_at = now
                    self.fault = ''
                    if self.log:
                        self.log.write(json.dumps(dict(ros_stamp=self.ros_now(), mode=self.mode, armed=self.armed,
                            has_checkpoint=self.policy is not None, observation=obs.tolist(),
                            action=self.action.tolist(), base=self.pi.base.tolist(),
                            accepted_residual=self.pi.accepted.tolist(), total=self.pi.total.tolist(),
                            wheel_velocity=self.wheel_velocity.tolist(), wheel_target=self.pi.target.tolist(),
                            **metrics), allow_nan=False)+'\n')
                        if now-self.last_log > 1: self.log.flush(); self.last_log = now
        except (ValueError, KeyError, IndexError, RuntimeError) as exc:
            # PI does not depend on AMCL/path observations, but all live modes
            # still check encoder freshness/status/ownership in on_joint/tick.
            if self.mode != 'pi': self.trip(str(exc))
            else: self.fault = 'Observation only: '+str(exc)
        if now-self.last_diag >= .5:
            self.last_diag = now
            self.diag.publish(String(data=json.dumps(dict(mode=self.mode, armed=self.armed,
                has_checkpoint=self.policy is not None, fault=self.fault,
                measured_feedback_hz=1/np.mean(self.joint_intervals) if self.joint_intervals else 0.,
                training_control_hz=1000., actor_hz=1/self.contract.step_seconds))))

    def close(self):
        self.trip('Shutdown')
        if self.log: self.log.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok(): node.close()
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
