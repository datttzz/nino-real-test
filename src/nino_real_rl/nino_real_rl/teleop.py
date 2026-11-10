"""Teleop vận tốc STM32: manual arm, không chạy PI/SAC/Nav2 trên Pi."""
import json
from math import isfinite
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import SetBool

from .core import source_stamp


class TeleopGuard(Node):
    def __init__(self):
        super().__init__('teleop_guard')
        defaults=dict(max_linear_speed=.50,max_angular_speed=.40,
                      command_timeout=.30,sensor_timeout=.25,motor_status_timeout=.60,
                      input_topic='/teleop/cmd_vel')
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.cfg={key:self.get_parameter(key).value for key in defaults}
        for key in ('max_linear_speed','max_angular_speed','command_timeout','sensor_timeout','motor_status_timeout'):
            if not isfinite(self.cfg[key]) or self.cfg[key]<=0:raise ValueError('Invalid '+key)
        if self.cfg['input_topic'] in ('/cmd_vel','/cmd_torque','/cmd_mit'):
            raise ValueError('Teleop input must be separate from firmware command topics')
        self.armed=False;self.has_moved=False;self.fault=''
        self.arm_pending=False;self.arm_requested_at=None
        self.last_output=(0.,0.)
        self.command=(0.,0.);self.command_at=None
        self.status=0;self.status_at=None;self.joint_stamp=None;self.joint_at=None
        self.output=self.create_publisher(Twist,'/cmd_vel',1)
        self.disable=self.create_publisher(Bool,'/motor_enable',1)
        self.diag=self.create_publisher(String,'/teleop/status',1)
        self.create_subscription(Twist,self.cfg['input_topic'],self.on_command,1)
        self.create_subscription(UInt8,'/motor_status',self.on_status,10)
        self.create_subscription(JointState,'/motor_fb',self.on_joint,qos_profile_sensor_data)
        self.create_service(SetBool,'/teleop/arm',self.on_arm)
        self.create_timer(.05,self.tick)
        self.create_timer(.5,self.publish_status)
        self.get_logger().info('Teleop DISARMED; enable motors manually, then /teleop/arm. No Nav2 or SAC.')

    def ros_now(self):return self.get_clock().now().nanoseconds*1e-9

    def publish_command(self,linear,angular):
        message=Twist();message.linear.x=float(linear);message.angular.z=float(angular)
        self.output.publish(message)
        output=(float(linear),float(angular))
        if output!=self.last_output:
            self.get_logger().info(
                f'Teleop TX /cmd_vel: vx={linear:.3f} m/s, wz={angular:.3f} rad/s; '
                'published to STM32, not confirmation of wheel motion',throttle_duration_sec=.5)
        self.last_output=output

    def trip(self,reason,force_disable=False):
        was_armed=self.armed
        blocked=self.command
        self.armed=False;self.has_moved=False;self.command=(0.,0.)
        self.arm_pending=False;self.arm_requested_at=None
        self.fault=reason
        if was_armed or force_disable:
            self.publish_command(0.,0.)
            self.disable.publish(Bool(data=False))
        if was_armed or blocked!=(0.,0.):
            action='zero/disable requested' if was_armed or force_disable else 'no actuation while disarmed'
            self.get_logger().warning(
                f'Teleop DISARMED: {reason}; blocked vx={blocked[0]:.3f}, '
                f'wz={blocked[1]:.3f}; {action}',throttle_duration_sec=2.)

    def graph_error(self):
        if self.count_publishers('/cmd_torque') or self.count_publishers('/cmd_mit'):
            return 'Torque/MIT controller active; stop it before teleop'
        if self.count_publishers('/cmd_vel')!=1:
            return 'Another or undiscovered firmware velocity publisher'
        if self.count_subscribers('/cmd_vel')==0:
            return 'Firmware /cmd_vel subscriber not discovered'
        # ros2 topic echo cũng là subscriber; không được coi nó là firmware.
        firmware={(e.node_name,e.node_namespace)
                  for e in self.get_publishers_info_by_topic('/motor_status')
                  if e.topic_type=='std_msgs/msg/UInt8'}
        receivers={(e.node_name,e.node_namespace)
                   for e in self.get_subscriptions_info_by_topic('/cmd_vel')
                   if e.topic_type=='geometry_msgs/msg/Twist'}
        if not firmware.intersection(receivers):
            return 'Firmware /cmd_vel subscriber not discovered: must match the motor-status publisher node'
        count=self.count_publishers(self.cfg['input_topic'])
        if count!=1:
            return f'Need exactly one keyboard/input publisher; found {count}'
        return ''

    def basic_ready(self,now):
        # Status MCU chậm hơn encoder; dùng cùng ngưỡng 0,6 s như bridge.
        if self.status_at is None or not 0<=now-self.status_at<=self.cfg['motor_status_timeout']:
            raise ValueError('Missing/stale motor status')
        if self.status & 0x03 != 0x03 or self.status & 0x0C:
            raise ValueError('CAN feedback missing or bus-off')
        if not self.status & 0x10:raise ValueError('Motor disabled; enable manually first')
        if self.joint_at is None or not 0<=now-self.joint_at<=self.cfg['sensor_timeout']:
            raise ValueError('Missing/stale wheel feedback')
        source_stamp(self.joint_stamp,self.ros_now(),timeout=self.cfg['sensor_timeout'])
        if self.arm_pending and not 0<=now-self.arm_requested_at<=self.cfg['motor_status_timeout']:
            raise ValueError('Firmware velocity startup not ready: no fresh motor status with command timeout cleared')
        if self.has_moved and self.status & 0x20:raise ValueError('Firmware command timeout')

    def on_status(self,message):
        self.status=message.data;self.status_at=monotonic()
        if self.armed:
            try:
                self.basic_ready(self.status_at)
                # Chỉ mở chuyển động sau một callback mới; không dùng bit timeout
                # cũ lúc standby để ngắt ngay sau lệnh tiến đầu tiên.
                if self.arm_pending and not self.status & 0x20:
                    self.arm_pending=False
                    self.get_logger().info('Teleop READY: fresh motor status reports velocity timeout cleared')
            except ValueError as exc:self.trip(str(exc))

    def on_joint(self,message):
        try:
            stamp=source_stamp(message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                               self.ros_now(),self.joint_stamp,self.cfg['sensor_timeout'])
            names=list(message.name)
            left=next((names.index(n) for n in ('left_wheel_joint','left_wheel') if n in names),None)
            right=next((names.index(n) for n in ('right_wheel_joint','right_wheel') if n in names),None)
            if left is None or right is None or max(left,right)>=len(message.velocity):
                raise ValueError('Missing named wheel velocities')
            if not all(isfinite(message.velocity[i]) for i in (left,right)):
                raise ValueError('Nonfinite wheel feedback')
            self.joint_stamp=stamp;self.joint_at=monotonic()
        except ValueError as exc:
            self.joint_at=None;self.trip(str(exc))

    def on_command(self,message):
        now=monotonic()
        try:
            values=(message.linear.x,message.linear.y,message.linear.z,
                    message.angular.x,message.angular.y,message.angular.z)
            if not all(isfinite(v) for v in values):raise ValueError('Nonfinite keyboard command')
            if any(abs(v)>1e-9 for v in values[1:5]):raise ValueError('Only linear.x/angular.z supported')
            if self.armed and self.command_at is not None and now-self.command_at>self.cfg['command_timeout']:
                raise ValueError('Keyboard recovered after timeout; manual rearm required')
            self.command=(max(-self.cfg['max_linear_speed'],min(self.cfg['max_linear_speed'],values[0])),
                          max(-self.cfg['max_angular_speed'],min(self.cfg['max_angular_speed'],values[5])))
            self.command_at=now
            if self.command!=(0.,0.):
                self.get_logger().info(
                    f'Teleop RX: vx={self.command[0]:.3f} m/s, wz={self.command[1]:.3f} rad/s',
                    throttle_duration_sec=1.)
                if not self.armed:
                    self.get_logger().warning(
                        'Nonzero teleop blocked: DISARMED; enable motor and arm with zero input. '
                        f'Last fault: {self.fault or "none"}',throttle_duration_sec=2.)
                elif self.arm_pending:
                    self.get_logger().warning(
                        'Nonzero teleop held at zero: waiting for fresh STM32 velocity-ready status',
                        throttle_duration_sec=2.)
            if self.armed and self.command==(0.,0.):self.publish_command(0.,0.)
        except ValueError as exc:
            self.get_logger().warning('Teleop input rejected: '+str(exc),throttle_duration_sec=2.)
            self.trip(str(exc))

    def on_arm(self,request,response):
        if not request.data:
            self.trip('Operator disarmed',force_disable=True)
            response.success=True;response.message='Zero velocity and disable sent'
            return response
        try:
            now=monotonic();self.basic_ready(now)
            error=self.graph_error()
            if error:raise ValueError(error)
            if self.command_at is None or not 0<=now-self.command_at<=self.cfg['command_timeout']:
                raise ValueError('Start keyboard first; need fresh zero input')
            if self.command!=(0.,0.):raise ValueError('Press k/space and release motion keys before arming')
            self.armed=True;self.has_moved=False;self.fault=''
            self.arm_pending=True;self.arm_requested_at=now
            self.publish_command(0.,0.)
            response.success=True;response.message='Armed; sending zero until fresh STM32 velocity-ready status, then keyboard input'
            self.get_logger().info(response.message+'; motor enable is a separate service')
        except ValueError as exc:
            response.success=False;response.message=str(exc)
            self.get_logger().warning('Teleop arm rejected: '+str(exc),throttle_duration_sec=2.)
        return response

    def tick(self):
        if not self.armed:return
        try:
            now=monotonic();self.basic_ready(now)
            error=self.graph_error()
            if error:raise ValueError(error)
            if self.command_at is None or not 0<=now-self.command_at<=self.cfg['command_timeout']:
                raise ValueError('Keyboard input timeout')
            if self.arm_pending:
                self.publish_command(0.,0.)
                return
            if self.command!=(0.,0.):self.has_moved=True
            self.publish_command(*self.command)
        except ValueError as exc:self.trip(str(exc))

    def publish_status(self):
        now=monotonic()
        self.diag.publish(String(data=json.dumps(dict(armed=self.armed,fault=self.fault,
            command=list(self.command),max_linear_speed=self.cfg['max_linear_speed'],
            max_angular_speed=self.cfg['max_angular_speed'],
            motor_status_age_s=None if self.status_at is None else now-self.status_at,
            motor_status_timeout_s=self.cfg['motor_status_timeout'],
            ready=self.armed and not self.arm_pending,
            waiting_for_velocity_status=self.arm_pending,
            output_command=list(self.last_output),output_topic='/cmd_vel',
            keyboard_age_s=None if self.command_at is None else now-self.command_at,
            wheel_feedback_age_s=None if self.joint_at is None else now-self.joint_at,
            motor_status=self.status,motor_enabled=bool(self.status & 0x10),
            firmware_command_timeout=bool(self.status & 0x20),
            source='STM32 velocity control; no host PI/SAC'))))


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=TeleopGuard();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:
            if rclpy.ok():node.trip('Shutdown',force_disable=True)
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
