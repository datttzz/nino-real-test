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
        defaults=dict(max_linear_speed=.10,max_angular_speed=.40,
                      command_timeout=.50,sensor_timeout=.25,
                      input_topic='/teleop/cmd_vel')
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.cfg={key:self.get_parameter(key).value for key in defaults}
        for key in ('max_linear_speed','max_angular_speed','command_timeout','sensor_timeout'):
            if not isfinite(self.cfg[key]) or self.cfg[key]<=0:raise ValueError('Invalid '+key)
        if self.cfg['input_topic'] in ('/cmd_vel','/cmd_torque','/cmd_mit'):
            raise ValueError('Teleop input must be separate from firmware command topics')
        self.armed=False;self.has_moved=False;self.fault=''
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

    def trip(self,reason,force_disable=False):
        was_armed=self.armed
        self.armed=False;self.has_moved=False;self.command=(0.,0.)
        self.fault=reason
        if was_armed or force_disable:
            self.publish_command(0.,0.)
            self.disable.publish(Bool(data=False))
        if was_armed:self.get_logger().error('Teleop DISARMED: '+reason)

    def graph_error(self):
        if self.count_publishers('/cmd_torque') or self.count_publishers('/cmd_mit'):
            return 'Torque/MIT controller active; stop it before teleop'
        if self.count_publishers('/cmd_vel')!=1:
            return 'Another or undiscovered firmware velocity publisher'
        if self.count_subscribers('/cmd_vel')==0:
            return 'Firmware /cmd_vel subscriber not discovered'
        if self.count_publishers(self.cfg['input_topic'])!=1:
            return 'Need exactly one keyboard/input publisher'
        return ''

    def basic_ready(self,now):
        if self.status_at is None or not 0<=now-self.status_at<=self.cfg['sensor_timeout']:
            raise ValueError('Missing/stale motor status')
        if self.status & 0x03 != 0x03 or self.status & 0x0C:
            raise ValueError('CAN feedback missing or bus-off')
        if not self.status & 0x10:raise ValueError('Motor disabled; enable manually first')
        if self.joint_at is None or not 0<=now-self.joint_at<=self.cfg['sensor_timeout']:
            raise ValueError('Missing/stale wheel feedback')
        source_stamp(self.joint_stamp,self.ros_now(),timeout=self.cfg['sensor_timeout'])
        if self.has_moved and self.status & 0x20:raise ValueError('Firmware command timeout')

    def on_status(self,message):
        self.status=message.data;self.status_at=monotonic()
        if self.armed:
            try:self.basic_ready(self.status_at)
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
            if self.armed and self.command==(0.,0.):self.publish_command(0.,0.)
        except ValueError as exc:self.trip(str(exc))

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
            response.success=True;response.message='Armed; motion requires keyboard input'
        except ValueError as exc:
            response.success=False;response.message=str(exc)
        return response

    def tick(self):
        if not self.armed:return
        try:
            now=monotonic();self.basic_ready(now)
            error=self.graph_error()
            if error:raise ValueError(error)
            if self.command_at is None or not 0<=now-self.command_at<=self.cfg['command_timeout']:
                raise ValueError('Keyboard input timeout')
            if self.command!=(0.,0.):self.has_moved=True
            self.publish_command(*self.command)
        except ValueError as exc:self.trip(str(exc))

    def publish_status(self):
        self.diag.publish(String(data=json.dumps(dict(armed=self.armed,fault=self.fault,
            command=list(self.command),max_linear_speed=self.cfg['max_linear_speed'],
            max_angular_speed=self.cfg['max_angular_speed'],source='STM32 velocity control; no host PI/SAC'))))


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
