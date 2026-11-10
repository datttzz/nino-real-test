"""Controller integration on isolated localhost DDS; all hardware is test fixtures."""
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]

CODE = r'''
import json, tempfile, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8,Float64MultiArray
from std_srvs.srv import SetBool
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from nino_real_rl.node import RealController

with tempfile.TemporaryDirectory(prefix='test_only_no_robot_') as tmp:
    # Test fixture only, never written to the user's hardware.local.json.
    hardware=Path(tmp)/'test_only_hardware.json'
    data={k:True for k in ['firmware_torque_verified','wheel_torque_nm_verified',
        'watchdog_verified','feedback_signs_verified','accept_feedback_rate_difference']}
    data.update(command_topic='/cmd_torque',left_command_sign=1,right_command_sign=1,
        verified_wheel_torque_limit_nm=2.,verified_watchdog_seconds=.404)
    hardware.write_text(json.dumps(data))
    rclpy.init(args=['--ros-args','-p','mode:=pi','-p','hardware_file:='+str(hardware)])
    fixture=Node('test_only_sensor_fixtures')
    raw=fixture.create_publisher(Imu,'/imu/data',qos_profile_sensor_data)
    wheels=fixture.create_publisher(JointState,'/motor_fb',qos_profile_sensor_data)
    odom=fixture.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
    status=fixture.create_publisher(UInt8,'/motor_status',10)
    commands=[]
    sink=fixture.create_subscription(Float64MultiArray,'/cmd_torque',
        lambda m:commands.append(list(m.data)),qos_profile_sensor_data)
    mounting=StaticTransformBroadcaster(fixture)
    tf=TransformStamped();tf.header.frame_id='base_footprint';tf.child_frame_id='imu_link'
    tf.transform.rotation.w=1.;tf.header.stamp=fixture.get_clock().now().to_msg()
    mounting.sendTransform(tf)
    enabled_imu=[True]
    def publish():
        stamp=fixture.get_clock().now().to_msg()
        j=JointState();j.header.stamp=stamp;j.header.frame_id='base_link'
        j.name=['left_wheel','right_wheel'];j.position=[0.,0.];j.velocity=[0.,0.]
        wheels.publish(j)
        o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_footprint'
        o.pose.pose.orientation.w=1.;odom.publish(o)
        status.publish(UInt8(data=0x33))
        if enabled_imu[0]:
            m=Imu();m.header.stamp=stamp;m.header.frame_id='imu_link'
            m.orientation.w=1.;m.linear_acceleration.z=9.81;raw.publish(m)
    timer=fixture.create_timer(.02,publish)
    controller=RealController()
    executor=SingleThreadedExecutor();executor.add_node(fixture);executor.add_node(controller)
    try:
        end=time.monotonic()+2.
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        assert controller.cfg['imu_topic']=='/imu/data'
        assert controller.count_publishers('/imu/bridge')==0
        client=fixture.create_client(SetBool,'/rl/arm')
        assert client.wait_for_service(timeout_sec=2)
        req=SetBool.Request();req.data=True;future=client.call_async(req)
        end=time.monotonic()+2.
        while not future.done() and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        assert future.done() and future.result().success,str(future.result())
        end=time.monotonic()+2.
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)
            assert controller.armed,controller.safety_fault
        assert commands and all(c==[0.,0.] for c in commands)
        enabled_imu[0]=False
        end=time.monotonic()+1.
        while controller.armed and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        assert not controller.armed
        assert 'imu:' in controller.safety_fault,controller.safety_fault
        print('PASS: real controller stays armed at zero with raw IMU and no bridge; stops when actual IMU cache expires')
    finally:
        controller.close();executor.remove_node(controller);executor.remove_node(fixture)
        controller.destroy_node();fixture.destroy_node();executor.shutdown();rclpy.shutdown()
'''


def test_raw_imu_standby_and_loss_on_isolated_ros_domain():
    pytest.importorskip('rclpy')
    env = dict(os.environ, ROS_DOMAIN_ID='218', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               PYTHONPATH=str(ROOT/'src/nino_real_rl')+':'+os.environ.get('PYTHONPATH',''))
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    env.pop('FASTDDS_DEFAULT_PROFILES_FILE', None)
    r = subprocess.run(['/usr/bin/python3','-c',CODE],env=env,capture_output=True,text=True,timeout=20)
    assert r.returncode == 0, r.stdout+'\n'+r.stderr
    assert 'PASS:' in r.stdout
