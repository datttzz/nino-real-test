"""Real ROS message classes with mocked publishers; never initialize DDS/motors."""
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import importlib.util
from pathlib import Path

import pytest
pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool
from rclpy.time import Time
from nino_real_rl.teleop import TeleopGuard


def guard(armed=False):
    n=object.__new__(TeleopGuard)
    n.cfg=dict(max_linear_speed=.1,max_angular_speed=.4,command_timeout=.3,sensor_timeout=.25,input_topic='/teleop/cmd_vel')
    n.armed=armed;n.has_moved=False;n.fault='';n.command=(0.,0.);n.command_at=100.
    n.status=0x13;n.status_at=100.;n.joint_at=100.;n.joint_stamp=10.
    n.ros_now=lambda:10.
    n.output=Mock();n.disable=Mock();n.diag=Mock();n.get_logger=Mock()
    n.count_publishers=lambda topic:1 if topic in ('/cmd_vel','/teleop/cmd_vel') else 0
    n.count_subscribers=lambda topic:1
    return n


def command(v,w=0.):
    m=Twist();m.linear.x=v;m.angular.z=w;return m


def test_starts_disarmed_and_requires_zero_before_arm():
    n=guard();n.tick();n.output.publish.assert_not_called();n.disable.publish.assert_not_called()
    n.command=(.1,0.)
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):response=n.on_arm(SetBool.Request(data=True),SetBool.Response())
    assert not response.success and not n.armed
    n.command=(0.,0.)
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):response=n.on_arm(SetBool.Request(data=True),SetBool.Response())
    assert response.success and n.armed
    n.disable.publish.assert_not_called()


def test_clamps_speed_and_never_enables_motor_automatically():
    n=guard(True)
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):n.on_command(command(9.,9.));n.tick()
    message=n.output.publish.call_args.args[0]
    assert message.linear.x==.1 and message.angular.z==.4
    n.disable.publish.assert_not_called()


def test_command_timeout_disarms_and_recovery_cannot_resume():
    n=guard(True);n.command=(.1,0.);n.has_moved=True
    with patch('nino_real_rl.teleop.monotonic',return_value=100.31):n.tick()
    assert not n.armed and n.output.publish.call_args.args[0].linear.x==0.
    assert n.disable.publish.call_args.args[0].data is False
    before=n.output.publish.call_count
    with patch('nino_real_rl.teleop.monotonic',return_value=100.32):n.on_command(command(.1));n.tick()
    assert not n.armed and n.output.publish.call_count==before


def test_late_command_cannot_hide_a_timeout_from_timer():
    n=guard(True)
    with patch('nino_real_rl.teleop.monotonic',return_value=100.31):n.on_command(command(.1))
    assert not n.armed and n.command==(0.,0.)


@pytest.mark.parametrize('status',[0x33,0x17,0x03])
def test_mcu_timeout_busoff_and_disable_latch_stop(status):
    n=guard(True);n.has_moved=True
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):n.on_status(UInt8(data=status))
    assert not n.armed and n.disable.publish.call_args.args[0].data is False
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):n.on_status(UInt8(data=0x13))
    assert not n.armed


@pytest.mark.parametrize('topic',['/cmd_torque','/cmd_mit','/cmd_vel','/teleop/cmd_vel'])
def test_conflicting_publishers_block_arm(topic):
    n=guard();original=n.count_publishers
    n.count_publishers=lambda name:original(name)+1 if name==topic else original(name)
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):response=n.on_arm(SetBool.Request(data=True),SetBool.Response())
    assert not response.success and not n.armed
    n.output.publish.assert_not_called()


def test_missing_firmware_subscriber_and_disabled_motor_block_arm():
    n=guard();n.count_subscribers=lambda _:0
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):response=n.on_arm(SetBool.Request(data=True),SetBool.Response())
    assert not response.success
    n=guard();n.status=0x03
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):response=n.on_arm(SetBool.Request(data=True),SetBool.Response())
    assert not response.success


def test_stale_feedback_and_invalid_input_stop():
    n=guard(True);n.joint_at=99.
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):n.tick()
    assert not n.armed
    for bad in (command(float('nan')),command(.1)):
        if bad.linear.x==.1:bad.linear.y=.1
        n=guard(True)
        with patch('nino_real_rl.teleop.monotonic',return_value=100.):n.on_command(bad)
        assert not n.armed


def test_raw_joint_duplicate_is_not_freshened():
    n=guard(True)
    message=JointState(name=['left_wheel_joint','right_wheel_joint'],velocity=[0.,0.])
    message.header.stamp=Time(seconds=10.).to_msg()
    with patch('nino_real_rl.teleop.monotonic',return_value=100.):n.on_joint(message)
    assert not n.armed and n.joint_at is None and n.joint_stamp==10.


def test_operator_stop_also_disables_when_already_disarmed():
    n=guard();response=n.on_arm(SetBool.Request(data=False),SetBool.Response())
    assert response.success and n.disable.publish.call_args.args[0].data is False


def test_teleop_hardware_launch_has_no_nav2_ekf_or_slam(monkeypatch):
    from launch import LaunchContext
    from launch_ros.actions import Node
    root=Path(__file__).resolve().parents[3]
    file=root/'src/pi5_slam_bringup/launch/robot.launch.py'
    spec=importlib.util.spec_from_file_location('test_teleop_hardware_launch',file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'get_package_share_directory',lambda _:str(root/'src/pi5_slam_bringup'))
    ctx=LaunchContext();ctx.launch_configurations.update(dict(mode='teleop',robot_config=str(root/'src/pi5_slam_bringup/config/robot.yaml'),
        start_agent='false',start_lidar='false',stm32_port='',lidar_port='',allow_motor_enable='true',use_sim_time='false',slam_params_file='',rviz='false'))
    actions=module.setup(ctx)
    packages=[a.node_package for a in actions if isinstance(a,Node)]
    assert packages==['robot_state_publisher','stm32_base_bridge']
