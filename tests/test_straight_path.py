"""Kiểm tra offline; không init ROS, mở serial, gọi action server hay chạy motor."""
from collections import deque
import importlib.util
from math import pi
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import sys

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/nino_real_rl'))
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from nino_real_rl.straight_path import StraightPath, line_points
from nino_real_rl.features import path_features


def test_straight_geometry_matches_odom_endpoints_and_signed_error():
    points=line_points((1.,2.),(1.,4.))
    assert len(points)==41
    assert points[0]==pytest.approx((1.,2.,pi/2))
    assert points[-1]==pytest.approx((1.,4.,pi/2))
    error,heading,progress,remaining=path_features(1.1,3.,pi/2,[(x,y) for x,y,_ in points])
    assert (error,heading,progress,remaining)==pytest.approx((-.1,0.,1.,1.))


@pytest.mark.parametrize('end',[(0.,0.),(.01,0.),(3.01,0.),(float('nan'),0.)])
def test_invalid_or_too_distant_goal_rejected(end):
    with pytest.raises(ValueError):line_points((0.,0.),end)


def node():
    n=object.__new__(StraightPath)
    n.max_distance=3.;n.active=False;n.goal_handle=None;n.cancel_pending=False;n.reference=None
    n.get_clock=Mock(return_value=NS(now=lambda:Time(seconds=100)))
    n.get_logger=Mock()
    tf=TransformStamped();tf.header.stamp=Time(seconds=99.9).to_msg()
    tf.transform.translation.x=.5;tf.transform.translation.y=1.
    n.tf=Mock();n.tf.lookup_transform.return_value=tf
    n.client=Mock();n.client.server_is_ready.return_value=True
    n.trace=deque(maxlen=3000);n.last_trace_stamp=None
    return n


def goal():
    message=PoseStamped();message.header.frame_id='odom'
    message.pose.position.x=1.5;message.pose.position.y=1.
    return message


def test_odom_goal_sends_one_follow_path_and_never_retargets_active_goal():
    n=node();n.on_goal(goal())
    request=n.client.send_goal_async.call_args.args[0]
    assert request.path.header.frame_id=='odom'
    assert request.path.poses[0].pose.position.x==.5
    assert request.path.poses[-1].pose.position.x==1.5
    assert request.controller_id=='FollowPath'
    assert n.active
    n.on_goal(goal())
    n.client.send_goal_async.assert_called_once()


@pytest.mark.parametrize('case',['wrong_frame','stale_tf','missing_controller'])
def test_invalid_goal_never_requests_motion(case):
    n=node();message=goal()
    if case=='wrong_frame':message.header.frame_id='map'
    elif case=='stale_tf':n.tf.lookup_transform.return_value.header.stamp=Time(seconds=99).to_msg()
    else:n.client.server_is_ready.return_value=False
    n.on_goal(message)
    n.client.send_goal_async.assert_not_called()
    assert not n.active and n.reference is None


def test_stop_before_goal_ack_cancels_when_handle_arrives():
    n=node();n.on_goal(goal())
    response=n.on_stop(NS(),NS())
    assert response.success and n.cancel_pending
    handle=Mock(accepted=True)
    n.on_accepted(NS(result=lambda:handle))
    handle.cancel_goal_async.assert_called_once()
    n.on_result(NS(result=lambda:NS(status=5,result=NS(error_code=0,error_msg='canceled'))))
    assert not n.active and n.goal_handle is None


def test_odom_trace_keeps_source_stamp_and_drops_duplicates_wrong_frames():
    n=node();message=Odometry()
    message.header.frame_id='odom';message.child_frame_id='base_footprint'
    message.header.stamp=Time(seconds=99.9).to_msg();message.pose.pose.position.x=1.
    n.on_odom(message);n.on_odom(message)
    assert len(n.trace)==1
    assert n.trace[0].header.stamp==message.header.stamp
    message.header.frame_id='map';n.on_odom(message)
    assert len(n.trace)==1
    assert n.trace[0].header.frame_id=='odom'


def test_mapless_launch_uses_collision_chain_and_one_controller(monkeypatch):
    from launch import LaunchContext
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('straight_launch',root/'src/nino_real_rl/launch/straight_line.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'get_package_share_directory',lambda name:str(root/'src'/name))
    for name in ('Node','IncludeLaunchDescription','TimerAction'):
        monkeypatch.setattr(module,name,lambda *args,**kwargs:NS(**kwargs))
    context=LaunchContext();context.launch_configurations.update(dict(controller='shadow',speed='0.10',
        max_distance='3.0',checkpoint='',hardware_file='',robot_config=str(root/'src/pi5_slam_bringup/config/robot.yaml'),
        stm32_port='',lidar_port='',log_dir='runs',rviz='false'))
    actions=module.setup(context)
    includes=[a for a in actions if hasattr(a,'launch_arguments')]
    included=dict(includes[0].launch_arguments)
    assert included['mode']=='path_test' and included['allow_motor_enable']=='false'
    nodes=[a for a in actions if hasattr(a,'package')]
    executables=[a.executable for a in nodes]
    assert executables.count('real_controller')==1
    assert {'controller_server','velocity_smoother','collision_monitor','straight_path'}<=set(executables)
    assert not {'map_server','amcl','planner_server'}&set(executables)
    controller=next(a for a in nodes if a.executable=='real_controller')
    assert controller.parameters[-1]['map_frame']=='odom'
    assert controller.parameters[-1]['path_topic']=='/straight/reference_path'
    monitor=next(a for a in nodes if a.executable=='collision_monitor')
    assert monitor.parameters[-1]['cmd_vel_out_topic']=='/rl/cmd_vel_reference'
    context.launch_configurations.update(controller='pi_sac')
    with pytest.raises(ValueError,match='hardware_file'):module.setup(context)
