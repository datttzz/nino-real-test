"""IMU callback with ROS message classes, no ROS init, DDS or hardware publishers."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
from rclpy.time import Time
from sensor_msgs.msg import Imu
from stm32_base_bridge.bridge_node import Stm32BaseBridge


def bridge():
    node=object.__new__(Stm32BaseBridge)
    node.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.))
    node.get_logger=Mock()
    node.last_stamp={};node.stamp_tolerance=.05;node.imu_frame_id='imu_link'
    node.imu_publisher=Mock()
    return node


def sample(stamp_ns,frame='imu_link'):
    msg=Imu();msg.header.stamp=Time(nanoseconds=stamp_ns).to_msg()
    msg.header.frame_id=frame;msg.orientation.w=1.;msg.linear_acceleration.z=9.80665
    return msg


def test_old_imu_dropped_and_recovery_preserves_source_stamp():
    node=bridge();old=sample(1_000_000_000)
    node._imu_callback(old)
    node.imu_publisher.publish.assert_not_called()
    assert old.header.stamp.sec==1 and 'imu' not in node.last_stamp
    fresh=sample(9_980_000_000)
    node._imu_callback(fresh)
    output=node.imu_publisher.publish.call_args.args[0]
    assert output.header.stamp==fresh.header.stamp
    assert node.last_stamp['imu']==9_980_000_000


def test_duplicate_backwards_and_future_never_create_fresh_imu():
    node=bridge();node._imu_callback(sample(9_990_000_000))
    for stamp in (9_990_000_000,9_980_000_000,10_010_000_000):
        node._imu_callback(sample(stamp))
    node.imu_publisher.publish.assert_called_once()
    assert node.last_stamp['imu']==9_990_000_000


def test_filling_missing_frame_does_not_modify_raw_message_or_stamp():
    node=bridge();raw=sample(9_980_000_000,frame='')
    node._imu_callback(raw)
    output=node.imu_publisher.publish.call_args.args[0]
    assert raw.header.frame_id=='' and output.header.frame_id=='imu_link'
    assert output.header.stamp==raw.header.stamp
