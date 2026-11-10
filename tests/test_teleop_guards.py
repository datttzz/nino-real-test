"""Kiểm tra guard bằng callback giả; không init ROS hoặc phát lệnh tới xe."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/nino_real_rl'))
from nino_real_rl.teleop import TeleopGuard


def guard(now=10.3):
    n = object.__new__(TeleopGuard)
    n.cfg = dict(sensor_timeout=.25, motor_status_timeout=.6, command_timeout=.3,
                 max_linear_speed=.5,max_angular_speed=.4,input_topic='/teleop/cmd_vel')
    n.armed = True
    n.has_moved = False
    n.fault = ''
    n.arm_pending = False
    n.arm_requested_at = None
    n.last_output = (0.,0.)
    n.status = 0x13
    n.status_at = 10.
    n.joint_at = now
    n.joint_stamp = 100.
    n.ros_now = lambda: 100.
    n.command = (.1, 0.)
    n.command_at = now
    n.publish_command = Mock()
    n.disable = Mock()
    n.get_logger = Mock()
    n.graph_error = Mock(return_value='')
    return n


def test_status_gap_between_encoder_and_status_limits_does_not_trip():
    n = guard()
    with patch('nino_real_rl.teleop.monotonic', return_value=10.3):
        n.tick()
    assert n.armed
    n.publish_command.assert_called_once_with(.1, 0.)
    n.disable.publish.assert_not_called()


@pytest.mark.parametrize('status_at', [None, 9.69, 10.31])
def test_missing_stale_or_future_status_zeroes_and_disables(status_at):
    n = guard()
    n.status_at = status_at
    with patch('nino_real_rl.teleop.monotonic', return_value=10.3):
        n.tick()
    assert not n.armed
    assert n.fault == 'Missing/stale motor status'
    n.publish_command.assert_called_once_with(0., 0.)
    assert n.disable.publish.call_args.args[0].data is False
    # Trạng thái phục hồi không tự arm lại.
    with patch('nino_real_rl.teleop.monotonic', return_value=10.4):
        n.on_status(SimpleNamespace(data=0x13))
    assert not n.armed


def test_encoder_still_has_short_timeout():
    n = guard()
    n.status_at = 10.3
    n.joint_at = 10.049
    with patch('nino_real_rl.teleop.monotonic', return_value=10.3):
        n.tick()
    assert not n.armed
    assert n.fault == 'Missing/stale wheel feedback'


def test_keyboard_watchdog_still_disarms():
    n = guard()
    n.command_at = 9.99
    with patch('nino_real_rl.teleop.monotonic', return_value=10.3):
        n.tick()
    assert not n.armed
    assert n.fault == 'Keyboard input timeout'


def test_mcu_command_timeout_still_disarms_after_motion():
    n = guard()
    n.has_moved = True
    n.status = 0x33
    with patch('nino_real_rl.teleop.monotonic', return_value=10.3):
        n.tick()
    assert not n.armed
    assert n.fault == 'Firmware command timeout'


def test_arm_zero_handshake_prevents_cached_watchdog_from_killing_first_motion():
    n=guard(10.1);n.armed=False;n.command=(0.,0.);n.status=0x33
    with patch('nino_real_rl.teleop.monotonic',return_value=10.1):
        response=n.on_arm(SimpleNamespace(data=True),SimpleNamespace())
    assert response.success and n.arm_pending
    n.publish_command.assert_called_once_with(0.,0.)
    n.disable.publish.assert_not_called()  # Arm không tự bật motor.
    request=Twist();request.linear.x=.5
    with patch('nino_real_rl.teleop.monotonic',return_value=10.12):
        n.on_command(request);n.tick()
    assert n.armed and not n.has_moved
    assert n.publish_command.call_args.args==(0.,0.)
    with patch('nino_real_rl.teleop.monotonic',return_value=10.2):
        n.on_status(SimpleNamespace(data=0x13))
        n.tick()
    assert n.armed and not n.arm_pending and n.has_moved
    n.publish_command.assert_called_with(.5,0.)


def test_cached_healthy_status_is_not_a_new_startup_ack():
    n=guard(10.1);n.armed=False;n.command=(0.,0.)
    with patch('nino_real_rl.teleop.monotonic',return_value=10.1):
        assert n.on_arm(SimpleNamespace(data=True),SimpleNamespace()).success
        n.tick()
    assert n.arm_pending and not n.has_moved
    n.publish_command.assert_called_with(0.,0.)


def test_startup_timeout_still_stops_even_if_motor_status_keeps_arriving():
    n=guard(10.75);n.arm_pending=True;n.arm_requested_at=10.1
    n.status=0x33;n.status_at=10.75
    with patch('nino_real_rl.teleop.monotonic',return_value=10.75):
        n.on_status(SimpleNamespace(data=0x33))
    assert not n.armed and not n.arm_pending
    assert 'startup not ready' in n.fault
    n.publish_command.assert_called_once_with(0.,0.)
    assert n.disable.publish.call_args.args[0].data is False


def test_late_startup_ack_cannot_rearm_after_deadline():
    n=guard(10.75);n.arm_pending=True;n.arm_requested_at=10.1
    with patch('nino_real_rl.teleop.monotonic',return_value=10.75):
        n.on_status(SimpleNamespace(data=0x13))
    assert not n.armed
    assert 'startup not ready' in n.fault


def test_nonzero_input_while_disarmed_is_logged_and_not_forwarded():
    n=guard();n.armed=False
    request=Twist();request.linear.x=.5
    with patch('nino_real_rl.teleop.monotonic',return_value=10.3):n.on_command(request)
    n.publish_command.assert_not_called()
    assert 'DISARMED' in n.get_logger.return_value.warning.call_args.args[0]


def test_arm_rejects_disabled_motor_instead_of_auto_enabling():
    n=guard();n.armed=False;n.status=3;n.command=(0.,0.)
    with patch('nino_real_rl.teleop.monotonic',return_value=10.3):
        response=n.on_arm(SimpleNamespace(data=True),SimpleNamespace())
    assert not response.success
    assert 'Motor disabled' in response.message
    n.disable.publish.assert_not_called()
    n.publish_command.assert_not_called()


def test_topic_echo_cannot_masquerade_as_firmware_cmd_subscriber():
    n=guard()
    n.count_publishers=lambda topic: 1 if topic in ('/cmd_vel','/teleop/cmd_vel') else 0
    n.count_subscribers=lambda topic:1
    endpoint=lambda name,kind:SimpleNamespace(node_name=name,node_namespace='/',topic_type=kind)
    n.get_publishers_info_by_topic=lambda topic:[endpoint('stm32h7_node','std_msgs/msg/UInt8')]
    n.get_subscriptions_info_by_topic=lambda topic:[endpoint('_ros2cli_echo','geometry_msgs/msg/Twist')]
    assert 'must match' in TeleopGuard.graph_error(n)
    n.get_subscriptions_info_by_topic=lambda topic:[endpoint('stm32h7_node','geometry_msgs/msg/Twist')]
    assert TeleopGuard.graph_error(n)==''
