"""Service enable chỉ dùng publisher giả; không init ROS hay chạm phần cứng."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
from stm32_base_bridge.bridge_node import Stm32BaseBridge


def bridge(allow=True,subscribers=1):
    node=object.__new__(Stm32BaseBridge)
    node.get_parameter=lambda key:SimpleNamespace(value=allow)
    node.count_subscribers=lambda topic:subscribers
    node.motor_enable_publisher=Mock()
    node.get_logger=Mock()
    return node


def test_enable_needs_launch_permission():
    n=bridge(allow=False)
    response=n._set_motor_enable(SimpleNamespace(data=True),SimpleNamespace())
    assert not response.success
    n.motor_enable_publisher.publish.assert_not_called()
    n.get_logger.return_value.warning.assert_called_once()


def test_enable_needs_firmware_subscriber():
    n=bridge(subscribers=0)
    response=n._set_motor_enable(SimpleNamespace(data=True),SimpleNamespace())
    assert not response.success
    assert 'subscriber not discovered' in response.message
    n.motor_enable_publisher.publish.assert_not_called()


def test_enable_publishes_bool_without_pretending_can_ack():
    n=bridge()
    response=n._set_motor_enable(SimpleNamespace(data=True),SimpleNamespace())
    assert response.success and response.message=='Motor enable command sent'
    assert n.motor_enable_publisher.publish.call_args.args[0].data is True


def test_disable_is_allowed_even_without_permission_or_subscriber():
    n=bridge(allow=False,subscribers=0)
    response=n._set_motor_enable(SimpleNamespace(data=False),SimpleNamespace())
    assert response.success
    assert n.motor_enable_publisher.publish.call_args.args[0].data is False
