"""Callback state-machine tests using message/API stand-ins, without ROS/DDS.

Tests controller decisions, not ROS middleware/launch/firmware behavior.
"""
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/nino_real_rl'))


def load_node_with_standins():
    saved = sys.modules.copy()
    names = ['rclpy','rclpy.node','rclpy.qos','rclpy.time','geometry_msgs','geometry_msgs.msg',
             'nav_msgs','nav_msgs.msg','sensor_msgs','sensor_msgs.msg','std_msgs','std_msgs.msg',
             'std_srvs','std_srvs.srv','tf2_ros']
    for name in names: sys.modules[name] = types.ModuleType(name)
    class Message:
        def __init__(self,**kw): self.__dict__.update(kw)
    sys.modules['rclpy'].node = sys.modules['rclpy.node']
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.qos'].qos_profile_sensor_data = None
    sys.modules['rclpy.time'].Time = Message
    for module,classes in [('geometry_msgs.msg',['Twist']),('nav_msgs.msg',['Odometry','Path']),
                           ('sensor_msgs.msg',['Imu','JointState']),
                           ('std_msgs.msg',['Bool','UInt8','String','Float64MultiArray']),
                           ('std_srvs.srv',['SetBool']),('tf2_ros',['Buffer','TransformListener'])]:
        for cls in classes: setattr(sys.modules[module],cls,Message)
    from nino_real_rl.node import RealController
    # Keep imported class bindings but restore the interpreter's original modules.
    for name in names:
        if name in saved: sys.modules[name] = saved[name]
        else: sys.modules.pop(name,None)
    return RealController


RealController = load_node_with_standins()


class RuntimeTests(unittest.TestCase):
    def make(self,armed=True,mode='pi_sac'):
        from nino_real_rl.core import WheelPI,Contract
        n = RealController.__new__(RealController)
        n.armed=armed; n.mode=mode; n.pi=WheelPI(.06,.345)
        n.action=np.array([.8,-.8]);n.action_at=5.;n.moving_since=1
        n.output=Mock() if mode!='shadow' else None
        n.disable=Mock() if mode!='shadow' else None
        n.get_logger=Mock(); n.get_logger.return_value=Mock()
        n.contract=Contract(); n.cfg=dict(command_timeout=.25)
        n.status_at=1.;n.command_at=5.;n.last_actor=5.;n.last_diag=5.
        n.command=np.array([.1,0.])
        n.diag=Mock();n.joint_intervals=[.02]*20
        return n

    def test_fault_zero_then_disable_latches_disarmed(self):
        n=self.make()
        n.trip('stale encoder')
        self.assertFalse(n.armed)
        self.assertEqual(n.output.publish.call_args.args[0].data,[0.,0.])
        self.assertFalse(n.disable.publish.call_args.args[0].data)
        np.testing.assert_array_equal(n.action,[0,0])

    def test_shadow_fault_never_writes_firmware(self):
        n=self.make(False,'shadow')
        n.trip('missing checkpoint')
        self.assertIsNone(n.output);self.assertIsNone(n.disable)

    def test_ownership_detects_velocity_and_mit_conflicts(self):
        n=self.make(); n.count_publishers=lambda name: {'/cmd_torque':1}.get(name,0)
        n.count_subscribers=lambda name: 1
        self.assertEqual(n.graph_error(),'')
        for topic in ('/cmd_vel','/cmd_mit'):
            n.count_publishers=lambda name,topic=topic: 1 if name in (topic,'/cmd_torque') else 0
            self.assertTrue(n.graph_error())
        n.count_publishers=lambda name: 2 if name=='/cmd_torque' else 0
        self.assertTrue(n.graph_error())

    def test_missing_firmware_torque_subscriber_blocks_arm(self):
        n=self.make()
        n.count_publishers=lambda name: 1 if name=='/cmd_torque' else 0
        n.count_subscribers=lambda name: 0
        self.assertIn('not discovered',n.graph_error())

    def test_pi_watchdog_disarms_even_without_more_joint_callbacks(self):
        n=self.make(True,'pi')
        n.basic_ready=Mock(side_effect=ValueError('wheel timeout'))
        with patch('nino_real_rl.node.monotonic',return_value=5.01): n.tick()
        self.assertFalse(n.armed)
        n.disable.publish.assert_called_once()

    def test_shadow_cannot_arm(self):
        n=self.make(False,'shadow')
        response=types.SimpleNamespace(success=None,message='')
        n.on_arm(types.SimpleNamespace(data=True),response)
        self.assertFalse(response.success);self.assertFalse(n.armed)

    def test_imu_failure_during_pi_sac_disarms(self):
        n=self.make()
        n.basic_ready=Mock();n.graph_error=Mock(return_value='')
        n.observe=Mock(side_effect=ValueError('IMU stale'))
        with patch('nino_real_rl.node.monotonic',return_value=5.2): n.tick()
        self.assertFalse(n.armed)

    def test_armed_standby_before_first_goal_stays_zero(self):
        n=self.make();n.command=np.zeros(2);n.moving_since=None
        n.basic_ready=Mock();n.graph_error=Mock(return_value='')
        n.observe=Mock(side_effect=ValueError('No path before first goal'))
        with patch('nino_real_rl.node.monotonic',return_value=5.2): n.tick()
        self.assertTrue(n.armed)
        n.observe.assert_not_called()
        self.assertEqual(n.output.publish.call_args.args[0].data,[0.,0.])

    def test_command_timeout_after_motion_disarms(self):
        n=self.make();n.command_at=4.;n.moving_since=4.
        n.basic_ready=Mock();n.graph_error=Mock(return_value='')
        with patch('nino_real_rl.node.monotonic',return_value=5.01): n.tick()
        self.assertFalse(n.armed)


if __name__=='__main__': unittest.main()
