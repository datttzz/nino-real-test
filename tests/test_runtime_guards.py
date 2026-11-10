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
    sys.modules['rclpy.qos'].QoSProfile = Message
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
        n.armed=armed; n.mode=mode; n.pi=WheelPI(.06,.345);n.safety_fault=''
        n.action=np.array([.8,-.8]);n.action_at=5.;n.moving_since=1
        n.output=Mock() if mode!='shadow' else None
        n.disable=Mock() if mode!='shadow' else None
        n.get_logger=Mock(); n.get_logger.return_value=Mock()
        n.contract=Contract(); n.cfg=dict(command_timeout=.25,motor_status_timeout=.35,
                                        imu_topic='/imu/data',imu_input_max_age=.05)
        n.status_at=1.;n.command_at=5.;n.last_actor=5.;n.last_diag=5.
        n.status=0x13
        n.samples={};n.sample_rejections={}
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

    def test_firmware_timeout_zeroes_disables_and_requires_manual_rearm(self):
        for mode in ('pi', 'pi_sac'):
            with self.subTest(mode=mode):
                n=self.make(mode=mode);n.pi.integral[:]=1.;n.moving_since=0.
                # Feedback OK + motor enabled + MCU command timeout.
                n.on_status(types.SimpleNamespace(data=0x33))
                self.assertFalse(n.armed)
                self.assertIn('Firmware command timeout',n.fault)
                self.assertEqual(n.output.publish.call_args.args[0].data,[0.,0.])
                self.assertFalse(n.disable.publish.call_args.args[0].data)
                np.testing.assert_array_equal(n.pi.integral,[0.,0.])
                np.testing.assert_array_equal(n.action,[0.,0.])
                # Status healthy again must not rearm or restart actuation.
                n.on_status(types.SimpleNamespace(data=0x13))
                self.assertFalse(n.armed)
                n.output.publish.assert_called_once()
                n.disable.publish.assert_called_once()

    def test_firmware_timeout_before_first_goal_keeps_zero_standby(self):
        n=self.make();n.command=np.zeros(2);n.moving_since=None
        n.on_status(types.SimpleNamespace(data=0x33))
        self.assertTrue(n.armed)
        n.basic_ready=Mock();n.graph_error=Mock(return_value='')
        with patch('nino_real_rl.node.monotonic',return_value=5.2):n.tick()
        self.assertTrue(n.armed)
        self.assertEqual(n.output.publish.call_args.args[0].data,[0.,0.])
        n.disable.publish.assert_not_called()

    def test_firmware_timeout_does_not_make_shadow_actuate(self):
        n=self.make(False,'shadow')
        n.on_status(types.SimpleNamespace(data=0x33))
        self.assertFalse(n.armed)
        self.assertIsNone(n.output);self.assertIsNone(n.disable)

    def test_can_fault_and_motor_disable_still_disarm(self):
        for status in (0x17,0x03):
            with self.subTest(status=status):
                n=self.make();n.moving_since=None
                n.on_status(types.SimpleNamespace(data=status))
                self.assertFalse(n.armed)
                n.disable.publish.assert_called_once()

    def test_pi_observation_does_not_hide_latched_disarm_reason(self):
        import json
        n=self.make(True,'pi')
        n.basic_ready=Mock(side_effect=ValueError('Stale motor status: age=0.260s > 0.250s; bits=0x53'))
        n.policy=None
        with patch('nino_real_rl.node.monotonic',return_value=5.6):n.tick()
        status=json.loads(n.diag.publish.call_args.args[0].data)
        self.assertFalse(status['armed'])
        self.assertTrue(status['fault'].startswith('Stale motor status:'))
        n.basic_ready=Mock()
        n.observe=Mock(side_effect=ValueError('Missing/stale reference path'))
        with patch('nino_real_rl.node.monotonic',return_value=6.2):n.tick()
        status=json.loads(n.diag.publish.call_args.args[0].data)
        self.assertTrue(status['fault'].startswith('Stale motor status:'))
        self.assertIn('reference path',status['observation_note'])
        self.assertFalse(n.armed)
        n.disable.publish.assert_called_once()

    def test_status_guard_reasons_are_distinct_and_limit_unchanged(self):
        for age,bits,reason in [(0.36,0x53,'Stale motor status'),
                                (0.1,0x51,'CAN feedback not fresh'),
                                (0.1,0x57,'CAN bus-off')]:
            with self.subTest(reason=reason):
                n=self.make();n.cfg['sensor_timeout']=.25
                n.status_at=5.-age;n.status=bits
                with self.assertRaisesRegex(ValueError,reason):n.basic_ready(5.)

    def test_successful_manual_arm_clears_latched_reason(self):
        n=self.make(False,'pi');n.safety_fault='previous stale status'
        n.hardware={};n.basic_ready=Mock();n.graph_error=Mock(return_value='');n.imu_in_base=Mock()
        response=types.SimpleNamespace(success=None,message='')
        with patch('nino_real_rl.node.hardware_contract'):
            n.on_arm(types.SimpleNamespace(data=True),response)
        self.assertTrue(response.success);self.assertTrue(n.armed)
        self.assertEqual(n.safety_fault,'')

    def fresh_wheels(self,n,now):
        n.cfg.update(sensor_timeout=.25,joint_min_rate_hz=30.)
        n.joint_at=now-.01;n.joint_stamp=now-.01
        n.ros_now=Mock(return_value=now)
        sec=int(now-.01);nano=round((now-.01-sec)*1e9)
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec,nanosec=nano)))
        n.samples={key:(msg,now-.01) for key in ('imu','odom')}

    def test_279ms_status_is_allowed_but_351ms_still_stops_both_modes(self):
        for mode in ('pi','pi_sac'):
            with self.subTest(mode=mode):
                n=self.make(True,mode);self.fresh_wheels(n,5.2)
                n.status_at=5.2-.279;n.command[:]=0.;n.moving_since=None;n.policy=None
                n.graph_error=Mock(return_value='')
                n.observe=Mock(side_effect=ValueError('Missing/stale reference path'))
                with patch('nino_real_rl.node.monotonic',return_value=5.2):n.tick()
                self.assertTrue(n.armed);n.disable.publish.assert_not_called()
                n.status_at=5.6-.351;self.fresh_wheels(n,5.6)
                with patch('nino_real_rl.node.monotonic',return_value=5.6):n.tick()
                self.assertFalse(n.armed)
                self.assertIn('0.350s',n.safety_fault)
                self.assertFalse(n.disable.publish.call_args.args[0].data)

    def test_separate_status_deadline_never_extends_wheel_freshness(self):
        n=self.make();self.fresh_wheels(n,5.)
        n.status_at=5.-.279;n.joint_at=5.-.251
        with self.assertRaisesRegex(ValueError,'Missing/stale wheel feedback'):
            n.basic_ready(5.,enabled=True)
        n.joint_at=4.99;n.joint_stamp=4.749
        with self.assertRaisesRegex(ValueError,'Stale/future source timestamp'):
            n.basic_ready(5.,enabled=True)

    def test_status_can_fault_is_not_ignored_within_350ms_window(self):
        n=self.make();self.fresh_wheels(n,5.);n.status_at=5.-.279
        for bits,reason in [(0x51,'CAN feedback not fresh'),(0x57,'CAN bus-off')]:
            n.status=bits
            with self.assertRaisesRegex(ValueError,reason):n.basic_ready(5.,enabled=True)

    def sensor_message(self,stamp):
        sec=int(stamp)
        return types.SimpleNamespace(header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec,nanosec=round((stamp-sec)*1e9))))

    def test_late_or_reordered_packet_does_not_erase_fresh_sample(self):
        for key in ('imu','odom'):
            for badstamp in (9.,9.95,11.):
                with self.subTest(key=key,stamp=badstamp):
                    n=self.make();n.cfg['sensor_timeout']=.25
                    cached=self.sensor_message(9.95)
                    n.samples[key]=(cached,9.96);n.ros_now=Mock(return_value=10.)
                    with patch('nino_real_rl.node.monotonic',return_value=10.):
                        n.on_sample(key,self.sensor_message(badstamp))
                    self.assertTrue(n.armed)
                    self.assertIs(n.samples[key][0],cached)
                    self.assertEqual(n.samples[key][1],9.96)
                    self.assertEqual(n.sample_rejections[key],1)
                    n.disable.publish.assert_not_called()

    def test_invalid_packet_without_fresh_cache_disarms(self):
        for key in ('imu','odom'):
            n=self.make();n.cfg['sensor_timeout']=.25
            n.samples[key]=(self.sensor_message(9.),9.)
            n.ros_now=Mock(return_value=10.)
            with patch('nino_real_rl.node.monotonic',return_value=10.):
                n.on_sample(key,self.sensor_message(9.5))
            self.assertFalse(n.armed)
            self.assertNotIn(key,n.samples)
            self.assertIn('source_age_s=0.500',n.safety_fault)

    def test_cached_source_expiration_stops_pi_and_sac_without_new_callback(self):
        for mode in ('pi','pi_sac'):
            for key in ('imu','odom'):
                n=self.make(True,mode);self.fresh_wheels(n,5.6)
                n.status_at=5.59;n.graph_error=Mock(return_value='');n.policy=None
                n.observe=Mock(side_effect=ValueError('Missing/stale reference path'))
                n.samples[key]=(self.sensor_message(5.0),5.59)
                with patch('nino_real_rl.node.monotonic',return_value=5.6):n.tick()
                self.assertFalse(n.armed)
                n.disable.publish.assert_called_once()

    def test_new_valid_source_replaces_old_cache_with_original_stamp(self):
        n=self.make();n.cfg['sensor_timeout']=.25
        n.samples['odom']=(self.sensor_message(9.9),9.91)
        n.ros_now=Mock(return_value=10.)
        msg=self.sensor_message(9.99)
        with patch('nino_real_rl.node.monotonic',return_value=10.):n.on_sample('odom',msg)
        self.assertIs(n.samples['odom'][0],msg)
        self.assertEqual(n.samples['odom'][1],10.)
        self.assertEqual(n.sample_rejections,{})

    def test_direct_imu_cannot_accept_stale_or_future_raw_input(self):
        for stamp in (9.9,10.001):
            n=self.make();n.cfg['sensor_timeout']=.25
            n.ros_now=Mock(return_value=10.)
            with patch('nino_real_rl.node.monotonic',return_value=10.):
                n.on_sample('imu',self.sensor_message(stamp))
            self.assertFalse(n.armed)
            self.assertNotIn('imu',n.samples)
            self.assertIn('50ms',n.safety_fault)

    def test_direct_imu_keeps_original_message_and_bounded_cache(self):
        n=self.make();n.cfg['sensor_timeout']=.25;n.ros_now=Mock(return_value=10.)
        msg=self.sensor_message(9.99)
        with patch('nino_real_rl.node.monotonic',return_value=10.):n.on_sample('imu',msg)
        self.assertIs(n.samples['imu'][0],msg)
        n.sample_ready('imu',10.2,10.2)
        with self.assertRaisesRegex(ValueError,'imu: Stale/future'):
            n.sample_ready('imu',10.25,10.25)


if __name__=='__main__': unittest.main()
