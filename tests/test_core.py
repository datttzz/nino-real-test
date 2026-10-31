"""Offline contract/safety checks. These are NOT physical motor validation."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/nino_real_rl'))
from nino_real_rl.core import Contract, WheelPI, observation, source_stamp, hardware_contract
from nino_real_rl.features import imu_features, path_features


class CoreTests(unittest.TestCase):
    def test_original_23_element_observation_with_known_values(self):
        # Independent explicit expected order; avoids a same-code round trip.
        obs, metrics = observation(Contract(), (2,.1,math.pi/6), [(0,0),(10,0)],
            [1,2,12.80665], [0,0,0,1], [3,6,9], .25,.2,[.1,.3],
            [6,12],[1,-2],[.4,-.5],[.15,-.3])
        np.testing.assert_allclose(obs, [.5,1/6,.8,0,0,.1,.2,.3,1,2,3,
                                      .5,.2,.2,.3,.5,1,.5,-1,.4,-.5,.5,-1], atol=1e-6)
        self.assertEqual(obs.dtype, np.float32)
        self.assertAlmostEqual(metrics['cross_track'], .1)

    def test_tilted_imu_gravity_removal(self):
        angle = .3
        q = [math.sin(angle/2),0,0,math.cos(angle/2)]
        acceleration = [0,9.80665*math.sin(angle),9.80665*math.cos(angle)]
        a, roll, pitch = imu_features(acceleration,q)
        np.testing.assert_allclose(a,0,atol=1e-12)
        self.assertAlmostEqual(roll,angle); self.assertAlmostEqual(pitch,0)

    def test_heading_wrap_and_duplicate_path_points(self):
        e,h,p,r = path_features(-1,0,-math.pi+.01, [(0,0),(0,0),(-2,0)])
        self.assertAlmostEqual(h,.01); self.assertAlmostEqual(r,1)

    def test_invalid_sensor_values_rejected(self):
        with self.assertRaises(ValueError): imu_features([0,0,0],[0,0,0,0])
        with self.assertRaises(ValueError): path_features(0,0,0,[(0,0),(0,0)])
        with self.assertRaises(ValueError):
            observation(Contract(), (0,0,0),[(0,0),(1,0)], [float('nan'),0,9.8],
                        [0,0,0,1],[0,0,0],0,0,[0,0],[0,0],[0,0],[0,0],[0,0])

    def test_pi_matches_original_first_control_update(self):
        pi = WheelPI(.06,.345)
        out = pi.update([.12,0],[0,0],[0,0],.02)
        # target = accel*dt = .24 rad/s; I=.0048, torque=.07248 N.m.
        np.testing.assert_allclose(out,[.07248,.07248],rtol=1e-12)
        self.assertEqual((pi.KP,pi.KI),(.3,.1))

    def test_slew_and_total_clamp_with_residual(self):
        pi = WheelPI(.06,.345)
        old = np.zeros(2)
        for i in range(100):
            total = pi.update([.6,0],[-10,-10],[.3,.3],.02)
            self.assertLessEqual(np.max(np.abs(total)),2)
            self.assertLessEqual(np.max(np.abs(total-old)),.2000000001)
            self.assertLessEqual(np.max(np.abs(pi.accepted)),.3000000001)
            old = total
        self.assertEqual(float(pi.integral.max()),4)

    def test_stop_clears_integral_residual_and_torque_immediately(self):
        pi = WheelPI(.06,.345)
        for i in range(30): pi.update([.2,0],[0,0],[.3,.3],.02)
        out = pi.update([0,0],[2,2],[.3,.3],.02)
        for v in (out,pi.integral,pi.base,pi.accepted,pi.target):
            np.testing.assert_array_equal(v,[0,0])

    def test_overspeed_forbids_torque_accelerating_wheel(self):
        pi = WheelPI(.06,.345)
        pi.total[:] = .5
        out = pi.update([.3,0],[13,13],[.3,.3],.02)
        self.assertTrue(np.all(out <= 0))

    def test_bad_dt_and_action_clear_state(self):
        pi = WheelPI(.06,.345)
        with self.assertRaises(ValueError): pi.update([1,0],[0,0],[0,0],0)
        with self.assertRaises(ValueError): pi.update([1,0],[0,0],[.4,0],.02)
        np.testing.assert_array_equal(pi.total,[0,0])

    def test_timestamp_duplicate_stale_future_rejected(self):
        self.assertEqual(source_stamp(10,10.01,9.98),10)
        for stamp,now,old in ((10,10.01,10), (9.9,10,10), (9,10,None),(11,10,None),(0,10,None)):
            with self.assertRaises(ValueError): source_stamp(stamp,now,old)

    def test_hardware_example_fails_closed(self):
        import json
        data = json.loads((Path(__file__).resolve().parents[1]/'hardware.example.json').read_text())
        with self.assertRaises(ValueError): hardware_contract(data)

    def test_hardware_live_requires_all_assertions_and_units(self):
        data = dict(firmware_torque_verified=True,wheel_torque_nm_verified=True,
                    watchdog_verified=True,feedback_signs_verified=True,
                    accept_feedback_rate_difference=True,command_topic='/cmd_torque',
                    left_command_sign=1,right_command_sign=-1,
                    verified_wheel_torque_limit_nm=2,verified_watchdog_seconds=.5)
        hardware_contract(data)
        for key in ('firmware_torque_verified','wheel_torque_nm_verified','watchdog_verified',
                    'feedback_signs_verified','accept_feedback_rate_difference'):
            with self.assertRaises(ValueError): hardware_contract(dict(data,**{key:False}))
        for limit in (0,1,float('nan')):
            with self.assertRaises(ValueError): hardware_contract(dict(data,verified_wheel_torque_limit_nm=limit))


if __name__ == '__main__': unittest.main()
