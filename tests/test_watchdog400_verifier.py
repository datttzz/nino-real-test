"""Known MCU-clock fixtures; never open ROS, serial or motors."""
import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('wd400',Path(__file__).resolve().parents[1]/'scripts/verify_watchdog400.py')
wd=importlib.util.module_from_spec(spec);spec.loader.exec_module(wd)


class VerifierTests(unittest.TestCase):
    def event(self,base=1000):
        data=[0.]*40
        data[22:]=[4001,400,2,1,20,base,(base+401)&0xffffffff,(base+402)&0xffffffff,
                   (base+403)&0xffffffff,(base+404)&0xffffffff,401,402,403,404,3,3,1,0]
        return wd.decode_debug(data)

    def test_completed_transmission_passes(self):
        self.assertTrue(wd.validate_event(self.event(),0))

    def test_old_event_and_enqueue_without_ack_never_pass(self):
        self.assertFalse(wd.validate_event(self.event(),1))
        event=self.event();event['tx_mask']=1
        self.assertFalse(wd.validate_event(event,0))
        event=self.event();event['queued_mask']=1
        self.assertFalse(wd.validate_event(event,0))

    def test_late_or_inconsistent_times_rejected(self):
        event=self.event();event.update(left_tx_tick=1501,left_tx_age_ms=501)
        with self.assertRaisesRegex(ValueError,'500ms'):wd.validate_event(event,0)
        event=self.event();event['left_tx_age_ms']=402
        with self.assertRaisesRegex(ValueError,'Inconsistent'):wd.validate_event(event,0)

    def test_tick_wrap_does_not_mix_host_clock(self):
        self.assertTrue(wd.validate_event(self.event(0xfffffff0),0))

    def test_old_firmware_and_wrong_profile_rejected(self):
        with self.assertRaises(ValueError):wd.decode_debug([0.]*22)
        data=[0.]*40;data[22:25]=[4001,500,2]
        with self.assertRaises(ValueError):wd.decode_debug(data)
        data[23]=400;data[39]=1
        with self.assertRaises(ValueError):wd.decode_debug(data)


if __name__=='__main__':unittest.main()
