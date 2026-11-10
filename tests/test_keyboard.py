"""Keyboard inactivity must clear motion while zero heartbeat stays publishable."""
from pathlib import Path
import os
import select
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/nino_real_rl'))
from nino_real_rl.keyboard import KeyboardCommand, read_key


class KeyboardTests(unittest.TestCase):
    def test_motion_keys_and_timeout(self):
        keys=KeyboardCommand()
        self.assertEqual(keys.current(10.),(0.,0.))
        keys.press('i',10.)
        self.assertEqual(keys.current(10.59),(.5,0.))
        self.assertEqual(keys.current(10.6001),(0.,0.))
        keys.press('j',11.);self.assertEqual(keys.current(11.),(0.,.4))
        keys.press(',',12.);self.assertEqual(keys.current(12.),(-.5,0.))

    def test_stop_and_unknown_keys_clear_motion(self):
        keys=KeyboardCommand()
        for stop in ('k',' ','x'):
            keys.press('u',10.);keys.press(stop,10.1)
            self.assertEqual(keys.current(10.1),(0.,0.))

    def test_key_repeat_refreshes_only_real_motion_key_activity(self):
        keys=KeyboardCommand();keys.press('i',10.)
        # Key-repeat của laptop chờ 0,5 s trước ký tự lặp đầu tiên.
        self.assertEqual(keys.current(10.49),(.5,0.))
        keys.press('i',10.5)
        self.assertEqual(keys.current(11.),(.5,0.))
        self.assertEqual(keys.current(11.1001),(0.,0.))
        self.assertEqual(keys.current(9.),(0.,0.))

    def test_custom_hold_still_supported(self):
        keys=KeyboardCommand(speed=.12,key_timeout=.25);keys.press('i',1.)
        self.assertEqual(keys.current(1.2),(.12,0.))
        self.assertEqual(keys.current(1.3),(0.,0.))

    def test_partial_unicode_does_not_block_or_prefetch_next_key(self):
        reader,writer=os.pipe()
        try:
            os.write(writer,b'\xc3i')
            self.assertEqual(read_key(reader),'')
            self.assertTrue(select.select([reader],[],[],0)[0])
            self.assertEqual(read_key(reader),'i')
            os.close(writer);writer=None
            self.assertIsNone(read_key(reader))
        finally:
            os.close(reader)
            if writer is not None:os.close(writer)

    def test_invalid_keyboard_limits(self):
        for value in (0.,-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):KeyboardCommand(key_timeout=value)


if __name__=='__main__':unittest.main()
