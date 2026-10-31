"""Keyboard inactivity must clear motion while zero heartbeat stays publishable."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/nino_real_rl'))
from nino_real_rl.keyboard import KeyboardCommand


class KeyboardTests(unittest.TestCase):
    def test_motion_keys_and_timeout(self):
        keys=KeyboardCommand()
        self.assertEqual(keys.current(10.),(0.,0.))
        keys.press('i',10.)
        self.assertEqual(keys.current(10.24),(.1,0.))
        self.assertEqual(keys.current(10.25),(0.,0.))
        keys.press('j',11.);self.assertEqual(keys.current(11.),(0.,.4))
        keys.press(',',12.);self.assertEqual(keys.current(12.),(-.1,0.))

    def test_stop_and_unknown_keys_clear_motion(self):
        keys=KeyboardCommand()
        for stop in ('k',' ','x'):
            keys.press('u',10.);keys.press(stop,10.1)
            self.assertEqual(keys.current(10.1),(0.,0.))

    def test_key_repeat_refreshes_only_real_motion_key_activity(self):
        keys=KeyboardCommand();keys.press('i',10.);keys.press('i',10.2)
        self.assertEqual(keys.current(10.4),(.1,0.))
        self.assertEqual(keys.current(10.5),(0.,0.))
        self.assertEqual(keys.current(9.),(0.,0.))

    def test_invalid_keyboard_limits(self):
        for value in (0.,-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):KeyboardCommand(key_timeout=value)


if __name__=='__main__':unittest.main()
