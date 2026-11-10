"""Test recorder signal inheritance with a fake executable, never ROS/hardware."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest


class RecorderProcessTests(unittest.TestCase):
    def test_background_sigint_is_reset_and_process_can_close(self):
        script = (Path(__file__).resolve().parents[1] / 'scripts/run_logged_real.sh').read_text()
        launcher = script.split("python3 -c '", 1)[1].split("' \\\n", 1)[0]
        self.assertIn('--include-hidden-topics', script)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ready = root / 'ready'
            fake = root / 'ros2'
            fake.write_text('#!/usr/bin/python3\n'
                            'import signal, sys\n'
                            'from pathlib import Path\n'
                            'assert signal.getsignal(signal.SIGINT) != signal.SIG_IGN\n'
                            'signal.signal(signal.SIGINT, lambda *args: sys.exit(0))\n'
                            'Path(sys.argv[1]).write_text("ready")\n'
                            'while True: signal.pause()\n')
            fake.chmod(0o755)
            env = dict(os.environ, PATH=str(root) + ':' + os.environ['PATH'])
            process = subprocess.Popen([sys.executable, '-c', launcher, str(ready)], env=env,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertTrue(ready.exists())
                os.killpg(process.pid, signal.SIGINT)
                stdout, stderr = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, (stdout, stderr))
            finally:
                if process.poll() is None:
                    process.kill()
                process.communicate()


if __name__ == '__main__':
    unittest.main()
