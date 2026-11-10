"""Known-value offline fixtures; no real session or ROS/motor calls."""
import hashlib
import importlib.util
import json
import math
import csv
from pathlib import Path
import tempfile
import unittest
import zipfile

spec = importlib.util.spec_from_file_location(
    'compare_real_runs', Path(__file__).resolve().parents[1] / 'scripts/compare_real_runs.py')
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


class CompareTests(unittest.TestCase):
    def make_run(self, root, group, steps=80000):
        folder = root / group
        (folder / 'controller').mkdir(parents=True)
        files = []
        if group == 'sac80k':
            with zipfile.ZipFile(folder / 'model.zip', 'w') as archive:
                archive.writestr('data', json.dumps(dict(num_timesteps=steps)))
            digest = hashlib.sha256((folder / 'model.zip').read_bytes()).hexdigest()
            (folder / 'contract.json').write_text(json.dumps(dict(
                timesteps=steps, sha256={'model.zip': digest})))
            for name in ('model.zip', 'contract.json'):
                files.append(dict(source='/models/sac80k/' + name, snapshot=name,
                                  sha256=hashlib.sha256((folder / name).read_bytes()).hexdigest()))
        (folder / 'manifest.json').write_text(json.dumps(dict(group=group, files=files)))
        (folder / 'outcome.json').write_text(json.dumps(dict(nav2_outcome='ABORTED')))
        rows = [dict(ros_stamp=t, mode='pi' if group == 'pi' else 'pi_sac', armed=True,
                     cross_track=e, heading_error=.1, acceleration=[0, 2, 3],
                     wheel_target=[2, 2], wheel_velocity=[1, 1], total=[.2, -.2])
                for t, e in ((10, 0), (10.1, 3), (10.3, 4))]
        (folder / 'controller/real_test.jsonl').write_text(
            ''.join(json.dumps(r) + '\n' for r in rows))
        return folder

    def test_weighted_rmse_and_failure_not_discarded(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.make_run(Path(tmp), 'pi')
            m = report.load_run(run, 'pi')['metrics']
            self.assertAlmostEqual(m['amcl_cross_track_rmse_m'], math.sqrt(6))
            self.assertAlmostEqual(m['wheel_tracking_rmse_rad_s'], 1)
            self.assertEqual(m['nav2_outcome_operator_annotation'], 'ABORTED')
            self.assertIsNone(m['completion_time_s'])

    def test_gap_bad_timestamp_and_missing_imu(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.make_run(Path(tmp), 'pi')
            log = run / 'controller/real_test.jsonl'
            rows = [json.loads(s) for s in log.read_text().splitlines()]
            for row in rows:
                del row['acceleration']
            rows[2]['ros_stamp'] = 11
            rows.append(dict(rows[-1], ros_stamp=10.5))
            log.write_text(''.join(json.dumps(r) + '\n' for r in rows))
            m = report.load_run(run, 'pi')['metrics']
            self.assertEqual(m['rejected_rows'], 1)
            self.assertEqual(m['gaps_over_0.25s'], 1)
            self.assertAlmostEqual(m['weighted_coverage_s'], .1)
            self.assertIsNone(m['imu_vertical_accel_rms_m_s2'])

    def test_checkpoint_verified_and_html_offline(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pi = self.make_run(root, 'pi')
            sac = self.make_run(root, 'sac80k')
            path = report.build_report(pi, sac, root / 'report')
            self.assertIn('<svg', path.read_text())
            self.assertNotIn('<script src=', path.read_text())
            self.assertTrue((path.parent / 'summary.csv').exists())
            with self.assertRaises(FileExistsError):
                report.build_report(pi, sac, path.parent)

    def test_wrong_step_count_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            sac = self.make_run(Path(tmp), 'sac80k', steps=50000)
            with self.assertRaisesRegex(ValueError, '80000'):
                report.load_run(sac, 'sac80k')

    def test_three_runs_group_mean_std_and_reduction(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pi, sac = [], []
            for i in range(3):
                a = self.make_run(root / str(i), 'pi')
                b = self.make_run(root / str(i), 'sac80k')
                log = b / 'controller/real_test.jsonl'
                rows = [json.loads(s) for s in log.read_text().splitlines()]
                for row in rows:
                    row['cross_track'] *= .5
                    row['acceleration'] = [x * .5 for x in row['acceleration']]
                log.write_text(''.join(json.dumps(r) + '\n' for r in rows))
                pi.append(a)
                sac.append(b)
            path = report.build_group_report(pi, sac, root / 'report')
            with (path.parent / 'summary.csv').open() as f:
                summary = list(csv.DictReader(f))
            stat = next(r for r in summary if r['group'] == 'PI+SAC80k'
                        and r['metric'] == 'amcl_cross_track_rmse_m')
            self.assertAlmostEqual(float(stat['mean']), math.sqrt(6) / 2)
            self.assertAlmostEqual(float(stat['sample_std']), 0)
            self.assertEqual(stat['valid_episodes'], '3')
            self.assertIn('50.00%', path.read_text())
            self.assertIn('3 chưa có nhãn', path.read_text())
            with (path.parent / 'episodes.csv').open() as f:
                self.assertEqual(len(list(csv.DictReader(f))), 6)

    def test_duplicate_session_cannot_count_as_three_trials(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            a = self.make_run(root, 'pi')
            b = self.make_run(root, 'sac80k')
            with self.assertRaisesRegex(ValueError, 'Duplicate'):
                report.build_group_report([a, a, a], [b], root / 'report')


if __name__ == '__main__':
    unittest.main()
