"""Known-value fixtures for passive real-run logging; never open hardware."""
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


logger = module('measurement_logger', 'scripts/measure_real_run.py')
report = module('three_report', 'scripts/compare_real_runs.py')


class ThreeGroupTests(unittest.TestCase):
    def fixture(self, root, group, status=4):
        out = root/group
        (out/'measurements').mkdir(parents=True)
        files = []
        if group == 'sac80k':
            with zipfile.ZipFile(out/'model.zip', 'w') as archive:
                archive.writestr('data', json.dumps(dict(num_timesteps=80000)))
            digest = hashlib.sha256((out/'model.zip').read_bytes()).hexdigest()
            (out/'contract.json').write_text(json.dumps(dict(timesteps=80000, sha256={'model.zip':digest})))
            for name in ('model.zip','contract.json'):
                files.append(dict(source='/models/sac80k/'+name, snapshot=name,
                                  sha256=hashlib.sha256((out/name).read_bytes()).hexdigest()))
        (out/'manifest.json').write_text(json.dumps(dict(group=group, files=files, measurement_source='passive_v1')))
        (out/'outcome.json').write_text('{}')
        summary = dict(format_version=1, group=group, closed=True,
                       goals={'goal':dict(start=10.,terminal=10.4,status=status)},
                       references={'goal':dict(sha256='same-reference')}, rejected={})
        (out/'measurements/summary.json').write_text(json.dumps(summary))
        pose = [dict(ros_stamp=t, goal_id='goal', group=group, cross_track=e, heading_error=.1,
                     wheel_target=[2,2],wheel_velocity=[1,1])
                for t,e in [(10.,0.),(10.1,3.),(10.3,4.)]]
        imu = [dict(ros_stamp=10+i*.02,goal_id='goal',acceleration=[0,2,3]) for i in range(20)]
        for kind, rows in [('pose',pose),('imu',imu)]:
            (out/f'measurements/{kind}.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        return out

    def test_mission_excludes_reset_and_terminal_delay_keeps_first_receipt(self):
        mission = logger.Mission(10.)
        self.assertFalse(mission.status('old',1.,4,11.))
        self.assertTrue(mission.status('g',11.,2,11.1))
        self.assertIsNone(mission.at(10.9))
        self.assertEqual(mission.at(11.2),'g')
        mission.status('g',11.,4,12.)
        mission.status('g',11.,4,15.)
        self.assertEqual(mission.goals['g']['terminal'],12.)
        self.assertIsNone(mission.at(12.1))
        self.assertFalse(mission.status('bad',float('nan'),2,12.))
        self.assertFalse(mission.status('clock',13.,4,12.))

    def test_weighted_metrics_raw_imu_and_missing_nav_torque(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(Path(tmp),'nav_only')
            m = report.load_run(run,'nav_only')['metrics']
            self.assertAlmostEqual(m['amcl_cross_track_rmse_m'],math.sqrt(6))
            self.assertAlmostEqual(m['imu_lateral_accel_rms_m_s2'],2)
            self.assertAlmostEqual(m['imu_vertical_accel_rms_m_s2'],3)
            self.assertAlmostEqual(m['completion_time_s'],.4)
            self.assertEqual(m['nav2_outcome_from_status'],'SUCCEEDED')
            self.assertIsNone(m['command_torque_rms_nm'])

    def test_missing_imu_failure_and_bad_pose_timestamp_not_hidden(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(Path(tmp),'pi',status=6)
            (run/'measurements/imu.jsonl').write_text('')
            with (run/'measurements/pose.jsonl').open('a') as f:
                f.write(json.dumps(dict(ros_stamp=10.2,goal_id='goal'))+'\n')
            m = report.load_run(run,'pi')['metrics']
            self.assertIsNone(m['imu_vertical_accel_rms_m_s2'])
            self.assertEqual(m['nav2_outcome_from_status'],'ABORTED')
            self.assertEqual(m['rejected_rows'],1)
            self.assertIsNone(m['completion_time_s'])

    def test_multiple_goals_and_unclosed_logger(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(Path(tmp),'pi')
            p=run/'measurements/summary.json'
            summary=json.loads(p.read_text())
            summary['goals']['another']=dict(start=11.,terminal=12.,status=4)
            p.write_text(json.dumps(summary))
            m=report.load_run(run,'pi')['metrics']
            self.assertIsNotNone(m['invalid_session_reason'])
            self.assertIsNone(m['amcl_cross_track_rmse_m'])
            summary['closed']=False;p.write_text(json.dumps(summary))
            with self.assertRaisesRegex(ValueError,'not closed'):
                report.load_run(run,'pi')

    def test_fifteen_rows_three_columns_and_offline_html(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); groups={g:[] for g in ['nav_only','pi','sac80k']}
            for i in range(5):
                for group in groups:
                    groups[group].append(self.fixture(root/str(i),group,status=6 if i==4 else 4))
            page=report.build_group_report(groups['pi'],groups['sac80k'],root/'report',groups['nav_only'])
            text=page.read_text()
            self.assertIn('Nav-only Mean ± Std',text)
            self.assertIn('Nav2 SUCCEEDED 4/5',text)
            self.assertNotIn('<script src=',text)
            with (page.parent/'episodes.csv').open() as f:
                rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),15)
            self.assertEqual(sum(r['nav2_outcome_from_status']=='ABORTED' for r in rows),3)

    def test_logger_has_no_actuator_publishers(self):
        import ast
        tree=ast.parse((ROOT/'scripts/measure_real_run.py').read_text())
        calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertNotIn('create_publisher',calls)
        self.assertNotIn('create_service',calls)


if __name__=='__main__':
    unittest.main()
