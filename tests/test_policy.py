"""Kiểm tra bundle và inference giả offline; không khởi tạo ROS hay động cơ."""
from dataclasses import asdict
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/nino_real_rl'))
from nino_real_rl import policy
from nino_real_rl.core import Contract, OBSERVATIONS


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root/'source'
        self.source.mkdir()
        (self.source/'model.zip').write_bytes(b'offline test stand-in, not real weights')
        (self.source/'config.yaml').write_text(json.dumps(dict(environment=asdict(Contract()))))
        (self.source/'metadata.json').write_text(json.dumps(dict(
            observations=OBSERVATIONS, action_order=['left','right'], action_scale_nm=.3)))

    def model(self, steps):
        return Mock(num_timesteps=steps, predict=Mock(return_value=(np.array([.2,-.1]),None)))

    def bundle(self, steps):
        manifest = dict(format_version=1,timesteps=steps,observations=OBSERVATIONS,
                        environment=asdict(Contract()),sha256={
                            n:policy.digest(self.source/n)
                            for n in ('model.zip','config.yaml','metadata.json')})
        (self.source/'contract.json').write_text(json.dumps(manifest))
        return self.source

    def arguments(self, output):
        return ['--model',str(self.source/'model.zip'),'--config',str(self.source/'config.yaml'),
                '--metadata',str(self.source/'metadata.json'),'--output',str(output),
                '--confirm-source-contract']

    def test_loads_legacy_50k_and_new_80k_by_verified_manifest(self):
        for steps in (50000,80000):
            with self.subTest(steps=steps),patch.object(policy,'load_model',return_value=self.model(steps)):
                p=policy.Policy(self.bundle(steps))
                self.assertEqual(p.timesteps,steps)
                np.testing.assert_allclose(p.predict(np.zeros(23)),[.2,-.1])
                self.assertTrue(p.model.predict.call_args.kwargs['deterministic'])

    def test_rejects_model_step_mismatch(self):
        with patch.object(policy,'load_model',return_value=self.model(50000)):
            with self.assertRaisesRegex(ValueError,'timestep differs'):
                policy.Policy(self.bundle(80000))

    def test_rejects_changed_weights_before_deserialization(self):
        self.bundle(80000)
        (self.source/'model.zip').write_bytes(b'changed')
        with patch.object(policy,'load_model') as loader:
            with self.assertRaisesRegex(ValueError,'bundle changed'):policy.Policy(self.source)
            loader.assert_not_called()

    def test_rejects_manifest_scale_different_from_source_config(self):
        self.bundle(80000)
        path=self.source/'contract.json';manifest=json.loads(path.read_text())
        manifest['environment']['residual_limit']=.2
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError,'Manifest environment differs'):
            policy.Policy(self.source)

    def test_rejects_observation_order_or_action_scale_mismatch(self):
        config=json.loads((self.source/'config.yaml').read_text())
        original=json.loads((self.source/'metadata.json').read_text())
        for change in (dict(observations=list(reversed(OBSERVATIONS))),
                       dict(action_order=['right','left']),dict(action_scale_nm=.2)):
            with self.subTest(change=change),self.assertRaises(ValueError):
                policy.source_contract(config,dict(original,**change))

    def test_import_80k_requires_explicit_expected_steps_and_preserves_source(self):
        output=self.root/'sac80k'
        original=policy.digest(self.source/'model.zip')
        with patch.object(policy,'load_model',return_value=self.model(80000)):
            with self.assertRaises(SystemExit):policy.main(self.arguments(output))
            self.assertFalse(output.exists())
            policy.main(self.arguments(output)+['--expected-steps','80000'])
            manifest=json.loads((output/'contract.json').read_text())
            self.assertEqual(manifest['timesteps'],80000)
            self.assertEqual(policy.digest(output/'model.zip'),original)
            self.assertEqual(policy.digest(self.source/'model.zip'),original)
            with self.assertRaises(SystemExit):
                policy.main(self.arguments(output)+['--expected-steps','80000'])
            self.assertEqual(policy.digest(output/'model.zip'),original)

    def test_rejects_nonfinite_or_out_of_range_action(self):
        model=self.model(80000)
        with patch.object(policy,'load_model',return_value=model):
            p=policy.Policy(self.bundle(80000))
        for action in ([float('nan'),0.],[1.1,0.],[0.]):
            model.predict.return_value=(np.array(action),None)
            with self.subTest(action=action),self.assertRaises(ValueError):p.predict(np.zeros(23))


if __name__=='__main__':unittest.main()
