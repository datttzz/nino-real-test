"""Import an owned/trusted SB3 SAC checkpoint; CPU-only deterministic inference."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import numpy as np
import yaml

from .core import OBSERVATIONS, Contract


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_model(path):
    import torch
    from stable_baselines3 import SAC
    torch.set_num_threads(1)
    model = SAC.load(str(path), device='cpu')
    if model.observation_space.shape != (23,) or model.action_space.shape != (2,):
        raise ValueError('Checkpoint must have 23 observations and 2 actions')
    if not (np.all(model.observation_space.low == -10) and
            np.all(model.observation_space.high == 10) and
            np.all(model.action_space.low == -1) and np.all(model.action_space.high == 1)):
        raise ValueError('Checkpoint spaces differ from supplied training code')
    return model


class Policy:
    def __init__(self, directory):
        directory = Path(directory).expanduser().resolve()
        manifest = json.loads((directory/'contract.json').read_text())
        for name in ('model.zip', 'config.yaml', 'metadata.json'):
            if digest(directory/name) != manifest['sha256'][name]:
                raise ValueError('Checkpoint bundle changed: '+name)
        if manifest['observations'] != OBSERVATIONS or manifest['timesteps'] != 50000:
            raise ValueError('Unsupported checkpoint contract')
        self.contract = Contract(**manifest['environment'])
        self.model = load_model(directory/'model.zip')
        if self.model.num_timesteps != 50000:
            raise ValueError('Model is not the imported 50k checkpoint')

    def predict(self, obs):
        action, _ = self.model.predict(obs, deterministic=True)
        action = np.asarray(action, dtype=float)
        if action.shape != (2,) or not np.all(np.isfinite(action)) or np.any(np.abs(action) > 1.000001):
            raise ValueError('Nonfinite/out-of-range policy action')
        return np.clip(action, -1, 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--config', required=True, help='Original run config.yaml')
    parser.add_argument('--metadata', required=True, help='Original run metadata.json')
    parser.add_argument('--output', required=True)
    parser.add_argument('--confirm-source-contract', action='store_true',
                        help='Original run used supplied unwrapped 23D trainer, PI .30/.10/Kd0 and limits; no VecNormalize/history')
    args = parser.parse_args()
    if not args.confirm_source_contract:
        parser.error('Original controller config/wrappers cannot be inferred from model weights; verify and use --confirm-source-contract')
    config = yaml.safe_load(Path(args.config).read_text())
    metadata = json.loads(Path(args.metadata).read_text())
    if metadata.get('observations') != OBSERVATIONS or metadata.get('action_order') != ['left', 'right']:
        parser.error('Original metadata does not match observation/action order')
    env = config['environment']
    contract = Contract(**{k: env[k] for k in ('max_path_deviation', 'residual_limit', 'step_seconds')})
    if metadata.get('action_scale_nm') != contract.residual_limit:
        parser.error('Action scale mismatch between metadata and config')
    model = load_model(args.model)
    if model.num_timesteps != 50000:
        parser.error(f'Expected exactly 50000 steps; got {model.num_timesteps}')
    probe, _ = model.predict(np.zeros(23, dtype=np.float32), deterministic=True)
    if np.asarray(probe).shape != (2,) or not np.all(np.isfinite(probe)):
        parser.error('Checkpoint inference failed')
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        parser.error('Output exists; choose a new bundle directory')
    output.parent.mkdir(parents=True, exist_ok=True)
    # Publish the bundle only after all validation/copy steps succeed.
    with tempfile.TemporaryDirectory(dir=output.parent) as temporary:
        stage = Path(temporary)/'bundle'; stage.mkdir()
        for source, name in ((args.model, 'model.zip'), (args.config, 'config.yaml'), (args.metadata, 'metadata.json')):
            shutil.copyfile(source, stage/name)
        manifest = dict(format_version=1, timesteps=int(model.num_timesteps),
                        observations=OBSERVATIONS, environment=asdict(contract),
                        source_contract_operator_confirmed=True,
                        runtime='CPU deterministic; no learning',
                        sha256={n: digest(stage/n) for n in ('model.zip','config.yaml','metadata.json')})
        (stage/'contract.json').write_text(json.dumps(manifest, indent=2)+'\n')
        stage.rename(output)
    print(f'Imported verified 50k model bundle: {output}')


if __name__ == '__main__':
    main()
