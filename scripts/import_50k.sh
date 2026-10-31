#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$task_root/.venv-inference/bin/python" ]]; then
  echo 'Create .venv-inference and install requirements-inference.txt first; see README.md' >&2; exit 1
fi
export PYTHONPATH="$task_root/src/nino_real_rl:${PYTHONPATH:-}"
exec "$task_root/.venv-inference/bin/python" -m nino_real_rl.policy "$@"
