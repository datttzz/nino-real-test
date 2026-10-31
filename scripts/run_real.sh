#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$PI5_SLAM_ROOT"
if [[ ! -f install/setup.bash ]]; then
  echo 'Build first: ./scripts/build.sh' >&2; exit 1
fi
if [[ -d .venv-inference ]]; then
  source .venv-inference/bin/activate
  # colcon entry points use /usr/bin/python3: expose only this venv's packages.
  inference_site="$(.venv-inference/bin/python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
  export PYTHONPATH="$inference_site:${PYTHONPATH:-}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/pi5-stm32-slam-${UID}-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo 'Another project bringup is running in this ROS domain' >&2; exit 1; }
exec ros2 launch nino_real_rl real_test.launch.py "$@"
