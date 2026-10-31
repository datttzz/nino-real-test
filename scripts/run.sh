#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
mode="${1:-bench}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  bench|mapping|navigation) export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ;;
  demo) export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}" ;;
  *) echo 'Usage: scripts/run.sh {bench|demo|mapping|navigation} [launch arguments]' >&2; exit 2 ;;
esac
if [[ ! -f "$PI5_SLAM_ROOT/install/setup.bash" ]]; then
  echo 'Build the project first: ./scripts/build.sh' >&2; exit 1
fi
runtime_dir="${XDG_RUNTIME_DIR:-/tmp}"
exec 9>"$runtime_dir/pi5-stm32-slam-${UID}-${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  echo "A project launch is already running in ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >&2; exit 1
fi
echo "Mode=$mode ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec ros2 launch pi5_slam_bringup robot.launch.py "mode:=$mode" "$@"
