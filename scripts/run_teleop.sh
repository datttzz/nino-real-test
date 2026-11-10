#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$PI5_SLAM_ROOT"
if [[ ! -f install/setup.bash ]]; then
  echo 'Build first: ./scripts/build.sh' >&2; exit 1
fi
if ! agent_prefix="$(ros2 pkg prefix micro_ros_agent 2>/dev/null)"; then
  echo 'micro_ros_agent not found: build/source the native Agent before teleop.' >&2; exit 1
fi
agent_binary="$agent_prefix/lib/micro_ros_agent/micro_ros_agent"
if ! agent_libraries="$(ldd "$agent_binary" 2>&1)" || [[ "$agent_libraries" == *'not found'* ]]; then
  echo "micro-ROS Agent cannot load its libraries: $agent_libraries" >&2; exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/pi5-stm32-slam-${UID}-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo 'Another project bringup is running; stop PI/SAC/Nav2 before teleop' >&2; exit 1; }
exec ros2 launch nino_real_rl teleop.launch.py "$@"
