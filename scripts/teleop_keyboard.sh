#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
exec ros2 run nino_real_rl teleop_keyboard "$@"
