#!/usr/bin/env bash
# Chỉ đọc topic/TF; không chạy bringup, bật motor hoặc tạo scan giả.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
mode="${1:-bench}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$mode" == demo ]]; then export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"; fi
exec ros2 run pi5_slam_bringup doctor --mode "$mode" "$@"
