#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
target="${1:-$PI5_SLAM_ROOT/maps/map_$(date +%Y%m%d_%H%M%S)}"
mkdir -p -- "$(dirname -- "$target")"
if [[ -e "$target.yaml" || -e "$target.pgm" ]]; then
  echo "Map already exists: $target. Choose another name." >&2; exit 1
fi
exec ros2 run nav2_map_server map_saver_cli -f "$target" \
  --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
