#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$PI5_SLAM_ROOT"
colcon build --base-paths src --symlink-install --executor sequential "$@"
