#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
timeout 3s ros2 service call /teleop/arm std_srvs/srv/SetBool '{data: false}' || true
timeout 3s ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: false}'
