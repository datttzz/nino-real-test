#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
timeout 3s ros2 service call /rl/arm std_srvs/srv/SetBool '{data: false}' || true
timeout 3s ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: false}'
