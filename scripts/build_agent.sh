#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
python3 "$root/scripts/fetch_agent_sources.py"
agent_ws="$root/.agent_ws"
jobs="${BUILD_JOBS:-2}"
cmake -S "$agent_ws/src/spdlog" -B "$agent_ws/build/spdlog" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DSPDLOG_BUILD_SHARED=OFF -DSPDLOG_BUILD_EXAMPLE=OFF \
  -DCMAKE_INSTALL_PREFIX="$agent_ws/install/spdlog"
cmake --build "$agent_ws/build/spdlog" --parallel "$jobs"
cmake --install "$agent_ws/build/spdlog"
export CMAKE_PREFIX_PATH="$agent_ws/install/spdlog:${CMAKE_PREFIX_PATH:-}"
cmake -S "$agent_ws/src/xrceagent" -B "$agent_ws/build/xrceagent" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$agent_ws/install/xrceagent" \
  -DUAGENT_SUPERBUILD=OFF -DUAGENT_USE_SYSTEM_FASTDDS=ON \
  -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=ON \
  -DUAGENT_P2P_PROFILE=OFF -DUAGENT_CED_PROFILE=OFF -DUAGENT_BUILD_EXECUTABLE=OFF \
  -Dspdlog_DIR="$agent_ws/install/spdlog/lib/cmake/spdlog"
cmake --build "$agent_ws/build/xrceagent" --parallel "$jobs"
cmake --install "$agent_ws/build/xrceagent"
export CMAKE_PREFIX_PATH="$agent_ws/install/xrceagent:$CMAKE_PREFIX_PATH"
export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
cd "$agent_ws"
colcon build --base-paths src/micro_ros_msgs src/micro_ros_agent/micro_ros_agent \
  --executor sequential --cmake-args -DMICROROSAGENT_SUPERBUILD=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
echo "Native micro-ROS agent built in $agent_ws/install"
