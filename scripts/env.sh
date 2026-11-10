#!/usr/bin/env bash
# Source this file; it does not change the global shell configuration.
PI5_SLAM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is required: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html' >&2
  return 1
fi
source /opt/ros/jazzy/setup.bash
if [[ -f "$PI5_SLAM_ROOT/.agent_ws/install/setup.bash" ]]; then
  source "$PI5_SLAM_ROOT/.agent_ws/install/setup.bash"
  export LD_LIBRARY_PATH="$PI5_SLAM_ROOT/.agent_ws/install/xrceagent/lib:${LD_LIBRARY_PATH:-}"
fi
if [[ -f "$PI5_SLAM_ROOT/install/setup.bash" ]]; then
  source "$PI5_SLAM_ROOT/install/setup.bash"
fi
# Overlay của repo có thể dùng Agent native trong workspace Pi cũ. Prefix ROS
# tìm được binary nhưng không tự thêm thư viện XRCE xây ngoài ament vào loader.
if _nino_agent_prefix="$(ros2 pkg prefix micro_ros_agent 2>/dev/null)"; then
  _nino_xrce_lib="$(dirname -- "$_nino_agent_prefix")/xrceagent/lib"
  if [[ -d "$_nino_xrce_lib" ]]; then
    export LD_LIBRARY_PATH="$_nino_xrce_lib:${LD_LIBRARY_PATH:-}"
  fi
fi
unset _nino_agent_prefix _nino_xrce_lib
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -f "$PI5_SLAM_ROOT/src/pi5_slam_bringup/config/fastdds_unicast.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$PI5_SLAM_ROOT/src/pi5_slam_bringup/config/fastdds_unicast.xml}"
fi
