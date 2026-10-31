#!/usr/bin/env bash
set -eo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
  echo 'This installer targets Ubuntu 24.04 (Noble).' >&2; exit 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'Install ROS 2 Jazzy ros-base first, then rerun this script:' >&2
  echo 'https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html' >&2
  exit 1
fi
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git curl ca-certificates libasio-dev libtinyxml2-dev libssl-dev \
  python3-colcon-common-extensions python3-pytest python3-yaml python3-numpy python3-venv \
  ros-jazzy-ros-base ros-jazzy-robot-localization ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-diagnostic-msgs \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-rplidar-ros \
  ros-jazzy-rmw-fastrtps-cpp ros-jazzy-fastrtps ros-jazzy-fastcdr \
  ros-jazzy-fastrtps-cmake-module ros-jazzy-rosidl-default-generators \
  ros-jazzy-rosidl-typesupport-fastrtps-cpp ros-jazzy-ament-lint-auto \
  ros-jazzy-ament-cmake-gtest
"$(dirname -- "${BASH_SOURCE[0]}")/build_agent.sh"
"$(dirname -- "${BASH_SOURCE[0]}")/build.sh"
echo 'Build complete. Configure serial access with ./scripts/setup_udev.sh, then start bench or demo mode.'
