#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sudo install -m 0644 "$root/udev/99-pi5-stm32-slam.rules" /etc/udev/rules.d/99-pi5-stm32-slam.rules
sudo usermod -aG dialout "$(id -un)"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
echo 'Serial rules installed. Log out/in or reconnect over SSH for dialout membership.'
echo 'If the LiDAR adapter differs, set lidar_port:=/dev/serial/by-id/<device>.'
