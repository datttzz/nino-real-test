#!/usr/bin/env bash
# Tải driver chính thức vào workspace; không mở serial hoặc chạy LiDAR.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
target="$root/src/sllidar_ros2"
revision=34300099fadfc772965962dec837bf436706188f
if [[ ! -e "$target" ]]; then
  git clone --no-checkout --depth 1 https://github.com/Slamtec/sllidar_ros2.git "$target"
  git -C "$target" fetch --depth 1 origin "$revision"
  git -C "$target" checkout --detach "$revision"
fi
# Không ghi đè một bản driver khác hoặc có sửa đổi của người dùng.
if [[ "$(git -C "$target" rev-parse --show-toplevel)" != "$target" ]] ||
   [[ "$(git -C "$target" rev-parse HEAD)" != "$revision" ]] ||
   [[ -n "$(git -C "$target" status --porcelain)" ]]; then
  echo "Driver at $target differs from the pinned source; preserved without changes." >&2
  exit 1
fi
echo "SLLidar source ready: $revision"
