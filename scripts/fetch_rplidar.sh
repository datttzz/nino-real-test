#!/usr/bin/env bash
# Fetch bản driver có sửa vòng đời; không chạy serial hoặc motor.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
target="$root/src/rplidar_ros"
patch="$root/patches/rplidar_ros-2.1.0-startup.patch"
revision=6fc037db67e234ad7a4272cc2331afeb92ed7200
if [[ ! -e "$target" ]]; then
  git clone --no-checkout --depth 1 --branch 2.1.0 https://github.com/Slamtec/rplidar_ros.git "$target"
  git -C "$target" checkout --detach "$revision"
fi
if [[ "$(git -C "$target" rev-parse --show-toplevel)" != "$target" ]] ||
   [[ "$(git -C "$target" rev-parse HEAD)" != "$revision" ]]; then
  echo "Different driver source at $target; preserved without changes." >&2
  exit 1
fi
if git -C "$target" apply --reverse --check "$patch" 2>/dev/null; then
  echo 'RPLIDAR startup patch already present.'
elif [[ -z "$(git -C "$target" status --porcelain)" ]]; then
  git -C "$target" apply --check "$patch"
  git -C "$target" apply "$patch"
  echo 'RPLIDAR startup patch applied.'
else
  echo "Local driver changes at $target; not overwritten." >&2
  exit 1
fi
