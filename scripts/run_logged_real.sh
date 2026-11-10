#!/usr/bin/env bash
# Lưu từng phiên riêng; không tự enable/arm hoặc sửa controller.
set -eo pipefail
if [[ ${1:-} == --help || $# -ne 3 ]]; then
  echo 'Usage: scripts/run_logged_real.sh {nav_only|pi|sac80k} MAP.yaml hardware.local.json'
  echo 'Pi headless; enable/arm và đặt goal thủ công. Ctrl+C để dừng và đóng rosbag.'
  [[ ${1:-} == --help ]] && exit 0
  exit 2
fi
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$PI5_SLAM_ROOT"
group=$1
map_file=$(realpath -e -- "$2")
hardware_file=$(realpath -e -- "$3")
case "$group" in
  nav_only) controller=''; output_root="$PWD/runs/nav_only" ;;
  pi) controller=pi; output_root="$PWD/runs/pi" ;;
  sac80k) controller=pi_sac; output_root="$PWD/real_data/sac80k" ;;
  *) echo 'Chỉ nhận nav_only, pi hoặc sac80k.' >&2; exit 2 ;;
esac
if [[ ! -f install/setup.bash ]]; then
  echo 'Chạy ./scripts/build.sh trước.' >&2; exit 1
fi
initial_pose=''
initial_pose_file="${NINO_INITIAL_POSE_FILE:-}"
if [[ -z "$initial_pose_file" && -n "${NINO_SUITE_DIR:-}" ]]; then
  initial_pose_file="$NINO_SUITE_DIR/initial_pose.json"
fi
if [[ -n "$initial_pose_file" && -f "$initial_pose_file" ]]; then
  initial_pose=$(/usr/bin/python3 scripts/navigation_session.py read-pose \
    --file "$initial_pose_file" --map "$map_file")
  initial_pose_file=$(realpath -e -- "$initial_pose_file")
  echo "Nạp initial pose đã lưu: $initial_pose"
else
  initial_pose_file=''
  echo 'Chưa có pose lưu: lượt đầu đặt 2D Pose Estimate và lưu bằng navigation_session.py.'
fi
# Kiểm tra trước khi tạo phiên/rosbag; lỗi cấu hình không phải một lượt thử xe.
if [[ "$group" != nav_only ]]; then
PYTHONPATH="$PWD/src/nino_real_rl:${PYTHONPATH:-}" python3 - "$hardware_file" <<'PY'
import sys
from nino_real_rl.core import load_hardware_contract
try:
    load_hardware_contract(sys.argv[1])
except ValueError as exc:
    print(f'Controller startup blocked: {exc}', file=sys.stderr)
    sys.exit(2)
PY
fi
ros2 bag record --help >/dev/null
mkdir -p "$output_root"
session_dir=$(mktemp -d "$output_root/$(date +%Y%m%d_%H%M%S)_XXXXXX")
mkdir "$session_dir/controller" "$session_dir/config"
export NINO_CAPTURE_DIR="$session_dir" NINO_CAPTURE_MAP="$map_file"
export NINO_CAPTURE_HARDWARE="$hardware_file" NINO_CAPTURE_GROUP="$group"
export NINO_CAPTURE_INITIAL_POSE="$initial_pose" NINO_CAPTURE_INITIAL_POSE_FILE="$initial_pose_file"
python3 - <<'PY'
import hashlib, json, os, shutil, subprocess
from datetime import datetime, timezone
from pathlib import Path
import yaml
out = Path(os.environ['NINO_CAPTURE_DIR'])
map_path = Path(os.environ['NINO_CAPTURE_MAP'])
image = Path(yaml.safe_load(map_path.read_text())['image']).expanduser()
if not image.is_absolute():
    image = map_path.parent / image
files = [map_path, image.resolve(), Path(os.environ['NINO_CAPTURE_HARDWARE']),
         Path('scripts/measure_real_run.py'),
         Path('scripts/navigation_session.py'),
         Path('src/pi5_slam_bringup/launch/robot.launch.py'),
         Path('src/nino_real_rl/nino_real_rl/core.py'),
         Path('src/nino_real_rl/nino_real_rl/node.py'),
         Path('src/nino_real_rl/nino_real_rl/features.py'),
         Path('src/nino_real_rl/config/real.yaml'),
         Path('src/pi5_slam_bringup/config/robot.yaml'),
         Path('src/pi5_slam_bringup/config/bridge.yaml'),
         Path('src/pi5_slam_bringup/config/nav2.yaml')]
if os.environ['NINO_CAPTURE_INITIAL_POSE_FILE']:
    files.append(Path(os.environ['NINO_CAPTURE_INITIAL_POSE_FILE']))
for package, names in (
    ('nino_real_rl', ('real.yaml',)),
    ('pi5_slam_bringup', ('robot.yaml', 'bridge.yaml', 'nav2.yaml')),
):
    prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', package], text=True).strip()
    files += [Path(prefix) / 'share' / package / 'config' / name for name in names]
if os.environ['NINO_CAPTURE_GROUP'] == 'sac80k':
    files += [Path('models/sac80k') / n for n in
              ('model.zip', 'contract.json', 'config.yaml', 'metadata.json')]
records = []
for i, path in enumerate(files):
    data = path.read_bytes()
    # Giữ cả trọng số nguồn để truy nguyên chính xác policy, không ghi đè bundle.
    target = out / 'config' / f'{i:02d}_{path.name}'
    shutil.copy2(path, target)
    records.append(dict(source=str(path.resolve()), snapshot=str(target.relative_to(out)),
                        sha256=hashlib.sha256(data).hexdigest()))
rev = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True)
status = subprocess.run(['git', 'status', '--short'], capture_output=True, text=True)
(out / 'manifest.json').write_text(json.dumps(dict(
    created_utc=datetime.now(timezone.utc).isoformat(),
    group=os.environ['NINO_CAPTURE_GROUP'], git_revision=rev.stdout.strip(),
    measurement_source='passive_v1',
    git_status=status.stdout, files=records,
    requested_initial_pose=os.environ['NINO_CAPTURE_INITIAL_POSE'] or None,
    initial_pose_file=os.environ['NINO_CAPTURE_INITIAL_POSE_FILE'] or None,
    pose_source='AMCL + odometry TF; estimated, not absolute ground truth',
    body_velocity_source='encoder/IMU odometry estimate',
    torque_source=('N/A: firmware velocity loop' if os.environ['NINO_CAPTURE_GROUP'] == 'nav_only'
                   else 'controller commands, not measured wheel torque'),
    training_ready=False, result='not annotated'), ensure_ascii=False, indent=2))
(out / 'outcome.json').write_text(json.dumps(dict(
    nav2_outcome=None, amcl_task_pass=None, operator_stop_reason=None,
    notes='Điền theo dữ liệu và quan sát; không suy ra thành công từ mã thoát launch.'
), ensure_ascii=False, indent=2))
PY
if [[ -n "${NINO_SUITE_DIR:-}" ]]; then
  mkdir -p "$NINO_SUITE_DIR/$group"
  ln -s "$session_dir" "$NINO_SUITE_DIR/$group/$(basename "$session_dir")"
fi
bag_pid=''
logger_pid=''
readiness_pid=''
close_bag() {
  if [[ -n "$readiness_pid" ]]; then
    kill -TERM "$readiness_pid" 2>/dev/null || true
    wait "$readiness_pid" 2>/dev/null || true
    readiness_pid=''
  fi
  if [[ -n "$logger_pid" ]]; then
    kill -INT -- "-$logger_pid" 2>/dev/null || true
    wait "$logger_pid" 2>/dev/null || true
    logger_pid=''
  fi
  if [[ -n "$bag_pid" ]]; then
    kill -INT -- "-$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
    bag_pid=''
  fi
}
trap close_bag EXIT
# INT cũng tới launch đang chạy foreground; cho launch thực hiện shutdown chuẩn.
trap ':' INT
trap 'exit 143' TERM
# Bash background jobs inherit SIGINT ignored; reset it before exec so Ctrl+C
# cleanup can close the bag and write metadata.yaml instead of leaving an orphan.
python3 -c 'import os, signal, sys
if os.getpgrp() != os.getpid():
    os.setsid()
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp("ros2", ["ros2", *sys.argv[1:]])' \
  bag record --include-hidden-topics -s sqlite3 -o "$session_dir/rosbag" \
  /motor_fb /motor_status /joint_states /imu/data /imu/bridge \
  /imu/debug /odom/unfiltered /odom /tf /tf_static /scan /amcl_pose /map /plan \
  /cmd_vel /cmd_vel_nav /cmd_vel_smoothed \
  /initialpose /goal_pose /rl/cmd_vel_reference /rl/status /rl/torque_preview \
  /navigate_to_pose/_action/status /rosout \
  >"$session_dir/recorder.log" 2>&1 &
bag_pid=$!
sleep 1
if ! kill -0 "$bag_pid" 2>/dev/null; then
  cat "$session_dir/recorder.log" >&2
  echo "Recorder không khởi động được; chưa mở controller. Log: $session_dir" >&2
  exit 1
fi
# Logger chung chỉ subscribe; không tạo publisher/service điều khiển motor.
python3 -c 'import os, signal, sys
if os.getpgrp() != os.getpid():
    os.setsid()
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execv("/usr/bin/python3", ["/usr/bin/python3", *sys.argv[1:]])' \
  scripts/measure_real_run.py --session-dir "$session_dir" --group "$group" \
  --robot-config "$(ros2 pkg prefix pi5_slam_bringup)/share/pi5_slam_bringup/config/robot.yaml" \
  >"$session_dir/measurement_logger.log" 2>&1 &
logger_pid=$!
for attempt in $(seq 1 50); do
  [[ -f "$session_dir/measurements/READY" ]] && break
  kill -0 "$logger_pid" 2>/dev/null || break
  sleep .1
done
if [[ ! -f "$session_dir/measurements/READY" ]] || ! kill -0 "$logger_pid" 2>/dev/null; then
  cat "$session_dir/measurement_logger.log" >&2
  echo 'Logger chưa sẵn sàng; chưa mở bringup.' >&2
  exit 1
fi
echo "Lưu phiên tại: $session_dir"
echo 'Mỗi phiên đúng một goal. Đợi dữ liệu, đặt initial pose rồi enable/arm thủ công.'
# Chỉ chuyển lifecycle định vị khi cần; không enable, arm hoặc gửi goal.
/usr/bin/python3 -u scripts/navigation_session.py ready --timeout 90 \
  --result "$session_dir/localization_readiness.json" \
  >"$session_dir/localization_readiness.log" 2>&1 &
readiness_pid=$!
echo "Kiểm tra map/AMCL: $session_dir/localization_readiness.log"
nav_args=(map:="$map_file" allow_motor_enable:=true rviz:=false)
args=(map:="$map_file" hardware_file:="$hardware_file"
      log_dir:="$session_dir/controller" rviz:=false)
# ROS 2 CLI không nhận name:= rỗng; lượt đầu dùng default của launch.
if [[ -n "$initial_pose" ]]; then
  nav_args+=(initial_pose:="$initial_pose")
  args+=(initial_pose:="$initial_pose")
fi
if [[ "$group" != nav_only ]]; then args+=(controller:="$controller"); fi
if [[ "$group" == sac80k ]]; then args+=(checkpoint:="$PWD/models/sac80k"); fi
set +e
if [[ "$group" == nav_only ]]; then
  ./scripts/run.sh navigation "${nav_args[@]}" \
    2>&1 | tee "$session_dir/runner.log"
  launch_status=${PIPESTATUS[0]}
else
  ./scripts/run_real.sh "${args[@]}" 2>&1 | tee "$session_dir/runner.log"
  launch_status=${PIPESTATUS[0]}
fi
set -e
close_bag
printf '%s\n' "$launch_status" > "$session_dir/launch_exit_code.txt"
if [[ ! -f "$session_dir/rosbag/metadata.yaml" ]]; then
  echo 'Rosbag chưa có metadata.yaml; kiểm tra recorder.log, không coi phiên là đầy đủ.' >&2
fi
echo "Đã kết thúc phiên: $session_dir (launch exit=$launch_status)"
exit "$launch_status"
