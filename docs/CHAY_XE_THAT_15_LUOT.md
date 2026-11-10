# Chạy xe thật: 5 Nav-only, 5 PI, 5 PI + SAC80k

Cập nhật ngày 08/11/2026. Laptop mở RViz; Pi chạy agent, LiDAR, AMCL/Nav2,
controller và ghi dữ liệu. Mỗi phiên đúng **một goal**; người vận hành đưa xe
về vạch đầu sau khi đã disable motor. Không có vòng lặp tự chạy 15 lượt.

## 1. Những điều đã kiểm chứng và phần cần làm mỗi lượt

- Pi hiện dùng `d@192.168.0.102`, repo `~/nino-real-test`.
- Map: `maps/phong_that_20261104_122339.yaml` và ảnh PGM cùng tên.
- `hardware.local.json` trên Pi đã qua kiểm tra. Firmware CyberGear + SBUS có
  watchdog cấu hình 400 ms; hai lượt đo MCU/CAN đạt 403/404 ms. Đây không phải
  thời gian bánh dừng cơ học. Hồ sơ: `runs/watchdog400_verify_srt9tm5y/`.
- SAC tại `models/sac80k` đã nạp CPU và xác nhận đúng 80.000 step. Không train.
- Khi chạy, STM32 phải báo **RC:AUTO**, F nhả, cần giữa. E OFF trước khi bê xe;
  E ON chỉ khi xe đã sẵn sàng. `RC:LOST` là thiếu SBUS hợp lệ, khác `NO AGENT`.
- Pose AMCL là ước lượng định vị. Pose lưu chỉ giúp khởi tạo lại; không làm xe
  ngoài đời tự trở về đầu. Bạn đặt đúng vị trí/hướng đã đánh dấu và kiểm tra
  laser khớp tường mỗi lượt. Nếu chưa khớp, sửa 2D Pose Estimate trước khi chạy.

## 2. Nếu chỉ cần kết nối Pi–STM32 để kiểm tra cảm biến

Phần này chỉ dùng khi chưa mở phiên navigation. Mỗi phiên ở phần 4–8 đã tự
mở agent; không chạy thêm agent/bench/teleop song song.

**Terminal A trên laptop → SSH vào Pi:**

```bash
ssh d@192.168.0.102
```

Sau khi vào Pi:

```bash
cd ~/nino-real-test
source scripts/env.sh
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"

ls -l /dev/stm32_robot
./scripts/run.sh bench allow_motor_enable:=false rviz:=false
```

Giữ terminal này chạy. Trong **Terminal B, SSH vào Pi**, nạp cùng môi trường
như Terminal 2 ở phần 4, rồi chạy từng lệnh:

```bash
timeout 10s ros2 topic echo /motor_fb --once --qos-reliability best_effort
timeout 10s ros2 topic echo /imu/data --once --qos-reliability best_effort
timeout 10s ros2 topic echo /motor_status --once
```

Có encoder và IMU nghĩa là Pi–STM32 đã trao đổi dữ liệu. **Ctrl+C tại Terminal A
để đóng bench trước khi mở phiên Nav/PI/SAC.** Không cần nạp lại firmware.

## 3. Chuẩn bị bộ thử

Nếu đang có phiên cũ: disable motor, đóng phiên bằng Ctrl+C và chờ dòng
`Đã kết thúc phiên`. Không xóa các lượt cũ hoặc các lượt thất bại.

Đánh dấu trên sàn vị trí đầu, hướng xe và đích. Giữ cùng map/start/goal cho
ba nhóm. Nav2 vẫn có thể lập các đường khác nhau; báo cáo công khai hash đường.

## 4. Mở ba terminal theo thứ tự

### Terminal 1 — SSH vào Pi, tạo bộ mới và mở lượt đầu

```bash
ssh d@192.168.0.102
```

```bash
cd ~/nino-real-test
source scripts/env.sh
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"

export NINO_SUITE_DIR="$PWD/runs/real15_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$NINO_SUITE_DIR"
printf '%s\n' "$NINO_SUITE_DIR" > "$PWD/runs/latest_real_suite.txt"
echo "Bộ thử: $NINO_SUITE_DIR"

./scripts/run_logged_real.sh nav_only \
  "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  "$HOME/nino-real-test/hardware.local.json"
```

Giữ Terminal 1 chạy. Chỉ tạo bộ mới **một lần** cho 15 lượt. Nếu tiếp tục bộ
đang có hoặc mở lại Terminal 1, nạp môi trường ở trên nhưng thay đoạn tạo bộ bằng:

```bash
export NINO_SUITE_DIR="$(cat "$HOME/nino-real-test/runs/latest_real_suite.txt")"
```

### Chạy lại sau lỗi khởi động, không tạo bộ mới

Đã sửa lỗi `malformed launch argument 'initial_pose:='`: wrapper chỉ truyền
initial_pose khi có giá trị. Lượt đầu chưa lưu pose là bình thường, không cần
điền tọa độ giả để khởi động. Khi có lỗi khởi động, đợi phiên đóng rồi chạy
lại ở **Terminal 1 trên Pi**, giữ nguyên bộ hiện tại:

```bash
cd ~/nino-real-test
source scripts/env.sh
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"
export NINO_SUITE_DIR="$(cat "$PWD/runs/latest_real_suite.txt")"

./scripts/run_logged_real.sh nav_only \
  "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  "$HOME/nino-real-test/hardware.local.json"
```

Sau đó tiếp tục Terminal 2, Terminal 3 và lưu pose ở mục 5. Nếu nhóm đang thử
là PI hoặc SAC thì thay `nav_only` bằng `pi` hoặc `sac80k` tương ứng.
Không chạy lại đoạn tạo `real15_$(date ...)` chỉ vì một lần startup lỗi.

### Terminal 2 — SSH vào Pi, quản lý pose/enable/arm/dừng

```bash
ssh d@192.168.0.102
```

```bash
cd ~/nino-real-test
source scripts/env.sh
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"

SUITE="$(cat "$PWD/runs/latest_real_suite.txt")"
```

### Terminal 3 — RViz trực tiếp trên laptop, không SSH

```bash
cd /home/tiendat/Downloads/nino-real-test
source scripts/env.sh
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"

rviz2 -d "$PWD/src/pi5_slam_bringup/rviz/slam.rviz"
```

Giữ một cửa sổ RViz cho cả bộ. Fixed Frame = `map`; Map topic = `/map`,
Durability = Transient Local, Reliability = Reliable.

## 5. Map_server và AMCL phải active; lưu pose lượt đầu

Wrapper mới tự chạy kiểm tra định vị tối đa 90 giây. Nó chờ lifecycle manager
khởi động trước; nếu còn inactive/unconfigured thì đọc lại state và yêu cầu
activate/configure phù hợp. Không gửi transition khi node đã active hoặc đang
chuyển tiếp. Đây là phục hồi có giới hạn, không bảo đảm loại mọi lỗi DDS.

Tại **Terminal 2**, kiểm tra:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

Cả hai cần trả `active [3]`. Nếu chưa active, xem tiến trình tự phục hồi:

```bash
SESSION=$(find "$SUITE" -mindepth 2 -maxdepth 2 -type l -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)
tail -n 30 "$SESSION/localization_readiness.log"
```

Khi thành công sẽ có `READY: map_server active; amcl active; /map đã nhận`.
Nếu helper đã kết thúc lỗi (không còn chạy), có thể thử riêng một lần:

```bash
pgrep -af '[n]avigation_session.py ready'
```

**Chỉ khi không còn helper ready đang chạy**, dùng:

```bash
/usr/bin/python3 scripts/navigation_session.py ready --timeout 90
```

Nếu vẫn không active, giữ log, chưa enable/goal. Không chạy configure/activate
một cách mù quáng hoặc xóa phiên lỗi để coi như chưa xảy ra.

**Lưu pose đầu tiên, trước khi xe di chuyển:** tại Terminal 2 chạy lệnh sau
trước, rồi mới nhấn **2D Pose Estimate** trong RViz:

```bash
/usr/bin/python3 scripts/navigation_session.py save-pose \
  --map "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  --output "$SUITE/initial_pose.json"
```

Trong RViz chọn đúng vị trí, kéo mũi tên đúng hướng xe. Terminal 2 sẽ báo đã
lưu x/y/yaw. Kiểm tra laser khớp tường. Nếu pose chưa đúng: giữ xe đứng yên,
đổi tên bản đã lưu rồi chạy lại save-pose và chỉnh lại trong RViz:

```bash
mv "$SUITE/initial_pose.json" "$SUITE/initial_pose_truoc_chinh_$(date +%Y%m%d_%H%M%S).json"
```

**Các lượt sau:** wrapper tự tìm `$NINO_SUITE_DIR/initial_pose.json`, kiểm tra
frame/map YAML/ảnh và nạp x/y/yaw làm initial pose của AMCL. Chờ định vị và
kiểm tra laser. Không phải chọn lại pose nếu xe được đặt đúng và scan khớp.
Nếu chỉnh thủ công riêng một lượt, thao tác `/initialpose` được ghi trong bag;
file pose gốc không bị tự thay đổi. Không dùng file này cho map khác.

## 6. Chạy năm lượt Nav-only

Lượt đầu đã mở ở Terminal 1. Xe trên sàn, vùng chạy trống, RC:AUTO, E ON,
F nhả, cần giữa. IMU phải có dữ liệu; xem cách xử lý ở phần 9 nếu thiếu.

Tại **Terminal 2**:

```bash
ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: true}'
```

Chỉ khi `success: true`, chọn **một Nav2 Goal** trên RViz. Nhóm Nav-only không
gọi `/rl/arm`. Theo dõi:

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

Goal hiện tại: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED. Đừng lấy trạng thái goal cũ.
Kết thúc lượt: Ctrl+C dừng xem status ở Terminal 2 rồi disable:

```bash
ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: false}'
```

Terminal 1: Ctrl+C, đợi `Đã kết thúc phiên`. E OFF, đưa xe về vạch đầu.
Tại **Terminal 1**, lặp lệnh sau cho bốn lượt còn lại:

```bash
./scripts/run_logged_real.sh nav_only \
  "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  "$HOME/nino-real-test/hardware.local.json"
```

Mỗi lượt kiểm tra active, pose tự nạp và laser khớp; sau đó mới enable/goal.
Tổng cộng **5 phiên Nav-only**, mỗi phiên một goal.

## 7. Chạy năm lượt PI

Tại **Terminal 1**, mở một lượt:

```bash
./scripts/run_logged_real.sh pi \
  "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  "$HOME/nino-real-test/hardware.local.json"
```

Kiểm tra map_server/AMCL active, pose tự nạp, laser khớp và IMU có dữ liệu.
Tại **Terminal 2**, chạy từng lệnh, đọc phản hồi:

```bash
ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /rl/arm std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /rl/status --once --full-length
```

Enable và arm đều cần `success: true`. Chọn cùng đích trên RViz, theo dõi:

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

Kết thúc: Ctrl+C dừng xem status ở Terminal 2, rồi:

```bash
./scripts/stop_real.sh
```

Terminal 1 Ctrl+C, đợi đóng phiên, E OFF rồi đưa xe về đầu. Lặp cùng quy trình
đủ **5 phiên PI**. Nếu arm thất bại, đọc nguyên nhân; không gửi goal tiếp.

Nếu service trả arm thành công nhưng `/rl/status` lại báo `armed:false`,
controller đã bị guard disarm sau phản hồi service; chưa được chọn goal.
Bản chẩn đoán mới giữ lý do dừng trong `fault`, tách ghi chú thiếu observation
vào `observation_note`, hiển thị `motor_status_bits` và `motor_status_age_s`.
`Stale motor status` là status quá tuổi; `CAN feedback not fresh` là thiếu bit
feedback hai bánh; `CAN bus-off` là lỗi bus. Bản sửa status dùng hạn riêng
`motor_status_timeout: 0.35` giây vì firmware phát status khoảng 5 Hz (thực đo
cách nhau ~216 ms); hạn chung 250 ms đã trip ở age=279 ms. Đây là tăng riêng
freshness status, không áp dụng cho encoder/IMU: chúng vẫn 250 ms; watchdog
MCU vẫn 400 ms. Status quá 350 ms hoặc bit CAN/enable không hợp lệ vẫn disarm.
Không tiếp tục tăng hạn khi còn lỗi; giữ log để kiểm tra độ trễ/tải hệ thống.
Sau khi mở lại phiên, kiểm tra giá trị thực đang chạy:

```bash
ros2 param get /real_controller motor_status_timeout
ros2 topic echo /rl/status --once --full-length
```

Parameter cần là 0.35; diagnostic có `motor_status_timeout_s: 0.35`.

**Nếu status trễ 0,49 s dù hạn đã là 0,35 s:** không tăng hạn tiếp. Đã chuẩn
bị firmware phát status trên mỗi chu kỳ feedback 20 ms thay cho mỗi 10 chu kỳ;
debug vẫn 5 Hz, watchdog vẫn 400 ms. Bản trong
`Microros_H723_USB-main/artifacts/status50_20261108_i03_joab/` đã build/test
và **nạp qua ST-LINK, verify thành công ngày 08/11**. Bản flash trước đã sao
lưu đủ 1 MiB. Đang chờ nối lại STM32 với Pi để đo rate status/IMU/encoder và
kiểm chứng watchdog zero-torque sau nạp; chưa xác nhận status đạt 50 Hz thực
tế hoặc SAC đã hết lỗi. Trễ callback trong controller vẫn cần kiểm tra,
không bảo đảm thay nhịp phát sẽ loại mọi nguyên nhân trễ.
Trước goal đầu, thiếu path có thể chỉ là ghi chú; nó không thay thế lý do dừng
đã được latch. Bản sửa chỉ có hiệu lực khi mở lại phiên controller.

Nếu fault là `odom: Stale/future source timestamp`, kiểm tra nguồn và queue,
không tăng tiếp sensor_timeout. Bản sửa mới dùng subscriber IMU/odom depth=1,
loại mẫu cũ/lặp/tương lai mà không xóa cache hợp lệ còn mới, không làm mới
timestamp hoặc giờ nhận của cache bằng gói bị loại. Diagnostic có
`rejected_sensor_samples`. Khi cache IMU/odom thực sự hết hạn 250 ms, cả PI và
SAC vẫn phải disarm, kể cả không có callback mới; arm cũng cần nguồn còn tươi.
Đóng và mở lại phiên để áp dụng. Chưa được chọn goal khi `armed:false`.

Trong log ngày 08/11 đã thấy raw `/imu/data` liên tục nhưng `/imu/bridge`
ngắt khoảng 350 ms. Controller nay nhận trực tiếp `/imu/data` (cùng message,
frame, đơn vị và mounting), không chờ callback relay bridge. Queue IMU đầu
vào bridge cũng giảm về depth=1; EKF/odometry vẫn do bridge/EKF cũ sở hữu.
Mẫu raw mới nhận vẫn phải đúng timestamp 0..50 ms như gate bridge; cache của
controller vẫn hết hạn sau 250 ms. Không đổi timestamp hay tăng tuổi IMU.
Sau khi mở lại phiên, kiểm tra:

```bash
ros2 param get /real_controller imu_topic
ros2 topic echo /rl/status --once --full-length
```

Parameter và `imu_source` cần là `/imu/data`. Lỗi timestamp hiện ghi rõ nguồn
`imu`, `odom` hoặc `wheel`. Đó là lỗi tuổi dữ liệu, không phải lỗi lệch map.
Logger/bag vẫn giữ cả raw IMU và bridge; dữ liệu rung trong báo cáo hiện có
vẫn theo luồng logger đã công bố, xem coverage nếu bridge có gap.

## 8. Chạy năm lượt PI + SAC80k

Tại **Terminal 1**:

```bash
./scripts/run_logged_real.sh sac80k \
  "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  "$HOME/nino-real-test/hardware.local.json"
```

Làm đúng các bước kiểm tra/enable/arm/goal/dừng của phần PI. Lặp đủ **5 phiên
SAC80k**. Model chạy CPU deterministic; gain PI nền và giới hạn giữ nguyên.
Dữ liệu SAC nằm riêng trong `real_data/sac80k/`; Nav-only/PI nằm trong `runs/`.

## 9. Nếu mất IMU hoặc robot/map trong RViz

### Không có `/imu/data`

Tại **Terminal 2 trên Pi**, disable trước, giữ xe đứng yên và E OFF:

```bash
ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: false}'
timeout 10s ros2 topic echo /imu/data --once --qos-reliability best_effort
timeout 10s ros2 topic echo /imu/debug --once --field data --qos-reliability best_effort
```

Mảng debug tính chỉ số từ 0: `[0]` init status, `[1]` read valid, `[2]` sample
valid, `[3]` calibrated, `[4]` calibration count, `[16]` SPI error, `[17]` time
synced. Với firmware hiện tại, trạng thái đã hoạt động có calibrated=1 và
calibration count=3001. Nếu calibrated=0: chờ xe đứng yên tối thiểu 5 giây,
không chạm/bê xe rồi kiểm tra lại. Firmware chủ động không publish IMU chưa
hiệu chuẩn. Không giả dữ liệu, bỏ guard hoặc sửa cờ để ép chạy.

Nếu encoder/debug cũng mất, kiểm tra cáp và agent tại Terminal 2:

```bash
ls -l /dev/stm32_robot
pgrep -af '[m]icro_ros_agent|[r]os2 launch'
timeout 10s ros2 topic echo /motor_fb --once --qos-reliability best_effort
```

Nếu đang có phiên chạy, không mở agent thứ hai. Đóng phiên có kiểm soát rồi
kiểm tra bench theo phần 2. Nếu vẫn thiếu IMU sau khi đứng yên, giữ debug/log
để chẩn đoán SPI, timestamp hoặc calibration; không tự nạp firmware khác.
Phiên lỗi cảm biến phải được giữ và ghi nhận, không tính thành lượt thành công.

### Mất map, chỉ còn robot

Kiểm tra lifecycle và helper ở phần 5. Khi /map đã có nhưng RViz chưa hiện:
bật ô Map, Topic=/map, Transient Local, Reliable; tắt/bật lại ô Map. Fixed
Frame=map. AMCL active chưa đồng nghĩa đã có pose đúng: kiểm tra laser, đặt
lại 2D Pose Estimate nếu cần. Không phát static TF giả để lấp map→odom.

## 10. Đếm phiên và tạo báo cáo trên Pi

Sau khi đóng mọi phiên, tại **Terminal 1**:

```bash
SUITE="$(cat "$HOME/nino-real-test/runs/latest_real_suite.txt")"
for GROUP in nav_only pi sac80k; do
  echo "=== $GROUP ==="
  find "$SUITE/$GROUP" -mindepth 1 -maxdepth 1 -type l -printf '%f\n'
  echo 'Số phiên:'
  find "$SUITE/$GROUP" -mindepth 1 -maxdepth 1 -type l | wc -l
done
```

Mỗi nhóm cần đúng 5 phiên đã đóng, mỗi phiên một goal. Giữ mọi thất bại điều
khiển. Nếu có thêm phiên lỗi hạ tầng/thiếu goal, công khai chúng và chọn rõ
đường dẫn để phân tích; không thay lượt điều khiển thất bại bằng lượt tốt.

Riêng lỗi CLI initial_pose ngày 08/11, phiên `20261108_161021_iz3hzd` không có
goal đã được giữ nguyên log gốc và chuyển liên kết vào
`$SUITE/startup_failures/nav_only/`. Chi tiết công khai trong
`$SUITE/startup_failures.jsonl`; phiên này không nằm trong 5 lượt Nav.
Cách tách lỗi khởi động này không áp dụng để loại lượt đã chạy goal nhưng thất bại.

```bash
cd ~/nino-real-test
source scripts/env.sh
SUITE="$(cat "$PWD/runs/latest_real_suite.txt")"
/usr/bin/python3 scripts/compare_real_runs.py \
  --nav-dir "$SUITE/nav_only/"* \
  --pi-dir "$SUITE/pi/"* \
  --sac-dir "$SUITE/sac80k/"* \
  --expected-episodes 5 \
  --output-dir "$SUITE/report"
```

Đầu ra: `report.html`, `summary.csv`, `episodes.csv`. Nếu report đã tồn tại,
chọn thư mục mới, ví dụ `--output-dir "$SUITE/report_$(date +%H%M%S)"`.
RMSE dùng AMCL+odom TF, rung dùng IMU trừ trọng lực; torque là tín hiệu lệnh,
không phải phép đo mô-men độc lập. Mục không đủ dữ liệu là N/A. Cùng initial
pose không chứng minh đường tham chiếu, nhiễu hoặc vị trí thực giống hệt nhau.

## 11. Tải báo cáo về laptop và mở

Mở **terminal mới trên laptop, ngoài SSH**:

```bash
PI_HOST=192.168.0.102
SUITE=$(ssh "d@$PI_HOST" 'cat /home/d/nino-real-test/runs/latest_real_suite.txt')
LOCAL_REPORT="$HOME/Downloads/NINO_15_luot_$(date +%Y%m%d_%H%M%S)"
scp -r "d@$PI_HOST:$SUITE/report" "$LOCAL_REPORT"
xdg-open "$LOCAL_REPORT/report.html"
```

Nếu lấy log SAC về repo train sau này, đây vẫn là dữ liệu thô thực nghiệm;
không tự chuyển thành replay buffer hay bắt đầu train từ các lệnh này.
