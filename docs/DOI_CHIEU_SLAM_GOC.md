# Đối chiếu với pi5-stm32-slam

Ngày 03/11/2026, đối chiếu source tại `/home/tiendat/project/pi5-stm32-slam`,
commit `49d6c05`, với `/home/tiendat/Downloads/nino-real-test` đang có thay đổi
chưa commit. Không hoàn tác những thay đổi đó. Chưa đọc được install hoặc phần
cứng trên Pi từ laptop; kết luận dưới đây chỉ áp dụng cho hai cây source.

## Phần giống nhau tại thời điểm đối chiếu (trước khi đổi driver)

- Toàn bộ file config của pi5_slam_bringup: robot, bridge, EKF, SLAM, Nav2 và DDS.
- URDF, RViz, scan_deskew, doctor và entry point của package bringup.
- Đường mapping: encoder/IMU → bridge → EKF; LiDAR → /scan_raw → deskew → /scan
  → SLAM Toolbox. EKF phát odom→base_footprint, SLAM phát map→odom.
- Driver `rplidar_ros`, executable `rplidar_composition`, cổng `/dev/rplidar`,
  baud 115200, scan mode Sensitivity, frame laser, angle_compensate=true.
- Rule udev, script env/run/build/save_map và khóa nguồn micro-ROS Agent.

Do đó không có bản sửa baud/driver/SLAM mới trong source gốc cần chép đè sang.
SLLidar là driver được dùng trong cấu hình Linorobot cũ, không phải driver của
bản pi5-stm32-slam này.

## Khác biệt được giữ nguyên

| Phần | Khác biệt trong nino-real-test | Quyết định |
|---|---|---|
| robot.launch.py | Thêm teleop, path_test, đầu ra Nav2 riêng cho PI/SAC | Giữ; chế độ mapping vẫn có cùng luồng và tham số như bản gốc |
| Bridge IMU | Từ chối timestamp cũ/trùng/tương lai; giữ timestamp nguồn | Giữ; không chép lại cách thay timestamp IMU bằng giờ nhận của bản gốc |
| Controller | Có nino_real_rl, teleop guard, PI/SAC và checkpoint | Giữ; bản gốc không có những phần này |
| install.sh | Thêm numpy và venv | Giữ cho inference; không thiếu dependency mapping so với bản gốc |
| doctor.sh | Bản ghép thiếu wrapper dù node doctor đã có | Đã bổ sung script chỉ đọc, cùng giao diện bản gốc |
| check_demo.sh | Chưa có script tích hợp demo tổng hợp | Không cần cho mapping thật; không dùng demo làm bằng chứng LiDAR hoạt động |
| publish_github.sh | Không ghép script phát hành repo gốc | Không cần cho chạy xe; tránh nhầm repo đích |

Không thay gain, giới hạn, watchdog, map, checkpoint hoặc nguồn TF.

## Bằng chứng và giới hạn về timeout

Người dùng cho biết trước đây đã thu được map thật. Tuy nhiên, tài liệu
`docs/VALIDATION.md` của bản gốc có ghi cả một lần mapping thật không có /scan,
/map và journal báo RESULT_OPERATION_TIMEOUT. Tài liệu đó không xác nhận bản
install đang dùng trên Pi hôm tạo map thành công.

Log hiện tại của người dùng cho thấy cả rplidar_ros và sllidar_ros2 chạy riêng
đều timeout, cùng lỗi USB `cp210x ... failed set request 0x12 status: -110`.
Chưa xác định nguyên nhân cuối cùng; không được kết luận chỉ do code mới hoặc
thiết bị hỏng. Hai bộ source giống nhau cũng không bảo đảm binary/install và
môi trường ROS trên Pi giống nhau.

## Chạy trên Pi không màn hình

Dừng mọi bringup/driver trước. Trong terminal SSH mới:

```bash
cd ~/nino-real-test
source /opt/ros/jazzy/setup.bash
# Chỉ cần nếu agent native chưa build trong nino-real-test:
source ~/pi5-stm32-slam-main/scripts/env.sh
source scripts/env.sh
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"
./scripts/build.sh --packages-select stm32_base_bridge pi5_slam_bringup
source scripts/env.sh
ros2 pkg prefix pi5_slam_bringup
./scripts/run.sh mapping allow_motor_enable:=false rviz:=false
```

Prefix phải thuộc `~/nino-real-test/install`. Nếu thư mục repo agent cũ khác,
thay đúng đường dẫn trước khi source; không chạy agent cũ đồng thời.

Terminal SSH thứ hai, cùng các biến môi trường ROS/DDS trên:

```bash
cd ~/nino-real-test
source scripts/env.sh
./scripts/doctor.sh mapping --seconds 15
```

Doctor không chạy driver, không bật motor, không phát lệnh chuyển động. Báo
`ready: false` kèm những topic/TF thiếu; không coi launch còn sống là map đã chạy.
Lệnh exit code 1 là kết quả chưa sẵn sàng, không tự nó xác định nguyên nhân.
RViz mở trên laptop theo TAO_MAP_TREN_PI.md. Chỉ bật motor thủ công sau khi
cảm biến, nguồn và vùng thử đã được kiểm tra.

## Kiểm chứng của lần ghép này

Chỉ kiểm tra offline các phép tính deskew, freshness/IMU callback và cú pháp
script; không mở serial, chạy Agent, mô phỏng, driver, SLAM hoặc motor.
16 test liên quan đạt; `bash -n scripts/doctor.sh` và `git diff --check` đạt.
Kết quả offline không xác nhận hết timeout hoặc đã tạo map trên Pi.

## Cập nhật sau đối chiếu: chuyển driver

Theo yêu cầu người dùng, cả hai repo hiện chuyển sang `sllidar_ros2/sllidar_node`.
Phần mô tả rplidar_ros ở trên là kết quả đối chiếu lịch sử, không phải driver
hiện hành. Xem README để fetch/build driver; cấu hình baud, scan mode, frame và
remap deskew không đổi. Việc đổi driver không xác nhận timeout đã hết.

## Khôi phục driver theo yêu cầu tiếp theo

Đã chuyển bringup và dependency lại `rplidar_ros/rplidar_composition`. Phần chuyển
sang SL ở trên là lịch sử; source SL đã tải và log được giữ, không chạy đồng thời.
