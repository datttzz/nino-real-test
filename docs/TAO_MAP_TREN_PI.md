# Tạo, lưu và dùng map thật trong nino-real-test

Thẻ Pi đang có repo tại `/home/d/nino-real-test`; trên Pi đăng nhập tài khoản
`d` thì dùng `~/nino-real-test`. Các lệnh chạy ROS dưới đây thực hiện trên **Pi**
(trực tiếp hoặc terminal SSH), không chạy trên laptop khi thẻ còn cắm ở laptop.
Không chạy đồng thời bringup cũ, teleop bringup, Nav2 hoặc PI/SAC khác.

Khi tạo map: LiDAR `/scan` + odometry/IMU → SLAM Toolbox → `/map`. Điều khiển
bằng bàn phím qua guard; chưa dùng model SAC. Map được lưu trong `maps/` của repo.

## 1. Build trên Pi sau khi lắp lại thẻ

Sau khi copy xong, tháo thẻ an toàn khỏi laptop, lắp vào Pi rồi khởi động.

```bash
cd ~/nino-real-test
source /opt/ros/jazzy/setup.bash
```

Trên thẻ hiện có agent đã build ở repo phần cứng cũ. Có thể nạp môi trường
đó để dùng lại agent, không khởi động bringup cũ:

```bash
source "$HOME/pi5-stm32-slam-main/scripts/env.sh"
cd ~/nino-real-test
./scripts/build.sh
```

Nếu không dùng agent cũ, build agent riêng một lần bằng
`./scripts/build_agent.sh` rồi `./scripts/build.sh` theo README. Không copy
`build/`, `install/` hay venv x86 của laptop sang Pi ARM64.

## 2. Chuẩn bị chung cho mỗi terminal trên Pi

Mở 4 terminal trên Pi/SSH. **Mỗi terminal** chạy:

```bash
cd ~/nino-real-test
export ROS_DOMAIN_ID=0
if [ ! -f .agent_ws/install/setup.bash ] && [ -f "$HOME/pi5-stm32-slam-main/scripts/env.sh" ]; then
  source "$HOME/pi5-stm32-slam-main/scripts/env.sh"
fi
source ~/nino-real-test/scripts/env.sh
```

Mọi node/firmware phải dùng cùng domain. Dùng 0 ở đây nếu hệ thống của bạn
đã dùng 0; nếu firmware dùng domain khác thì thay thống nhất, không đoán.

## 3. Terminal 1: LiDAR + encoder/IMU + SLAM

```bash
./scripts/run.sh mapping allow_motor_enable:=true rviz:=false
```

Script chưa tự bật motor. Lệnh trên dành cho Pi không có màn hình; mở
RViz trên laptop theo mục 9. `/scan_raw` được deskew thành `/scan` khi cấu hình
`scan_deskew: true`; SLAM dùng `/scan`. Dùng RViz Fixed Frame **map**, bật Map,
LaserScan và RobotModel. Khi SLAM có dữ liệu, map xuất hiện và lớn dần khi đi.

Nếu cổng mặc định không tồn tại, kiểm tra `ls -l /dev/serial/by-id/` và truyền
đúng cổng, ví dụ `stm32_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0`. Tên này chỉ
là ví dụ, không xác nhận thiết bị thực tế. Không sửa baudrate/scan mode tùy ý.

## 4. Terminal 2: chỉ chạy teleop guard

```bash
ros2 run nino_real_rl teleop_guard --ros-args \
  --params-file "$HOME/nino-real-test/src/nino_real_rl/config/teleop.yaml"
```

**Không dùng `run_teleop.sh` cùng mapping**: script đó khởi động thêm bridge/agent
và bị khóa để tránh chạy trùng. Guard riêng dùng phần cứng Terminal 1 đã mở.
Giới hạn bàn phím hiện là 0,10 m/s và 0,40 rad/s; motor ban đầu DISARMED.

## 5. Terminal 3: bàn phím

```bash
./scripts/teleop_keyboard.sh
```

Giữ bàn phím ở trạng thái zero khi arm: không nhấn phím di chuyển, nhấn `k`
hoặc Space để dừng. Keyboard có heartbeat zero nên có thể arm ở terminal khác.

## 6. Terminal 4: kiểm tra dữ liệu, enable/arm

Chạy từng lệnh, nhấn Ctrl+C sau vài dòng `topic hz` để chuyển sang lệnh sau:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /motor_fb
ros2 topic echo /motor_status --once
ros2 topic info /map
```

Phải có scan/odom/encoder mới và motor status hợp lệ. Sau khi đã kiểm tra chiều
bánh, có người giữ vùng thử và sẵn dừng phần cứng, enable/arm thủ công:

```bash
ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /teleop/arm std_srvs/srv/SetBool '{data: true}'
```

Chỉ điều khiển khi cả hai trả `success: true`; nếu không, đọc `message`, không
bỏ guard. Quay về Terminal 3: `i` tiến, `,` lùi, `j/l` quay, `k`/Space dừng.
Đi chậm, quét đủ tường/góc và quay lại vùng đã đi để quan sát map ghép có khớp
không. Teleop mapping không có Nav2/collision monitor: người điều khiển tránh
chướng ngại và không di chuyển khi LiDAR/odom lỗi. Lệnh zero không thay phanh cơ.

## 7. Terminal 4: dừng xe và lưu map

Nhấn `k` ở Terminal 3, rồi dừng/disable bằng:

```bash
./scripts/stop_teleop.sh
```

Giữ Terminal 1 chạy SLAM trong lúc lưu. Ở Terminal 4:

```bash
MAP_BASE="$HOME/nino-real-test/maps/map_$(date +%Y%m%d_%H%M%S)"
./scripts/save_map.sh "$MAP_BASE"
ls -lh "$MAP_BASE.yaml" "$MAP_BASE.pgm"
cat "$MAP_BASE.yaml"
```

Map saver tạo YAML và ảnh PGM khi dùng cấu hình mặc định này. Nếu driver map
saver chọn định dạng ảnh khác thì dùng đúng tên ở trường `image:` trong YAML.
Không ghi đè map cũ; script từ chối nếu tên đã tồn tại. Ghi lại đường dẫn YAML
được in ra. Lưu occupancy map không đồng nghĩa lưu posegraph SLAM để tiếp tục
mapping từ phiên cũ.

Sau khi lưu thành công, Ctrl+C Terminal 3, Terminal 2 rồi Terminal 1.

## 8. Các file cấu hình cần hiểu

| File | Nội dung | Cách dùng |
|---|---|---|
| `maps/<tên>.yaml` + ảnh | Map đã đo, resolution, origin, ngưỡng occupied/free | Dùng nguyên thông số map saver. Không đổi origin/resolution để ép scan khớp. Chép cả ảnh đi cùng YAML. |
| `src/pi5_slam_bringup/config/robot.yaml` | Cổng, baud, scan mode, vị trí/hướng LiDAR, kích thước bánh | Phải khớp phần cứng và đo thực tế. Hiện laser_xyz=[0.223,0,0.1915] m, laser_rpy=[0,0,0] rad chỉ là giá trị cấu hình đang có, không phải phép đo mới. |
| `src/pi5_slam_bringup/config/slam.yaml` | SLAM frames map/odom/base_footprint, scan_topic=/scan, resolution=0.05 m | Dùng khi tạo map; chưa cần chỉnh tuning nếu chưa thấy lỗi cụ thể. |
| `src/pi5_slam_bringup/config/nav2.yaml` | AMCL/Nav2 dùng map đã lưu | AMCL đọc scan và định vị xe trên map; không dùng đồng thời SLAM mapping để cùng phát map→odom. |

Sau khi sửa file source cấu hình, chạy `./scripts/build.sh` trên Pi trước khi
launch lại. Vị trí LiDAR sai, chiều encoder sai hoặc trượt bánh có thể làm map
méo; không chữa bằng cách sửa tùy tiện metadata của ảnh map.

Để nạp map đã lưu và kiểm tra AMCL, mở lại Terminal 1 (không chạy mapping):

```bash
read -e -r -p 'Đường dẫn YAML của map vừa lưu: ' REAL_MAP
./scripts/run_real.sh controller:=shadow map:="$REAL_MAP" rviz:=false
```

RViz Fixed Frame=map → **2D Pose Estimate** đúng chỗ/hướng xe → quan sát điểm
scan trùng tường trên ảnh map. Shadow không chạy motor. Chưa cần nạp SAC để
kiểm tra LiDAR/TF/AMCL. Chỉ chuyển sang PI/SAC sau các bước kiểm chứng riêng
trong README; teleop mapping không chứng minh giao diện mô-men firmware.

## 9. Nếu xem RViz trên laptop

Pi và laptop phải chung mạng, cùng domain/RMW và profile Fast DDS của repo.
Không giữ `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` nếu cần thấy node máy khác.
Đặt các biến sau ở cả hai máy **trước khi khởi động node**:

```bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Trên laptop, mở terminal riêng với source của repo xe thật:

```bash
cd /home/tiendat/Downloads/nino-real-test
source scripts/env.sh
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"
ros2 node list
rviz2 -d "$PWD/src/pi5_slam_bringup/rviz/slam.rviz"
```

Repo dùng portBase=7650 nên phải dùng cùng profile trên Pi và laptop; script
env.sh ở Pi nạp profile của repo. Nếu `node list` không thấy node Pi thì xử lý
discovery/mạng trước. Không gọi scan/map không xuất hiện là lỗi SLAM ngay.

## Giới hạn kiểm chứng

Đã đọc launch/script/config và dữ liệu thẻ; chưa khởi động Pi, serial, LiDAR,
motor hoặc tạo map thật trong phiên hỗ trợ này. Các thông số mounting/geometry
vẫn cần đối chiếu xe. Không sửa model, firmware, WiFi hoặc map cũ trên thẻ.
