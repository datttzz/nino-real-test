# Tạo map mới bằng tay SBUS — Pi không có màn hình

Mỗi lần muốn thu map mới, làm lần lượt hướng dẫn này. Pi chạy SLAM và kết nối
STM32/LiDAR; laptop chỉ mở RViz. Không dùng PI, SAC hay Nav2 trong phiên mapping.
Map cũ được giữ nguyên; map mới có tên chứa ngày giờ.

## 0. Chuẩn bị

- Lắp thẻ vào Pi, nối USB STM32 và LiDAR, bật Pi. Laptop và Pi cùng mạng.
- Ví dụ IP bên dưới là `192.168.0.102`; thay bằng IP hiện tại nếu đã đổi.
- Dừng phiên Nav/PI/SAC/teleop cũ bằng Ctrl+C ở terminal chạy phiên đó.
  Trước khi dừng, đưa xe đứng yên và E OFF. Không chạy hai bringup cùng lúc.
- Bật tay SBUS, cần ở giữa, E OFF, F nhả. Giữ xe đứng yên khi IMU khởi tạo;
  chỉ lái khi đã có `/imu/data`, encoder, odometry và scan mới.
- Nếu repo trên Pi đã build và chạy được trước đây thì không cần build lại
  chỉ để tạo map. Không chép `install/`, `build/` hoặc venv laptop sang Pi.

## 1. Terminal 1 — SSH vào Pi và chạy mapping

Trên laptop:

```bash
ssh d@192.168.0.102
```

Sau khi vào Pi (prompt `d@d`):

```bash
cd ~/nino-real-test
source scripts/env.sh

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"

./scripts/run.sh mapping allow_motor_enable:=false rviz:=false
```

**Giữ terminal này chạy tới khi lưu map xong.** Lệnh tự mở micro-ROS agent,
bridge STM32, EKF, LiDAR, deskew (nếu được bật trong cấu hình) và SLAM Toolbox.
Không mở agent/driver LiDAR riêng, không chạy `run_teleop.sh` hoặc node PI/SAC.

`allow_motor_enable:=false` chặn bật motor qua service bridge trên Pi;
không chặn điều khiển SBUS trong firmware. Khi mapping bằng tay không gọi
`/set_motor_enable`, `/teleop/arm` hoặc `/rl/arm`.

## 2. Terminal 2 — SSH vào Pi, kiểm tra dữ liệu

Mở terminal mới trên laptop:

```bash
ssh d@192.168.0.102
```

Trong Pi:

```bash
cd ~/nino-real-test
source scripts/env.sh

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"
```

Chạy lần lượt; mỗi lệnh có giới hạn thời gian, không cần tự Ctrl+C:

```bash
timeout 8s ros2 topic hz /scan
timeout 8s ros2 topic hz /motor_fb
timeout 8s ros2 topic hz /imu/data
timeout 8s ros2 topic hz /odom
timeout 8s ros2 topic echo /map --once --field info
```

Scan thường khoảng vài Hz, encoder/IMU khoảng 50 Hz; xem nhịp thực tế thay vì
coi các giá trị này là cam kết. Map phải có `width`, `height` lớn hơn zero.
Chưa lái nếu thiếu scan, IMU hoặc odometry. Giữ Terminal 2 để lưu map ở bước 5.

## 3. Terminal 3 — Trên laptop, mở RViz

Terminal này **không SSH**, prompt phải là tài khoản laptop:

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

Trong RViz đặt:

| Mục | Giá trị |
|---|---|
| Global Options → Fixed Frame | `map` |
| Map → Topic | `/map` |
| Map → Durability Policy | `Transient Local` nếu có lựa chọn |
| LaserScan → Topic | `/scan` |
| LaserScan → Reliability Policy | `Best Effort` nếu thiếu scan |
| RobotModel | Bật |

**Không bấm 2D Pose Estimate hoặc Nav2 Goal trong phiên tạo map.** SLAM tự tạo
map và xác định pose trong phiên. Phóng to bằng con lăn để xem tường và laser.

## 4. Lái bằng SBUS để quét môi trường

Áp dụng cho firmware SBUS đang dùng trên xe; tên công tắc phải khớp Channel
Monitor của tay. Không đoán chiều cần nếu đã đổi cấu hình tay.

| Kênh/công tắc | Chức năng |
|---|---|
| CH5 / A | Thấp: MANUAL; cao: AUTO/ROS. Mapping bằng tay chọn **MANUAL**. |
| CH9 / E | Thấp: motor tắt; cao: motor bật. |
| CH10 / F | Nhấn: dừng khẩn; nhả trước khi enable. |
| CH1 | Tiến/lùi; vị trí cần và chiều tùy cấu hình tay. |
| CH2 | Quay trái/phải; vị trí cần và chiều tùy cấu hình tay. |

1. Cần giữa, E OFF, F nhả; chọn A MANUAL và kiểm tra màn hình STM32.
2. Khi vùng chạy trống, cấp nguồn motor nếu chưa cấp rồi E OFF → ON.
3. Nhích cần rất nhẹ để kiểm tra chiều, sau đó đi chậm và quay từ từ.
4. Đi qua các góc/tường; quay lại vùng đã đi để SLAM nhận lại khu vực cũ.
5. Theo dõi scan trùng tương đối với tường, map không bị nhân đôi hoặc méo lớn.

Giới hạn vận tốc SBUS không lấy từ `teleop.yaml`; không đẩy hết cần.
Mapping không tự tránh vật cản bằng Nav2. Nếu `RC:LOST`, giải quyết liên kết
tay/bộ thu trước; trạng thái này khác lỗi `NO AGENT` của micro-ROS.

Dừng thường: cần giữa rồi E OFF. Dừng khẩn: F. Sau khi xử lý nguyên nhân,
nhả F, giữ cần giữa và E OFF → ON để bỏ latch dừng theo firmware hiện hành.

## 5. Terminal 2 trên Pi — Lưu map mới

Sau khi quét xong, dừng xe và **E OFF**. Vẫn giữ mapping ở Terminal 1 chạy:

```bash
MAP_BASE="$HOME/nino-real-test/maps/phong_moi_$(date +%Y%m%d_%H%M%S)"

./scripts/save_map.sh "$MAP_BASE"

ls -lh "$MAP_BASE.yaml" "$MAP_BASE.pgm"
cat "$MAP_BASE.yaml"
echo "Map mới trên Pi: $MAP_BASE.yaml"
```

Phải thấy **`Map saved successfully`** và hai file tồn tại. Script từ chối
ghi đè tên cũ. YAML lưu thông số và đường dẫn ảnh; PGM là ảnh occupancy.
Nếu YAML ghi định dạng ảnh khác, giữ đúng file ở trường `image:`.

Lưu YAML/ảnh không phải lưu posegraph để tiếp tục mapping từ phiên cũ.
Sau khi lưu thành công mới Ctrl+C ở Terminal 1. Giữ E OFF khi kết thúc.

## 6. Trên laptop — Lấy map về và xem ảnh

Mở terminal laptop mới; không chạy các lệnh này trong phiên SSH:

```bash
PI_HOST=192.168.0.102
DEST="$HOME/Downloads/map_pi_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$DEST"

scp "d@$PI_HOST:~/nino-real-test/maps/phong_moi_*" "$DEST/"

xdg-open "$DEST"
```

Lệnh lấy tất cả map có tiền tố `phong_moi_` vào thư mục mới. Nếu chỉ lấy một
map, thay dấu `*` bằng đúng tên ngày giờ đã in ở bước 5; lấy cả YAML và ảnh.
Mở PGM bằng ứng dụng xem ảnh. Thư mục laptop và thư mục Pi không tự đồng bộ.

## 7. Khi dùng map mới để chạy Nav/PI/SAC

- Dừng mapping trước; không để SLAM và AMCL cùng phát `map → odom`.
- Thay đường dẫn map cũ trong lệnh chạy bằng **YAML mới trên Pi**, ví dụ
  `/home/d/nino-real-test/maps/phong_moi_YYYYMMDD_HHMMSS.yaml`.
- Tạo bộ chạy mới theo [hướng dẫn chạy xe thật](CHAY_XE_THAT_15_LUOT.md).
  Không dùng lại `initial_pose.json` của map cũ; công cụ kiểm tra hash map.
- Trong phiên navigation mới, dùng 2D Pose Estimate đặt vị trí/hướng xe và
  kiểm tra laser khớp tường, rồi lưu pose đầu theo hướng dẫn bộ chạy.
- Việc tạo map không thay đổi `hardware.local.json`, gain hoặc model SAC.

## 8. Xử lý nhanh nếu không có dữ liệu

### Báo “A project launch is already running”

Dừng đúng terminal bringup cũ bằng Ctrl+C. Không đổi ROS_DOMAIN_ID để chạy
hai phiên cùng chiếm USB. Kiểm tra tiến trình giữ khóa trên Pi:

```bash
fuser -v "${XDG_RUNTIME_DIR:-/tmp}/pi5-stm32-slam-${UID}-0.lock"
pgrep -af 'ros2 launch|micro_ros_agent|rplidar_composition|slam_toolbox'
```

### Không có IMU hoặc motor feedback

```bash
ls -l /dev/stm32_robot
ls -l /dev/serial/by-id/
ros2 topic list
timeout 8s ros2 topic echo /imu/data --once
timeout 8s ros2 topic echo /motor_status --once
```

Giữ E OFF và xe đứng yên cho IMU hiệu chuẩn. Đọc log agent Terminal 1.
Nếu cần nối lại USB STM32, dừng mapping trước rồi khởi động lại Terminal 1;
không mở agent thứ hai cùng cổng. Có `/motor_fb` không có nghĩa IMU đã sẵn sàng.

### LiDAR timeout hoặc mất scan

```bash
ls -l /dev/rplidar
sudo fuser -v /dev/rplidar
sudo dmesg -T | tail -n 30
```

Kiểm tra dây adapter–LiDAR và nguồn, nhất là lỗi sụt áp/USB disconnect đã gặp.
Không chạy driver riêng khi mapping đang giữ cổng. A1M8 hiện dùng 115200;
không đổi baudrate tùy ý để che lỗi dây/nguồn.

### Pi có map nhưng laptop RViz không thấy

Kiểm tra hai máy cùng mạng, cùng domain 0, cùng RMW và profile DDS của repo.
Kiểm tra `/map` ngay trên Pi ở Terminal 2 trước. Nếu Pi cũng chưa có `/map`,
xử lý scan/odom/SLAM; nếu Pi có nhưng laptop không có thì kiểm tra kết nối DDS.
Không thêm TF tĩnh `map → odom` để che lỗi SLAM.

### Lưu map báo timeout

Giữ Terminal 1 hoạt động và kiểm tra lại:

```bash
timeout 8s ros2 topic echo /map --once --field info
```

Chỉ lưu lại khi đã có map. Nếu đặt lại `MAP_BASE`, dùng tên ngày giờ mới để
tránh đụng file còn lại của lần lưu trước. Không xóa map cũ để xử lý timeout.
