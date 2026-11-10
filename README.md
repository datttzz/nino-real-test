# Nino Real Test — Pi5 + STM32 + PI / SAC

**Hướng dẫn chạy xe thật mới nhất:**
[15 lượt, tự kiểm tra map/AMCL, dùng lại pose đầu, xử lý IMU và tạo báo cáo](docs/CHAY_XE_THAT_15_LUOT.md).

**Tạo map mới bằng tay SBUS:**
[Lệnh theo từng terminal Pi/laptop, lưu map và lấy về máy](docs/TAO_MAP_MOI_SBUS.md).

Repo thử riêng ghép từ **pi5-stm32-slam-main(2).zip** và
**Ninobot_controlled_with_torque-add_rl_2-main (1).zip** đã gửi.
Hai repo gốc không bị sửa. Đây là workspace ROS 2 Jazzy độc lập,
không cần source workspace mô phỏng.

**Có:** LiDAR, encoder, IMU, bridge, EKF, AMCL/Nav2 từ repo Pi;
PI vận tốc bánh, observation 23 phần tử và inference SAC từ hợp đồng mã train.
Có chế độ teleop riêng để thử vòng vận tốc firmware trước PI/SAC, không cần map/model.
**Không có:** Gazebo, train, tuning PID, GT, replay buffer, benchmark.

## Trạng thái thực tế

- Đã kiểm tra các phép tính và logic callback bằng test offline.
- Chưa build/launch ROS trên Pi; môi trường tạo repo không có ROS.
- Bản nguồn ban đầu không chứa checkpoint. Ngày 2026-11-02 đã nhập model
  **80.000 step** vào `models/sac80k`, xác minh hash/metadata và inference CPU
  offline trên laptop. Chưa xác minh inference trên Pi hoặc hiệu quả trên xe.
- Firmware thực tế chưa được xác nhận hỗ trợ `/cmd_torque` và đơn vị N.m tại bánh.
  Tài liệu firmware gửi kèm chủ yếu mô tả `/cmd_vel`; tài liệu không chứng minh
  firmware đang nạp có giao diện mô-men. Có subscriber cũng chưa chứng minh đúng
  đơn vị, dấu hoặc watchdog: phải kiểm chứng riêng.
- PI trong sim cập nhật 1.000 Hz; adapter thật cập nhật **theo mẫu encoder mới**,
  dự kiến firmware khoảng 50 Hz. Repo không giả feedback 1.000 Hz.
  Chưa xác minh chất lượng điều khiển thật hoặc khả năng chuyển policy sang xe thật.

## Luồng điều khiển

Nav2 → velocity smoother → collision monitor → `/rl/cmd_vel_reference`
→ PI bánh + residual SAC → giới hạn → `/cmd_torque` → STM32.

`/cmd_vel` và `/cmd_mit` không được có publisher khi chạy adapter này:
firmware có thể đổi chế độ theo topic đến sau. Node kiểm tra tranh quyền,
nhưng phát hiện qua ROS graph có độ trễ và không thay thế việc tắt các nguồn khác.
Không chạy đồng thời bringup cũ, teleop cũ hoặc systemd tự khởi động repo cũ.

Bridge tiếp tục phát `/odom/unfiltered`, EKF phát `/odom` và TF.
Controller RL không phát odometry/TF. Kích thước thật giữ radius=0,060 m,
track=0,345 m; sim là 0,0625 m và 0,34273666 m. Nav2 Pi dùng RPP;
trainer dùng DWB. Đây là những khác biệt triển khai chưa được đánh giá.

## Driver LiDAR hiện tại: rplidar_ros

Bringup dùng `rplidar_ros/rplidar_composition`, baud 115200, frame `laser` và
scan mode trong robot.yaml. Giữ `/scan_raw` → deskew → `/scan` cho mapping.
Không chạy driver SL hoặc driver LiDAR khác cùng lúc với bringup.

Sau khi nhận code mới, dừng launch cũ rồi chạy trên Pi trong repo này:

```bash
sudo apt-get install -y ros-jazzy-rplidar-ros
source /opt/ros/jazzy/setup.bash
./scripts/fetch_rplidar.sh
./scripts/build.sh --packages-select rplidar_ros pi5_slam_bringup
source scripts/env.sh
ROS_DOMAIN_ID=0 ./scripts/run.sh mapping allow_motor_enable:=false rviz:=false
```

Driver source khóa ở 2.1.0 và áp dụng patch trong `patches/`: lỗi kết nối/info/
health thoát với mã lỗi, không để node không hoạt động tiếp tục spin; driver được
quản lý bằng RAII để không dùng lại con trỏ đã giải phóng. `install.sh` tải và áp
patch trước khi build. Không đổi SDK DTR, baud, timeout, health check hoặc scan mode.
SDK vẫn có thể dừng đầu LiDAR lúc kết nối; chỉ khởi động quét sau khi info/health
hợp lệ. Bản sửa vòng đời không chứng minh đã sửa nguyên nhân thiết bị không phản hồi.

Source/binary sllidar_ros2 đã tải được giữ lại nhưng không được bringup gọi.
Quay lại driver cũ chưa chứng minh đã khắc phục timeout; cần có scan thật để xác nhận.

## 1. Build trên Pi

Giải nén repo này vào một thư mục riêng trên Pi, mở terminal tại thư mục đó:

```bash
source /opt/ros/jazzy/setup.bash
./scripts/build.sh
```

Nếu Pi đã chạy được repo navigation cũ, tận dụng ROS và micro-ROS agent đang cài.
Nếu agent chỉ có trong workspace cũ, source overlay `.agent_ws/install/setup.bash`
của agent đó trước build/run, hoặc build agent trong repo mới bằng
`./scripts/build_agent.sh`. Không khởi chạy hai agent cùng mở một cổng serial.
`scripts/install.sh` là lựa chọn cho máy mới, có apt/build agent;
không cần chạy lại trên máy đã đủ phụ thuộc.

## 1a. Teleop bàn phím trước PI + SAC

Đối chiếu source SLAM gốc, các khác biệt được giữ và giới hạn chẩn đoán timeout:
**[docs/DOI_CHIEU_SLAM_GOC.md](docs/DOI_CHIEU_SLAM_GOC.md)**.

Muốn quét và lưu map ngay trong repo: xem hướng dẫn từng terminal
**[docs/TAO_MAP_TREN_PI.md](docs/TAO_MAP_TREN_PI.md)**.

Hướng dẫn từng terminal: **[docs/TELEOP.md](docs/TELEOP.md)**.

- Terminal 1: `./scripts/run_teleop.sh` mở bridge/agent và teleop guard, motor vẫn chưa được bật tự động.
- Terminal 2: `./scripts/teleop_keyboard.sh` mở bàn phím; chưa arm thì xe không nhận lệnh chuyển động từ guard.
- Terminal 3: xem feedback, enable thủ công qua `/set_motor_enable`, rồi arm qua `/teleop/arm` như hướng dẫn.
- `i` tiến, `,` lùi, `j/l` quay, `k` hoặc Space dừng; `q` hoặc Ctrl+C thoát bàn phím.
- Mặc định giới hạn 0,50 m/s và 0,40 rad/s. Ngừng phím khoảng 0,60 s gửi zero; mất bàn phím, feedback hoặc tranh quyền điều khiển làm disarm/disable.
- Kết thúc bằng `./scripts/stop_teleop.sh`, rồi Ctrl+C hai terminal teleop trước khi mở shadow/PI/SAC.

Teleop dùng `/cmd_vel` tới vòng vận tốc STM32, không chạy host PI/SAC, Nav2 hoặc collision monitor. Chế độ này không yêu cầu map, checkpoint hay xác nhận torque; kết quả teleop không chứng minh giao diện `/cmd_torque` đã đúng. Motor cần được enable/arm thủ công, kiểm tra trên bánh kê trước; tránh chướng ngại bằng người điều khiển.

## 1b. Tạo map bằng tay SBUS, lưu trên Pi và lấy về laptop

Map đã lưu trên Pi nằm trong `/home/d/nino-real-test/maps/`, tức
`~/nino-real-test/maps/` khi đăng nhập tài khoản `d`. Thư mục `maps/` trên
laptop là một thư mục khác, không tự đồng bộ với Pi. `.gitkeep` chỉ giữ thư
mục trong Git, không phải bản đồ. Map đang hiện trong RViz chưa chắc đã được
lưu thành file.

### Bước 1 — Terminal 1 trên Pi: khởi động mapping

Pi không cần màn hình; RViz sẽ mở trên laptop. Dừng các phiên mapping,
navigation hoặc teleop cũ bằng Ctrl+C trong terminal đã mở chúng trước khi
chạy. Mapping này tự mở agent STM32, bridge, EKF, driver LiDAR và SLAM Toolbox;
không mở thêm agent hoặc driver riêng.

Từ laptop, mở một terminal SSH (thay IP nếu Pi đổi địa chỉ):

```bash
ssh d@192.168.0.110
```

Trong phiên SSH trên Pi, chạy:

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

Giữ terminal này chạy. `allow_motor_enable:=false` chỉ từ chối bật motor qua
service bridge trên Pi, không vô hiệu hóa quyền điều khiển SBUS trong STM32.
Không chạy `run_teleop.sh`, `teleop_keyboard.sh`, `teleop_guard`, PI hoặc SAC
trong phiên quét map bằng SBUS.

### Bước 2 — Terminal 2 trên Pi: kiểm tra dữ liệu trước khi lái

Mở phiên SSH thứ hai, nạp môi trường như sau. Giữ phiên này để lưu map ở bước 5:

```bash
ssh d@192.168.0.110
```

```bash
cd ~/nino-real-test
source scripts/env.sh

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"
```

Chạy **từng lệnh** sau; với `topic hz`, xem vài dòng rồi nhấn Ctrl+C mới chạy
lệnh kế tiếp. Ctrl+C ở terminal kiểm tra không dừng mapping trong Terminal 1.

```bash
ros2 topic hz /scan
```

```bash
ros2 topic hz /motor_fb
```

```bash
ros2 topic hz /odom
```

```bash
ros2 topic echo /map --once --field info
```

Scan, feedback bánh và odometry phải có mẫu mới; map cần có width/height lớn
hơn 0. Chưa lái nếu LiDAR timeout hoặc mất feedback/odom. Map có kích thước
không tự chứng minh chất lượng ghép map; tiếp tục đối chiếu trên RViz.

### Bước 3 — Terminal 3 trên laptop: xem map trực tiếp bằng RViz

Mở terminal **trực tiếp trên laptop**, không SSH vào Pi. Hai máy phải cùng mạng.

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

Đặt **Fixed Frame = map**, Map dùng `/map`, LaserScan dùng `/scan` và bật
RobotModel. Không dùng `2D Pose Estimate` trong phiên SLAM mapping này.

### Bước 4 — Lái bằng tay SBUS để quét môi trường

Receiver phải nối đúng với STM32 và đã liên kết với tay. Trước khi cấp nguồn
động cơ, kiểm tra Channel Monitor trên tay: nhãn A/E/F trên vỏ chưa đủ để xác
nhận kênh đã gán đúng. Không cần gọi `/teleop/arm` hoặc `/set_motor_enable`.

**Bảng dưới chỉ áp dụng cho firmware `main` commit `0d325939` đã đối chiếu**;
chưa xác minh binary đang có trên board. Nếu dùng bản khác, đối chiếu firmware
trước khi bật motor; xem [giao diện firmware](docs/FIRMWARE_UPSTREAM.md).

| Kênh | Chức năng trong bản firmware trên |
|---|---|
| CH1 | Tiến/lùi, trung tâm là zero; vị trí cần và chiều thực tế tùy cấu hình tay. |
| CH2 | Quay trái/phải, trung tâm là zero. |
| CH5 / A | Thấp: lái tay; cao: ROS. |
| CH9 / E | Thấp: motor tắt; cao: motor bật. |
| CH10 / F | Thấp khi nhả; nhấn làm dừng khẩn và giữ trạng thái dừng. |

Với đúng firmware/kênh trên: bật tay, đưa cần về giữa, E OFF, nhả F, chọn
A MANUAL rồi cấp nguồn động cơ. Chuyển E OFF → ON và nhích cần rất nhẹ để
kiểm tra chiều. Giới hạn SBUS trong source này là 1,0 m/s và 2,5 rad/s,
không lấy giới hạn từ `teleop.yaml`; không đẩy hết cần khi thử.

Đi chậm, quay từ từ, quét đủ tường/góc và quay lại vùng đã đi. Theo dõi map
mở rộng, scan trùng tương đối với tường và pose xe di chuyển phù hợp. Người
lái trực tiếp tránh chướng ngại; phiên mapping không tự tránh vật cản bằng Nav2.

Dừng thường: đưa cần về giữa rồi E OFF. Dừng khẩn: nhấn F. Sau khi xử lý nguyên
nhân dừng, nhả F, giữ cần giữa rồi E OFF → ON để reset latch. Không dùng tắt
tay làm cách dừng duy nhất vì các phiên bản firmware xử lý mất link khác nhau.

### Bước 5 — Terminal 2 trên Pi: lưu map khi mapping còn chạy

Sau khi quét xong, dừng xe và tắt motor. Giữ terminal mapping chạy để SLAM
tiếp tục cung cấp `/map`. Mở terminal Pi/SSH thứ hai, chạy:

```bash
cd ~/nino-real-test
source scripts/env.sh

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"

MAP_BASE="$HOME/nino-real-test/maps/phong_that_$(date +%Y%m%d_%H%M%S)"
./scripts/save_map.sh "$MAP_BASE"
ls -lh "$MAP_BASE.yaml" "$MAP_BASE.pgm"
```

Dùng cùng ROS domain và môi trường mạng với terminal mapping; ví dụ trên dùng
domain `0`. Khi thấy `Map saved successfully`, đã lưu được map. Script dùng tên
có thời gian và từ chối ghi đè map đã có. Chỉ dừng mapping sau khi lưu thành công.
Nếu chỉ cần lấy map cũ, bỏ qua bước lưu, xem tên file bằng:

```bash
ls -lht ~/nino-real-test/maps/
```

Một map thường gồm hai file cùng tên: `.yaml` chứa resolution, origin và tên
ảnh; `.pgm` chứa ảnh bản đồ. Xem trường `image:` trong YAML nếu ảnh dùng đuôi
khác. Phải lấy cả YAML và ảnh tương ứng, không chỉ lấy một file.

### Bước 6 — Terminal mới trên laptop: tải map từ Pi

Mở terminal trực tiếp trên laptop: prompt phải là `tiendat@anhladat`, không phải
`d@d`. Nếu đang SSH vào Pi thì chạy `exit` trước. Không cần source ROS để dùng
`scp`.

```bash
PI_HOST=192.168.0.110
mkdir -p "$HOME/Downloads/nino_maps"

scp "d@$PI_HOST:/home/d/nino-real-test/maps/*.yaml" "$HOME/Downloads/nino_maps/"
scp "d@$PI_HOST:/home/d/nino-real-test/maps/*.pgm" "$HOME/Downloads/nino_maps/"
```

Đổi `PI_HOST` thành IP hiện tại của Pi nếu IP thay đổi; `.110` là địa chỉ dùng
trong phiên này. Hai lệnh trên lấy tất cả map YAML/PGM trong thư mục, giữ nguyên
tên file. Nhập mật khẩu tài khoản `d` trên Pi khi được hỏi. Nếu SSH lần đầu hỏi
fingerprint, đối chiếu với khóa của Pi trước khi chấp nhận.

Ví dụ chỉ tải map đã lưu ngày 04/11/2026 lúc 12:23:39:

```bash
MAP_NAME=phong_that_20261104_122339
scp "d@$PI_HOST:/home/d/nino-real-test/maps/$MAP_NAME.yaml" "$HOME/Downloads/nino_maps/"
scp "d@$PI_HOST:/home/d/nino-real-test/maps/$MAP_NAME.pgm" "$HOME/Downloads/nino_maps/"
```

Thay `MAP_NAME` bằng tên thực tế từ bước 5, không kèm đuôi file. Giữ ảnh ở đúng
đường dẫn được trường `image:` trong YAML tham chiếu; map saver thường ghi tên
ảnh tương đối để cặp file có thể chuyển sang máy khác.

### Bước 7 — Trên laptop: mở ảnh map đã tải

```bash
xdg-open "$HOME/Downloads/nino_maps"
```

Nhấp đúp file `.pgm` để xem hình bản đồ. Mở `.yaml` sẽ thấy chữ cấu hình.
Muốn xem scan và vị trí xe chồng lên map thì dùng RViz với Fixed Frame `map`
theo [hướng dẫn mapping](docs/TAO_MAP_TREN_PI.md).

Nếu `scp` báo đường dẫn `/home/d/Downloads/`, kiểm tra lại máy đang chạy lệnh:
`~` là thư mục home của máy hiện tại. Nếu chỉ có `.gitkeep`, chưa có map được
lưu tại thư mục đó. Nếu báo `Connection refused` hoặc `No route to host`, kiểm
tra Pi đã bật, SSH hoạt động và IP đúng trước khi tải lại.

## 2. Chạy shadow trước, chưa cần model

```bash
read -e -r -p 'Đường dẫn map thật (.yaml): ' REAL_MAP
./scripts/run_real.sh controller:=shadow map:="$REAL_MAP" rviz:=true
```

Trong RViz đặt **2D Pose Estimate**, sau đó đặt một goal ngắn để có `/plan`
và vận tốc tham chiếu. Shadow không tạo publisher `/cmd_torque` hay `/motor_enable`;
bridge có publisher enable riêng nhưng launch shadow từ chối yêu cầu bật motor.
Khởi động phần cứng ở trạng thái motor disable. Xe sẽ không tự đi để tạo dữ liệu.

Terminal khác tại cùng repo, cùng ROS_DOMAIN_ID:

```bash
source scripts/env.sh
ros2 topic echo /rl/status
# Chạy từng lệnh sau khi thoát echo bằng Ctrl+C:
ros2 topic hz /motor_fb
ros2 topic info /cmd_vel --verbose
ros2 topic info /cmd_torque --verbose
```

`has_checkpoint=false` nghĩa là chỉ kiểm tra dữ liệu/công thức observation,
chưa có inference. `fault` phải rỗng khi path, TF, IMU, odometry và encoder hợp lệ.
Nav2 có thể timeout vì shadow không làm robot đi; đó không phải kết quả đánh giá policy.
Chỉ nhận IMU có orientation hợp lệ, gia tốc gồm trọng lực và lắp cùng hướng body
như cấu hình hiện tại. IMU xoay khác hướng bị chặn, chưa có phép đổi frame tổng quát.
Bridge giữ nguyên timestamp nguồn IMU; mẫu cũ quá `stamp_tolerance_s` (mặc định
0,05 s), tương lai, thiếu stamp, trùng hoặc lùi bị loại. Cần đồng bộ clock
STM32–Pi; bridge không đổi timestamp lỗi thành giờ nhận để giả dữ liệu mới.

## 3. Cài inference và nhập checkpoint SAC thật

Chạy tại repo trên Pi, hoặc import trước trên laptop Ubuntu tương thích rồi chép
**cả thư mục bundle** sang Pi. Chỉ dùng checkpoint do bạn tin cậy:
SB3 load có thể deserialize mã Python.

```bash
python3 -m venv --system-site-packages .venv-inference
.venv-inference/bin/python -m pip install --only-binary=:all: \
  'torch==2.15.0+cpu' --index-url https://download.pytorch.org/whl/cpu
.venv-inference/bin/python -m pip install \
  -r requirements-inference.txt 'torch==2.15.0+cpu'
read -e -r -p 'File model.zip 50k: ' MODEL_50K
read -e -r -p 'config.yaml gốc của chính lần train đó: ' TRAIN_CONFIG
read -e -r -p 'metadata.json gốc của chính lần train đó: ' TRAIN_METADATA
```

Nếu tạo venv báo thiếu `ensurepip`, cài `sudo apt install python3.12-venv`
trên Pi rồi tạo lại venv. `--only-binary=torch` chỉ chọn wheel, không bảo đảm
wheel chạy CPU. Pi5 dùng bản CPU từ index PyTorch ở trên; không cài CUDA.
Nếu đã cài Torch kèm CUDA và import lỗi `nvidia/cu13/...libcublasLt.so.13`,
thay riêng Torch trong venv bằng:

```bash
.venv-inference/bin/python -m pip install --no-cache-dir --only-binary=:all: \
  --force-reinstall --no-deps 'torch==2.15.0+cpu' \
  --index-url https://download.pytorch.org/whl/cpu

.venv-inference/bin/python -c \
"import rclpy, torch, stable_baselines3; print(torch.__version__); print('CUDA build:', torch.version.cuda)"
```

`torch.version.cuda` phải là `None`. Đây là sửa thư viện inference trên Pi,
không chuyển model hoặc thay thư viện CUDA của môi trường train trên laptop.

Trước lệnh dưới, xác nhận lần train dùng đúng mã 23D không VecNormalize/history,
PI Kp=0,30 Ki=0,10 Kd=0, integral clamp=4, torque tổng ±2 N.m,
slew=10 N.m/s, wheel speed/accel=12 rad/s và 12 rad/s².
Metadata hiện tại không lưu tất cả tham số controller: xác nhận này là của người
vận hành, không được suy ra từ trọng số mạng. Nếu khác, dừng để làm adapter đúng.

```bash
./scripts/import_50k.sh --model "$MODEL_50K" --config "$TRAIN_CONFIG" \
  --metadata "$TRAIN_METADATA" --output "$PWD/models/sac50k" \
  --confirm-source-contract
```

Importer kiểm tra model thực sự 50.000 step, observation 23/action 2,
thứ tự metadata, action scale và inference CPU; lưu SHA256 và config kèm theo.
Không tự tải model hoặc train model thay thế. Không ghi đè bundle đã tồn tại.
Nếu wheel PyTorch ARM64 không có cho môi trường Pi, pip dừng thay vì build Torch
từ nguồn. Repo chưa xác minh wheel hoặc phiên bản SB3/PyTorch với model của bạn;
lỗi load/import phải xử lý trước khi bật chế độ chạy thật.

Dừng shadow cũ bằng Ctrl+C, chạy lại shadow có inference:

```bash
./scripts/run_real.sh controller:=shadow map:="$REAL_MAP" \
  checkpoint:="$PWD/models/sac50k" rviz:=true
```

`/rl/status` cần `has_checkpoint=true`; log JSONL trong `runs/` chứa observation,
action, PI torque, accepted residual và mô-men đề xuất. Log không chứa GT.
Nếu không có dữ liệu mới thì không ghi observation giả; lý do nằm ở `/rl/status`.

### Model 80k đã ghép vào repo cục bộ

Bundle `models/sac80k` gồm `model.zip`, `config.yaml`, `metadata.json` và
`contract.json`. Nguồn là run `train_20261102_095411_087973`; importer đọc đúng
80.000 step từ model. `IMPORT_VALIDATION.json` lưu bằng chứng kiểm tra offline.
Giữ nguyên PI Kp=0,30 Ki=0,10 Kd=0, observation 23D và residual ±0,30 N.m.

Khi chép repo sang Pi, phải chép **cả `models/sac80k`**. Thư mục `models/`
được `.gitignore` loại trừ nên chỉ `git clone` sẽ chưa có trọng số.
Cài môi trường inference và build theo các mục trước; trên Pi chạy shadow:

```bash
./scripts/run_real.sh controller:=shadow map:="$REAL_MAP" \
  checkpoint:="$PWD/models/sac80k" rviz:=true
```

Chỉ sau các bước kiểm chứng firmware/PI/enable/arm ở mục 4–5, dùng cùng
lệnh PI+SAC bên dưới và đổi checkpoint thành `models/sac80k`. Việc import
không tự enable/arm, không sửa `hardware.local.json` và không khởi động động cơ.
Watchdog, freshness, kiểm tra tranh quyền đầu ra và mode shadow giữ nguyên.
Khác biệt geometry, feedback rate và Nav2 sim/thật ở đầu tài liệu vẫn còn.

Nếu cần import lại từ nguồn khác, tên script cũ `import_50k.sh` được giữ tương
thích; chọn số step cần kiểm tra bằng `--expected-steps`:

```bash
./scripts/import_50k.sh --model "$MODEL_80K" --config "$TRAIN_CONFIG" \
  --metadata "$TRAIN_METADATA" --expected-steps 80000 \
  --output "$PWD/models/sac80k_new" --confirm-source-contract
```

Mặc định vẫn là 50.000 step cho lệnh cũ. Loader kiểm tra số step thực trong
model khớp manifest, hash cả ba file nguồn, thứ tự observation/action và scale;
không đổi tên model 50k thành 80k và không ghi đè bundle đã có.

## 4. Xác nhận firmware và thử PI trên bánh kê trước

Chép `hardware.example.json` thành `hardware.local.json` và điền **sau khi đã
kiểm tra thực tế**, không chỉ đổi toàn bộ false thành true:

```bash
cp hardware.example.json hardware.local.json
```

Các trường yêu cầu:

- Firmware nhận `std_msgs/Float64MultiArray` `/cmd_torque` đúng `[left,right]`.
- Giá trị là N.m tại bánh, không phải dòng điện, PWM hoặc torque phía motor chưa
  tính tỉ số truyền. Giới hạn đã kiểm chứng phải bao phủ ±2 N.m của adapter.
- Dấu lệnh hai bánh, dấu encoder và thứ tự bánh đã xác nhận; dấu feedback lấy
  từ config bridge, dấu command lấy từ file hardware.
- Watchdog mô-men thực sự đưa về trạng thái an toàn khi mất lệnh ≤0,5 s;
  không được chỉ kiểm tra watchdog cmd_vel rồi coi là watchdog torque.
- Chấp nhận khác biệt vòng PI encoder thật so với vòng 1.000 Hz mô phỏng,
  để thử có kiểm soát; đây không phải xác nhận policy chạy tốt.

Node còn kiểm tra có subscriber `/cmd_torque`. Nếu firmware đang nạp không hỗ trợ,
live mode bị chặn; repo này không sửa hay flash STM32.

Dừng shadow. Chạy PI riêng để xác nhận đường mô-men trước:

```bash
./scripts/run_real.sh controller:=pi map:="$REAL_MAP" \
  hardware_file:="$PWD/hardware.local.json" rviz:=true
```

Motor vẫn disable, controller vẫn disarmed. Sau khi kê bánh và sẵn nút dừng,
terminal khác, cùng domain:

```bash
source scripts/env.sh
ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /rl/arm std_srvs/srv/SetBool '{data: true}'
```

Chỉ tiếp tục khi response `success: true`. Dùng goal ngắn trong RViz sau khi
đặt initial pose; không mở teleop lên `/cmd_vel`. Có thể kiểm tra dấu/tốc độ
trên bánh kê trước, nhưng AMCL/Nav2 không phải phép đo vận hành đáng tin khi
robot bị kê đứng yên. Sau đó disable trước khi đặt robot xuống đất.

## 5. Chạy PI + SAC 50k

Chỉ sau khi đường PI torque đã được thử đúng. Dừng PI bằng script dưới và Ctrl+C;
khởi động lại:

```bash
./scripts/run_real.sh controller:=pi_sac map:="$REAL_MAP" \
  checkpoint:="$PWD/models/sac50k" hardware_file:="$PWD/hardware.local.json" rviz:=true
```

Đặt initial pose. Lặp hai lệnh enable/arm ở trên rồi đặt goal ngắn trên mặt phẳng.
Chỉ thử dây/gờ sau khi kiểm tra chiều chuyển động, stop và inference thật ổn định.
PI+SAC cần observation/path hợp lệ; lỗi IMU/TF/path/action gây disarm và disable,
không âm thầm thay bằng observation 0. PI riêng không phụ thuộc AMCL/path để
tính mô-men, nhưng vẫn kiểm tra feedback, motor status và quyền sở hữu đầu ra.

Ở mọi chế độ, lệnh dừng/velocity timeout xóa PI integral và residual,
đưa mô-men yêu cầu về 0 ngay, bỏ qua slew khi dừng. **Mô-men 0 không phải
phanh cơ khí**; giữ nút dừng khẩn phần cứng. Mất tiến trình dựa vào watchdog
firmware đã kiểm chứng, không dựa vào callback shutdown chắc chắn được chạy.
Nếu MCU báo bit 5 command-timeout trong lúc đã bắt đầu chuyển động, adapter
gửi torque 0, disarm và gửi disable. Status trở lại healthy không tự arm lại.
Timeout khi đang chờ goal đầu tiên được giữ ở standby torque 0.

## Lưu kết quả PI và dữ liệu xe thật SAC80k riêng từng lượt

Sau khi đã xác minh hardware contract và thử PI đúng như mục 4, có thể thay
lệnh `run_real.sh` bằng `run_logged_real.sh` dưới đây. Script không tự bật motor,
arm hoặc đặt goal. Dừng mapping/teleop/launch cũ trước; không chạy hai phiên cùng
lúc. Chạy trên **Pi**, RViz tiếp tục mở riêng trên laptop.

Mỗi terminal Pi dùng cùng môi trường mạng:

```bash
cd ~/nino-real-test
source scripts/env.sh
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"
```

**Một lượt PI**, lưu trong `~/nino-real-test/runs/pi/<thời_gian>_<mã_riêng>/`:

```bash
./scripts/run_logged_real.sh pi \
  "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  "$HOME/nino-real-test/hardware.local.json"
```

**Một lượt PI+SAC80k**, lưu riêng trong
`~/nino-real-test/real_data/sac80k/<thời_gian>_<mã_riêng>/`:

```bash
./scripts/run_logged_real.sh sac80k \
  "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  "$HOME/nino-real-test/hardware.local.json"
```

Đổi đường dẫn map nếu dùng map khác. Trước SAC phải cài inference theo mục 3.
Script cần `ros2 bag record` và storage SQLite3; nếu máy thiếu rosbag, cài trên Pi:

```bash
sudo apt install ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-default-plugins
```

Sau khi launch sẵn sàng, đặt initial pose trên RViz và enable/arm thủ công theo
mục 4, rồi chọn một goal cho mỗi phiên. Muốn lưu lượt tiếp theo riêng, dừng xe
bằng `./scripts/stop_real.sh` ở terminal Pi khác, sau đó Ctrl+C terminal chạy
script. Chờ script đóng rosbag xong rồi chạy lại lệnh cho lượt tiếp theo.
Không tắt nguồn Pi trong lúc ghi bag. Mã thoát launch không phải kết quả nhiệm vụ.

Mỗi thư mục phiên chứa:

| File/thư mục | Nội dung |
|---|---|
| `controller/real_*.jsonl` | Logger hiện có: observation, action, PI, residual được chấp nhận, mô-men lệnh, vận tốc/target bánh và metric khi observation hợp lệ. |
| `rosbag/` | Dữ liệu nguồn có timestamp: encoder, IMU, odometry, AMCL/TF, scan/map/path, goal, tham chiếu vận tốc, status và torque preview. |
| `manifest.json`, `config/` | Hash và bản chụp map, ảnh, hardware, cấu hình source/cài đặt; với SAC có cả bundle model 80k để truy nguyên. |
| `runner.log`, `recorder.log` | Log launch/controller và recorder; giữ cả lỗi hoặc dừng giữa chừng. |
| `outcome.json` | Kết quả chưa gán nhãn; bổ sung outcome Nav2, AMCL pass và lý do dừng theo bằng chứng. |
| `launch_exit_code.txt` | Mã thoát launch, không dùng thay success/failure của nhiệm vụ. |

Recorder không subscribe `/cmd_torque` hoặc `/motor_enable`, tránh bị nhầm là
subscriber firmware trong các kiểm tra hiện có. Mô-men trong JSONL/preview là
**lệnh đề xuất/giới hạn của controller**, không phải mô-men đo thực hoặc xác nhận
STM32 đã nhận/áp dụng; JSONL được lấy theo nhịp actor, không ghi mọi lần publish
mô-men theo encoder. PI có thể tiếp tục điều khiển khi observation AMCL/path
không hợp lệ, lúc đó JSONL có thể thiếu dòng; xem raw bag và status.

**Dữ liệu SAC80k là dữ liệu thu từ xe thật, chưa mặc nhiên sẵn sàng train RL.**
Pose là định vị AMCL, vận tốc thân từ odometry là ước lượng encoder/IMU; không
gọi là vị trí thực tuyệt đối và không suy ra không trượt chỉ từ encoder. Muốn
train tiếp phải chọn các mẫu hợp lệ, đồng bộ thời gian, tạo transition
`(observation, action, reward, next_observation, terminated, truncated)` và gán
nhãn outcome/reset. Script không tự tính reward, không tạo replay buffer, không
train hoặc đổi thuật toán. Giữ riêng dữ liệu thành công, thất bại điều khiển và
lỗi hạ tầng; không xóa lượt xấu để làm đẹp dataset. Không replay rosbag chứa
topic điều khiển khi xe thật đang được bật.

Xem phiên SAC mới nhất trên Pi và dung lượng còn trống:

```bash
ls -td ~/nino-real-test/real_data/sac80k/*/ | head
df -h "$HOME/nino-real-test"
```

Trong thư mục phiên đã đóng, chạy `ros2 bag info /đường/dẫn/phiên/rosbag` để
kiểm tra topic và số message. Có `metadata.yaml` chưa chứng minh mọi topic đã
có dữ liệu. Scan có thể chiếm nhiều dung lượng. `real_data/` và `runs/` được
Git bỏ qua; cần sao lưu riêng, không trông chờ `git push` lưu dataset.

## So sánh một lượt PI và một lượt PI+SAC80k trên xe thật

Muốn **3 lượt PI và 3 lượt SAC80k**, xem
[hướng dẫn sáu phiên và báo cáo Mean ± Std](docs/SO_SANH_PI_SAC_3_LUOT.md).
Script phân tích nhận nhiều thư mục trong `--pi-dir` và `--sac-dir`.

Chạy tuần tự hai lệnh `run_logged_real.sh` ở mục trên, mỗi lần chỉ một controller.
Giữ cùng map, vị trí/hướng xuất phát, goal, tốc độ Nav2 và giới hạn chấp hành.
Dừng, disable và đưa xe về điểm đầu trước lượt tiếp theo. Không sửa gain giữa
hai lượt. Ghi lại hai thư mục phiên mà script in ra. Một lượt mỗi loại chỉ là
đối chiếu ban đầu, chưa đủ kết luận thống kê về model.

Sau khi xe đã dừng và cả hai rosbag đã đóng, trên **laptop** tải dữ liệu về repo
train (đổi IP nếu cần):

```bash
PI_HOST=192.168.0.110
TRAIN_REPO="$HOME/project/Ninobot_controlled_with_torque-add_rl_2"
mkdir -p "$TRAIN_REPO/real_robot_data/pi" "$TRAIN_REPO/real_robot_data/sac80k"
scp -r "d@$PI_HOST:/home/d/nino-real-test/runs/pi/." "$TRAIN_REPO/real_robot_data/pi/"
scp -r "d@$PI_HOST:/home/d/nino-real-test/real_data/sac80k/." "$TRAIN_REPO/real_robot_data/sac80k/"
ls -td "$TRAIN_REPO/real_robot_data/pi/"*/
ls -td "$TRAIN_REPO/real_robot_data/sac80k/"*/
```

Chép cả thư mục phiên, không chỉ JSONL: script phân tích kiểm tra hash snapshot
và số timesteps trong checkpoint zip đã lưu. Dữ liệu trên Pi vẫn được giữ nguyên.
Mỗi phiên có `outcome.json`: chỉ điền outcome/kết quả nếu đã có bằng chứng từ
Nav2/AMCL và lý do dừng; để null nếu chưa biết. Nhãn này là của người vận hành,
script chưa tự xác minh pass nhiệm vụ từ rosbag.

Chọn đúng hai thư mục phiên đã tải, chạy trên **laptop**:

```bash
cd /home/tiendat/Downloads/nino-real-test
read -e -r -p 'Thư mục phiên PI (chứa manifest.json): ' PI_RUN
read -e -r -p 'Thư mục phiên SAC80k (chứa manifest.json): ' SAC_RUN
REPORT_DIR="$TRAIN_REPO/benchmark_results/real_PI_SAC80k_$(date +%Y%m%d_%H%M%S)"

python3 scripts/compare_real_runs.py \
  --pi-dir "$PI_RUN" \
  --sac-dir "$SAC_RUN" \
  --output-dir "$REPORT_DIR"

xdg-open "$REPORT_DIR/report.html"
```

Báo cáo offline gồm bảng `summary.csv` và HTML có đồ thị lỗi ngang theo thời gian:
RMSE/P95/Max ngang AMCL, RMSE hướng, RMSE vận tốc bánh, RMS gia tốc ngang/đứng
đã xử lý trọng lực và RMS/Peak mô-men lệnh ở các snapshot logger. Có số mẫu,
coverage và gap; giữ các lượt bị lỗi, không loại theo outcome. Chỉ dùng mẫu
armed có observation hợp lệ; PI có thể thiếu JSONL khi AMCL/path lỗi dù vòng
PI vẫn chạy, nên cần đọc rosbag/status để đánh giá coverage và toàn bộ lượt.

Không tính thời gian quan sát thành thời gian hoàn thành. Thiếu terminal goal
pose/outcome có timestamp hoặc diagnostics limiter đầy đủ thì ghi N/A, không
suy ra từ `remaining` hoặc mô-men snapshot. Chưa tự xác minh cùng path/start/goal.
Không dùng báo cáo này làm bằng chứng vị trí thực tuyệt đối hoặc dữ liệu train
RL đã hoàn chỉnh. Chưa chạy hai lượt xe thật thì chưa có kết quả so sánh thật.

## Nếu không có `/rl/status`

Topic này do node `/real_controller` phát, không phải Nav2 hoặc bridge. Nav2
báo `Managed nodes are active` chưa chứng minh controller đang sống. Kiểm tra
`ros2 node list` và log có tiền tố `[real_controller-...]`, nhất là phần đầu
launch; đoạn log AMCL/planner cuối cùng có thể bỏ sót lỗi controller đã thoát.

Bản sửa startup kiểm tra hardware JSON trước khi mở bringup PI/PI+SAC. File
thiếu, JSON lỗi hoặc xác nhận chưa đạt sẽ báo `Controller startup blocked` và
không mở Nav2/phần cứng. Script lưu phiên cũng kiểm tra trước khi ghi rosbag.
Không tạo `/rl/status` giả và không tự xác nhận các cờ trong hardware file.
Nếu controller thoát sau khi launch đã mở, bringup sẽ shutdown thay vì tiếp tục
để Nav2 chạy mà không có controller. Cần build lại `nino_real_rl` sau cập nhật:

```bash
cd ~/nino-real-test
source scripts/env.sh
./scripts/build.sh --packages-select nino_real_rl
```

Không bỏ bước xác minh phần cứng để làm topic xuất hiện. AMCL chưa có initial
pose là lỗi độc lập: đặt đúng vị trí/hướng xe bằng `2D Pose Estimate` mỗi phiên;
không gán mặc định `(0,0,0)` hoặc thêm TF tĩnh để ép hết lỗi. Một cảnh báo TF
extrapolation lúc vừa đặt pose chưa chứng minh lỗi kéo dài nếu costmap đã start
và các node đã active sau đó. Cảnh báo timestamp IMU cần đo riêng nếu còn lặp;
không thay timestamp cảm biến bằng giờ nhận hoặc nới freshness để che lỗi.

## Dừng và quay lại cách chạy cũ

```bash
./scripts/stop_real.sh
```

Sau đó Ctrl+C terminal bringup. Muốn quay lại vòng vận tốc firmware, dùng repo Pi
cũ hoặc `./scripts/run.sh navigation map:="$REAL_MAP"` trong repo này, với adapter
RL đã tắt. Hai luồng không được chạy đồng thời. Mặc định launch cũ không bật motor.

## Chỉ kiểm tra offline

```bash
python3 -m unittest discover -s tests -v
```

Xem `VALIDATION.md` cho phạm vi kiểm chứng và `docs/INTEGRATION.md` cho các khác
biệt so với mã train. Thư mục này là repo Git cục bộ; chưa tạo remote GitHub.


## 15 lượt xe thật: Nav-only, PI và SAC80k

Hướng dẫn theo từng terminal, dừng/bê xe giữa mỗi lượt và xuất báo cáo HTML/CSV: [CHAY_15_LUOT_VA_BAO_CAO.md](docs/CHAY_15_LUOT_VA_BAO_CAO.md). Các phiên mới dùng cùng logger thụ động; PI/SAC vẫn cần hardware.local.json được xác minh.
