# Nino Real Test — Pi5 + STM32 + PI / SAC 50k

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
- File người dùng gửi **không chứa checkpoint 50k**. Model không được tạo giả.
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

Hướng dẫn từng terminal: **[docs/TELEOP.md](docs/TELEOP.md)**.

- Terminal 1: `./scripts/run_teleop.sh` mở bridge/agent và teleop guard, motor vẫn chưa được bật tự động.
- Terminal 2: `./scripts/teleop_keyboard.sh` mở bàn phím; chưa arm thì xe không nhận lệnh chuyển động từ guard.
- Terminal 3: xem feedback, enable thủ công qua `/set_motor_enable`, rồi arm qua `/teleop/arm` như hướng dẫn.
- `i` tiến, `,` lùi, `j/l` quay, `k` hoặc Space dừng; `q` hoặc Ctrl+C thoát bàn phím.
- Mặc định giới hạn 0,10 m/s và 0,40 rad/s. Ngừng phím khoảng 0,25 s gửi zero; mất bàn phím, feedback hoặc tranh quyền điều khiển làm disarm/disable.
- Kết thúc bằng `./scripts/stop_teleop.sh`, rồi Ctrl+C hai terminal teleop trước khi mở shadow/PI/SAC.

Teleop dùng `/cmd_vel` tới vòng vận tốc STM32, không chạy host PI/SAC, Nav2 hoặc collision monitor. Chế độ này không yêu cầu map, checkpoint hay xác nhận torque; kết quả teleop không chứng minh giao diện `/cmd_torque` đã đúng. Motor cần được enable/arm thủ công, kiểm tra trên bánh kê trước; tránh chướng ngại bằng người điều khiển.

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

## 3. Cài inference và nhập checkpoint 50k thật

Chạy tại repo trên Pi, hoặc import trước trên laptop Ubuntu tương thích rồi chép
**cả thư mục bundle** sang Pi. Chỉ dùng checkpoint do bạn tin cậy:
SB3 load có thể deserialize mã Python.

```bash
python3 -m venv --system-site-packages .venv-inference
.venv-inference/bin/python -m pip install --only-binary=torch -r requirements-inference.txt
read -e -r -p 'File model.zip 50k: ' MODEL_50K
read -e -r -p 'config.yaml gốc của chính lần train đó: ' TRAIN_CONFIG
read -e -r -p 'metadata.json gốc của chính lần train đó: ' TRAIN_METADATA
```

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
