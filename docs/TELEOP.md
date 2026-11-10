# Teleop trước PI + SAC

Luồng: bàn phím → `/teleop/cmd_vel` → teleop guard → `/cmd_vel` → vòng vận tốc STM32.
Bridge không subscribe `/cmd_vel` và không chuyển lệnh vận tốc/torque; nó chỉ
chuyển feedback, IMU và service enable. Không thêm relay `/cmd_vel`, vì publisher
thứ hai sẽ gây tranh quyền và bị guard chặn.
Không cần checkpoint, map hoặc Nav2. Không chạy host PI/SAC hay collision monitor trong chế độ này.

Các lệnh dưới chạy trong ba terminal SSH tương tác (`ssh -t`) trên Pi, repo ở
`~/nino-real-test`. Không chạy bringup cũ/systemd/teleop/Nav2 khác cùng lúc.
Nếu mapping đã mở phần cứng, chỉ mở guard theo TAO_MAP_TREN_PI.md, không chạy
run_teleop.sh trùng với mapping.

Trong mỗi terminal Pi, nạp môi trường trước:

```bash
cd ~/nino-real-test
source scripts/env.sh
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"
```

## Chuẩn bị một lần

Tại repo trên Pi, build sau khi nhận code mới:

```bash
cd ~/nino-real-test
source /opt/ros/jazzy/setup.bash
./scripts/build.sh --packages-select nino_real_rl stm32_base_bridge pi5_slam_bringup
```

Agent phải được cài/build như README chính. Kiểm tra cổng `/dev/stm32_robot`
và quyền serial. Có thể truyền `stm32_port:=/dev/ttyACM0` nếu cần.
Firmware cần đúng giao diện vận tốc Twist trên `/cmd_vel`, đúng dấu/đơn vị và
watchdog vận tốc đưa về trạng thái an toàn khi mất lệnh. Không cần đổi các cờ xác
nhận mô-men trong hardware.local.json để dùng teleop.

Lần đầu kê bánh, giới hạn vùng thử và sẵn nút dừng phần cứng. Kết quả test phần
mềm không xác nhận xe sẽ đi đúng dấu hoặc phanh an toàn.

## Terminal 1 — khởi động phần cứng và guard

```bash
cd ~/nino-real-test
export ROS_DOMAIN_ID=0
./scripts/run_teleop.sh
```

Luồng này không bật motor tự động. Guard bắt đầu DISARMED.
Nếu script báo một bringup khác đang chạy, dừng bringup đó rồi chạy lại.

## Terminal 2 — bàn phím

```bash
cd ~/nino-real-test
export ROS_DOMAIN_ID=0
./scripts/teleop_keyboard.sh
```

Để terminal này có focus. Bàn phím gửi heartbeat zero khi chưa nhấn phím.
Chưa arm thì guard chưa chuyển lệnh chuyển động tới firmware.

| Phím | Hành động |
|---|---|
| `i` | Tiến |
| `,` | Lùi |
| `j` / `l` | Quay trái / phải |
| `u` / `o` | Tiến kết hợp quay |
| `m` / `.` | Lùi kết hợp quay |
| `k` hoặc Space | Gửi zero, vẫn có thể nhấn phím đi tiếp khi còn armed |
| `q` hoặc Ctrl+C | Thoát bàn phím, gửi zero; guard disarm khi mất input/discovery |

Giới hạn mặc định: **0,50 m/s**, **0,40 rad/s**. Giữ/nhấn lặp phím để đi;
không có phím chuyển động mới trong khoảng **0,60 s** thì gửi zero. Do bàn phím
không phát sự kiện nhả phím, đây là timeout hoạt động phím, không phải phát hiện
nhả phím tức thì. Mức 0,60 s vượt key-repeat delay 0,50 s của laptop. Nhấn
`k`/Space để dừng ngay thay vì chờ timeout; ở 0,50 m/s, 0,60 s tương ứng khoảng
0,30 m lệnh tiến trước khi gửi zero, chưa gồm quãng phanh.
Không có phím tăng tốc tùy ý; guard vẫn clamp lệnh theo `config/teleop.yaml`.
Script bàn phím và launch guard cùng đọc teleop.yaml: block `teleop_guard` cho
guard, block `nino_teleop_keyboard` cho bàn phím. Khi đổi tốc độ, đổi cả hai block.
Heartbeat vẫn là 0,10 s; `command_timeout: 0.30` giám sát mất chương trình bàn phím,
khác với `key_hold_timeout: 0.60` giám sát ngừng bấm phím.

## Terminal 3 — kiểm tra feedback rồi enable/arm thủ công

```bash
cd ~/nino-real-test
export ROS_DOMAIN_ID=0
source scripts/env.sh
ros2 topic echo /motor_status --once
ros2 topic hz /motor_fb
```

`hz` chạy liên tục: nhấn Ctrl+C để thoát lệnh đó trước khi làm tiếp.
Guard yêu cầu encoder có tên hai bánh, vận tốc hữu hạn và timestamp nguồn mới,
CAN hai bánh có feedback, không bus-off. Sau khi đã kiểm tra bánh kê và sẵn dừng:

```bash
ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /teleop/arm std_srvs/srv/SetBool '{data: true}'
```

Giữ zero input ở Terminal 2 trong lúc arm. Chỉ tiếp tục khi cả hai trả
`success: true`; guard còn kiểm tra bit motor enabled thực tế trong status.
Nếu arm thất bại, đọc `message` và kiểm tra dữ liệu, không bỏ các điều kiện bảo vệ.
Sau arm, guard gửi zero và chờ một callback `/motor_status` mới báo hết bit
command-timeout trong tối đa 0,60 s. Chỉ thử phím khi `/teleop/status` có
`"armed": true` và `"ready": true`. `armed` một mình chưa xác nhận đã sẵn sàng.
Nếu không nhận được trạng thái sẵn sàng, guard ngắt có lý do; không bỏ handshake
để ép xe chạy. Đây là trạng thái firmware báo, không phải đo bánh đã quay.
Sau đó quay lại Terminal 2, thử từng phím ngắn và quan sát chiều bánh.

Có thể theo dõi bằng một terminal khác:

```bash
cd ~/nino-real-test
export ROS_DOMAIN_ID=0
source scripts/env.sh
ros2 topic echo /teleop/status --full-length
```

## Dừng và chuyển sang PI + SAC

1. Nhấn `k` hoặc Space ở Terminal 2 để yêu cầu vận tốc zero.
2. Terminal 3 chạy:

   ```bash
   ./scripts/stop_teleop.sh
   ```

3. Nhấn Ctrl+C ở Terminal 2 và Terminal 1, đợi bringup thoát.
4. Sau khi motor đã disable, làm các bước shadow, xác nhận torque, PI và PI+SAC
   trong README chính. `run_real.sh` dùng cùng lock nên không chạy cùng teleop.

Mất input trên 0,30 s, feedback bánh stale trên 0,25 s, trạng thái motor mất
trên 0,60 s (cùng ngưỡng bridge), MCU báo timeout khi đã di chuyển hoặc
xung đột publisher sẽ yêu cầu zero, disarm và disable. Dữ liệu trở lại không tự
arm; cần kiểm tra nguyên nhân rồi dùng lại hai service enable/arm.
Mô-men/vận tốc zero là yêu cầu phần mềm, không thay nút dừng cơ khí và watchdog firmware.

## Cách khoanh vùng khi vẫn không quay

- `Teleop RX` có vận tốc khác 0: phím đã tới guard.
- `Nonzero teleop blocked/held at zero`: đọc lý do disarmed, feedback, graph hoặc
  đang chờ firmware velocity-ready; không tự động enable motor để bỏ chặn.
- `Teleop TX /cmd_vel` có vận tốc khác 0: guard đã publish Twist. Không đồng nghĩa
  MCU đã xử lý callback hoặc CAN đã điều khiển motor thành công.
- `Wheel feedback/odometry blocked` từ bridge: lỗi feedback chỉ làm thiếu
  `/joint_states` hoặc `/odom/unfiltered`; bridge không đặt `/cmd_vel` về zero.

Dùng terminal thứ tư cùng môi trường để xem:

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /teleop/status --once --full-length
ros2 topic echo /cmd_vel --field linear.x
```

`/cmd_vel` cần đúng một publisher `teleop_guard`, subscriber firmware với kiểu
`geometry_msgs/msg/Twist` và QoS tương thích (guard reliable/volatile/depth 1).
Guard còn đối chiếu node subscriber `/cmd_vel` với node publisher `/motor_status`;
subscriber của `ros2 topic echo` không được coi là firmware.
`/set_motor_enable` chỉ publish `std_msgs/Bool` lên `/motor_enable`, không tự gửi
CAN từ Pi; firmware phải thực hiện phần đó. Service từ chối enable nếu launch
không cho phép hoặc chưa thấy subscriber. Response thành công chỉ xác nhận
publish; guard còn yêu cầu bit enable trong status, feedback hai bánh mới và
không bus-off. `/teleop/arm` không gọi enable thay cho người dùng.

Không có deadband, ramp hoặc đổi m/s sang torque trong teleop/bridge. Các helper
`nino_real_rl/kinematics.py` chỉ dùng bởi PI/SAC, không nằm trong teleop. Với bán
kính 0,060 m, lệnh thẳng 0,50 m/s tương đương mục tiêu lý thuyết 8,33 rad/s mỗi
bánh nếu firmware dùng đúng geometry. Không suy ra firmware đang nạp đúng chỉ
bằng việc có topic; phải đối chiếu callback, mode, giới hạn và CAN trong firmware.

## Phạm vi kiểm chứng bản sửa

Đã chạy 34 test liên quan guard, bàn phím, feedback gate và service enable. Luồng
ROS thật đã thử trên domain 166 với firmware giả lập, không serial/CAN: service
enable riêng, zero-handshake, chuyển 0,50 m/s nguyên đơn vị tới subscriber và
zero/disable khi mất bàn phím. Chưa xác minh bánh xe thật đã quay. Những lỗi
phần cứng/firmware sau subscriber `/cmd_vel` chưa được chứng minh đã sửa.

### Lỗi Agent từ workspace cũ đã xác minh trên Pi

Trong lần kiểm tra, run_teleop.sh tìm được Agent ở workspace pi5-stm32-slam nhưng
Agent thoát với lỗi thiếu libmicroxrcedds_agent.so.2.4. env.sh hiện bổ sung thư
mục xrceagent/lib cạnh prefix Agent đã resolve. run_teleop.sh kiểm tra thư viện
trước khi launch. Điều này không tự bật motor hoặc flash firmware.

### Lượt thử phần cứng ngày 04/11/2026

Đã triển khai lên Pi và build cả 3 package, chạy 34 test đạt. Agent khởi động
được sau sửa loader. Endpoint cmd_vel đúng stm32h7_node, Twist, QoS tương thích.
Thử có giới hạn 0,5 m, ở 0,10 m/s: đã dừng sớm sau 1,50 s vì không có đáp ứng
bánh, không chạy đủ quãng đường. Có 30 mẫu output khác zero, guard ready, status
19 trong lượt thử; cuối lượt status 35, guard disarmed. Encoder tích lũy khoảng
0,00123 m, chủ yếu có thể là nhiễu, không gọi là đo quãng đường xe thật.

Log trên Pi: `~/nino-real-test/runs/teleop_repair_20261104_114631/`, gồm
probe_result.json, probe_trace.jsonl và bringup.log. Root cause phần cứng/firmware
sau subscriber chưa được xác định hoàn toàn; xem mục đối chiếu SBUS trong
FIRMWARE_UPSTREAM.md. Source host không bypass failsafe firmware.
