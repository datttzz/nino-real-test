# Teleop trước PI + SAC

Luồng: bàn phím → `/teleop/cmd_vel` → teleop guard → `/cmd_vel` → vòng vận tốc STM32.
Không cần checkpoint, map hoặc Nav2. Không chạy host PI/SAC hay collision monitor trong chế độ này.

Các lệnh dưới dùng đường dẫn repo trên laptop. Khi chép lên Pi, thay lệnh `cd`
bằng đường dẫn repo trên Pi. Bringup, bàn phím và service nên chạy ở ba terminal
trên Pi; bàn phím có thể chạy qua SSH tương tác (`ssh -t`). Cùng ROS_DOMAIN_ID với
firmware/agent; ví dụ dưới dùng 0. Không chạy bringup cũ/systemd/teleop/Nav2 khác cùng lúc.

## Chuẩn bị một lần

Tại repo trên Pi, build sau khi nhận code mới:

```bash
cd /home/tiendat/Downloads/nino-real-test
source /opt/ros/jazzy/setup.bash
./scripts/build.sh
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
cd /home/tiendat/Downloads/nino-real-test
export ROS_DOMAIN_ID=0
./scripts/run_teleop.sh
```

Luồng này không bật motor tự động. Guard bắt đầu DISARMED.
Nếu script báo một bringup khác đang chạy, dừng bringup đó rồi chạy lại.

## Terminal 2 — bàn phím

```bash
cd /home/tiendat/Downloads/nino-real-test
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

Giới hạn mặc định: **0,10 m/s**, **0,40 rad/s**. Giữ/nhấn lặp phím để đi;
không có phím chuyển động mới trong khoảng **0,25 s** thì gửi zero. Do bàn phím
không phát sự kiện nhả phím, đây là timeout hoạt động phím, không phải phát hiện
nhả phím tức thì. OS key-repeat có thể làm xe ngắt một nhịp khi mới giữ phím.
Không có phím tăng tốc tùy ý; guard vẫn clamp lệnh theo `config/teleop.yaml`.

## Terminal 3 — kiểm tra feedback rồi enable/arm thủ công

```bash
cd /home/tiendat/Downloads/nino-real-test
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
Sau đó quay lại Terminal 2, thử từng phím ngắn và quan sát chiều bánh.

Có thể theo dõi bằng một terminal khác:

```bash
cd /home/tiendat/Downloads/nino-real-test
export ROS_DOMAIN_ID=0
source scripts/env.sh
ros2 topic echo /teleop/status
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

Mất input trên 0,30 s, feedback stale, MCU báo timeout khi đã di chuyển hoặc
xung đột publisher sẽ yêu cầu zero, disarm và disable. Dữ liệu trở lại không tự
arm; cần kiểm tra nguyên nhân rồi dùng lại hai service enable/arm.
Mô-men/vận tốc zero là yêu cầu phần mềm, không thay nút dừng cơ khí và watchdog firmware.
