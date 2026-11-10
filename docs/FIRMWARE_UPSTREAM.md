# Giao diện firmware STM32

Pi mới dùng lại firmware trên board hiện tại; không cần nạp STM32 chỉ vì đổi Pi.
Project này không tự nạp hoặc thay đổi bộ lọc IMU trên STM32.

Mốc kiểm tra ban đầu ngày 17/10/2026: STM32H723, BMI088, Mahony, USB CDC,
calibration gyro khi đứng yên sau startup; động cơ mặc định DISABLED.
SHA-256 của file BIN đã nạp:

`ea3801f646af794bd7c55905464a02efbe86f93808c14096d45cb4df5fadf336`

Source firmware tương ứng với bản BIN trên đã được cập nhật lên repo:
https://github.com/nguyenbinh-shark/Microros_H723_USB (commit `f348ec2`).

| Topic | Kiểu | Ghi chú |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | Best effort; gyro rad/s, accel m/s² gồm trọng lực, quaternion trong `imu_link` |
| `/imu/debug` | `std_msgs/Float64MultiArray` | Best effort; mảng 22 phần tử của firmware đã sửa |
| `/motor_fb` | `sensor_msgs/JointState` | Best effort; hai bánh, rad và rad/s; chiều dương cùng hướng xe tiến |
| `/motor_status` | `std_msgs/UInt8` | Reliable; bit 0/1 feedback trái/phải mới, bit 2/3 CAN bus-off, bit 4 enable, bit 5 command timeout |
| `/cmd_vel` | `geometry_msgs/Twist` | vx m/s, wz rad/s; navigation via collision monitor; dry-run output: `/cmd_vel_dry` |
| `/motor_enable` | `std_msgs/Bool` | Reliable; firmware nhận bật/tắt motor |

`/motor_fb` vẫn có thể gửi giá trị 0 khi không có phản hồi CAN. Bridge luôn
kiểm tra thêm `/motor_status` trước khi tạo `/odom/unfiltered` và joint states.
Trạng thái `32` là motor disabled, timeout lệnh, chưa có feedback bánh.

Giữ board cố định tối thiểu 4 giây sau startup. Gyro phải dùng rad/s và timestamp
ROS epoch đồng bộ với agent; không dùng thời gian uptime làm ROS timestamp.
Firmware cần watchdog dừng lệnh sau 500 ms. Geometry firmware cần khớp `robot.yaml`.

Chỉ số `/imu/debug` dùng để kiểm tra calibration:
0 init status; 1 read valid; 2 sample valid; 3 calibrated; 4 số mẫu calibration;
5 sequence; 6 tuổi mẫu ms; 7–9 accel XYZ; 10–12 gyro XYZ trước residual bias;
13 accel scale; 14 g norm; 15 dt; 16 SPI error; 17 time synced;
18 SYSCFG PMCR; 19 SPI2 SR; 20 accel chip ID (30); 21 gyro chip ID (15).

## Phân biệt điều khiển và bridge

Teleop gửi `Twist` trực tiếp tới `/cmd_vel` của firmware. `stm32_base_bridge`
không subscribe topic này và không chuyển vx/wz sang rad/s hoặc mô-men. Service
`/set_motor_enable` publish Bool lên `/motor_enable`; firmware mới thực hiện CAN
enable. Bit 4 của status là trạng thái firmware báo, không chứng minh motor đã
quay. Arm chỉ cho phép guard phát lệnh, không tự bật nguồn/enable motor.

Sau arm, guard mới gửi zero trước và chờ trạng thái mới hết bit 5 rồi mới phát
lệnh khác zero. Điều này tránh dùng lại bit timeout ở standby để ngắt ngay sau
lệnh tiến đầu tiên. Firmware cần cập nhật timestamp nhận `/cmd_vel` với cả lệnh
zero, không chỉ khi có chuyển động. Nếu bit 5 không xóa dù có subscriber và
zero được publish, phải kiểm tra callback/transport firmware; không bỏ watchdog.

Source/commit trong tài liệu mô tả một bản firmware lịch sử. Không đủ bằng chứng
xác nhận binary hiện đang nạp trùng bản đó hoặc giao diện CAN, gain, deadband và
mode điều khiển hiện tại đã đúng. Bản sửa host không flash firmware.

## Đối chiếu source SBUS ngày 04/11/2026

Đã đọc hai commit của repo firmware, không nạp lại STM32:

- `0d32593988cc817311d3dfdd8683df1f1b444a2c`: không có link SBUS trả về
  `RC_MODE_EMERGENCY_STOP`; Motor_task đặt `effective_enable=0` và gửi stop.
  `/motor_status` lại lấy bit enable từ `g_system_enabled`, không phải biến
  `effective_enable`, nên có thể báo enabled dù RC đã chặn chuyển động.
- `f348ec28494aea214242a862f3f1b58a7765b54f` được tài liệu cũ nhắc tới: không có
  SBUS thì trả AUTO/ROS2; khi có SBUS vẫn có thể bị manual/E-stop/RC disarm chặn.

Cả hai source dùng vx m/s, wz rad/s, radius 0,060 m, track 0,345 m, command
watchdog 500 ms. Zero cmd_vel cũng cập nhật thời gian lệnh. Host reliable Twist
có thể giao tiếp với subscriber best-effort Twist của firmware. Không suy ra
MCU đang chạy commit nào chỉ từ tên topic hoặc giá trị motor_status.

Trong lượt thử thật sau bản sửa, guard ready, đã publish 30 mẫu 0,10 m/s trong
1,50 s; status trong giai đoạn chuyển động luôn 19. Encoder chỉ biến thiên cỡ
milimét, không chứng minh xe đã đi được. Đã dừng sớm do không có đáp ứng bánh và
MCU báo disabled sau cleanup. Chưa đạt mục tiêu làm bánh quay. Cần xác nhận binary
hoặc source nạp thực tế, RC policy và diagnostics CAN trước khi sửa/nạp firmware;
không tự bỏ E-stop hoặc ép torque để vượt chặn.
