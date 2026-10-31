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
