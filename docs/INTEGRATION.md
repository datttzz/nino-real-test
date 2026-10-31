# Phạm vi tích hợp và hợp đồng điều khiển

## Những gì đã lấy

| Nguồn | Thành phần | Thay đổi |
|---|---|---|
| Pi | stm32_base_bridge | Giữ nguyên code bridge, encoder signs, IMU, odometry, motor enable service |
| Pi | pi5_slam_bringup | Giữ EKF, AMCL/Nav2, LiDAR, deskew, URDF, RViz; thêm tham số `command_output` ở launch |
| Pi | scripts / udev / agent lock | Giữ build, môi trường, serial, mapping; thêm script chạy/tắt/import model |
| RL | features | Chỉ lấy path projection, heading wrap, gravity removal; không lấy reward/training |
| RL | kinematics | Giữ công thức wheel targets và torque slew/speed limiter |
| RL | observation / PI contract | Adapter mới triển khai đúng thứ tự/scales, gains và giới hạn từ mã nguồn đã gửi |

Không nhập nino_description, nino_navigation, effort_drive node gốc, episode manager,
logger GT, tuning, reports hay replay buffer. Demo sensor nhỏ của package Pi gốc
vẫn tồn tại để tránh sửa package nền, nhưng real_test không khởi chạy nó.

## Observation 23D theo đúng thứ tự

| Index | Giá trị trước clip [-10,10] |
|---|---|
| 0 | cross-track / max_path_deviation của config gốc |
| 1 | heading error / pi |
| 2 | remaining path length / 10 |
| 3–4 | roll/0.5, pitch/0.5 |
| 5–7 | gia tốc body đã bỏ trọng lực / 10 |
| 8–10 | gyro x,y,z / 3 |
| 11–12 | odom vx/0.5, odom wz |
| 13–14 | safe reference vx/0.5, wz |
| 15–16 | omega trái/phải /12 |
| 17–18 | PI-commanded torque trái/phải /2 |
| 19–20 | previous normalized SAC action trái/phải |
| 21–22 | accepted residual trước slew / residual_limit |

Input float32, output action trái/phải [-1,1], nhân residual_limit của run gốc,
tổng torque clamp ±2 N.m, sau đó slew 10 N.m/s và chặn torque đẩy quá 12 rad/s.
Scale policy không được âm thầm thay đổi theo motor thật.

## Những khác biệt có chủ đích so với mô phỏng

1. Vòng PI cập nhật một lần trên timestamp encoder mới. Không tích phân I hoặc
   giả đạo hàm dựa trên một mẫu lặp 1.000 lần/giây. Actor giữ chu kỳ của checkpoint.
2. Vẫn Kp=.30, Ki=.10, Kd=0; integral clamp ±4, torque ±2, target accel12.
   Thuật toán I giữ cách clamp của mã train, không thêm tuning/anti-windup mới.
3. Khi zero reference/timeout/fault: reset target/I/residual và yêu cầu torque0
   ngay; không kéo dài torque cũ bằng slew. Với stop bình thường, motor còn enable;
   fault khi đang armed phát thêm disable. Torque0 không bảo đảm dừng cơ khí.
4. Vật lý thật giữ theo robot.yaml Pi, không dùng geometry mô phỏng. Dữ liệu thực
   không được đổi tên thành GT hay kết quả đo độc lập; pose là AMCL/TF.
5. Chỉ hỗ trợ IMU lắp identity rotation như profile hiện tại. Mount khác hướng
   bị chặn thay vì lặng lẽ dùng sai frame.
6. Path là global plan mới của Nav2, không phải fixed reference 11m của bài train.
   Ý nghĩa remaining/projection vẫn giữ, nhưng phân phối input có thể khác.
   Replan không làm policy trở thành bằng chứng cải thiện bám một đường cố định.

Những khác biệt trên cần thử shadow và thử motor có kiểm soát; không thể kết luận
sim-to-real thành công chỉ nhờ test Python. Không tuyên bố PID/RL tốt hơn nhau.

## Quyền điều khiển và lỗi

- Shadow: không tạo publisher tới firmware, không arm được.
- PI/live: cần hardware contract và một người enable qua bridge rồi arm qua
  `/rl/arm`. Startup không enable tự động, recovery không tự rearm.
- Node không subscribe residual từ ngoài; policy nội bộ là nguồn duy nhất.
- Cần đúng một publisher torque (adapter), không publisher cmd_vel/cmd_mit,
  và có subscriber torque. Đây là kiểm tra discovery, không khóa độc quyền DDS.
- Callback joint kiểm tra source stamp raw `/motor_fb`, tên/dấu hai bánh,
  timestamp phải mới và tăng, status CAN fresh, tốc độ mẫu tối thiểu cấu hình.
- Đầu vào IMU/odom phải có stamp mới; observation kiểm tra skew, pose TF tại
  timestamp odometry. Không thay missing feature bằng 0.
- Timer watchdog vẫn kiểm tra mất encoder khi không còn callback joint.
- Actor chạy cùng executor; nếu actor treo, firmware torque watchdog đã xác nhận
  là lớp xử lý mất lệnh cuối cùng. Repo không dựa vào Python shutdown luôn chạy.

Để dừng robot thật, dùng `scripts/stop_real.sh` và nút dừng phần cứng. Dừng service
có timeout; không xác nhận robot đã dừng chỉ vì tiến trình gửi lệnh đã thoát.
