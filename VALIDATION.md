# Kiểm chứng bản tích hợp

- **21 test offline đạt**: observation đúng thứ tự/scale, trọng lực IMU, heading wrap, PI gains, torque/slew/speed limit, stop, timestamp, hardware contract, shadow không actuation, tranh quyền topic, thiếu subscriber firmware, watchdog encoder/command, arm trước goal và lỗi observation khi đang chạy.
- Parse cú pháp toàn bộ Python, XML 3 package và bash scripts: đạt.
- Test callback sử dụng message/API stand-in để kiểm tra quyết định controller; không phải chạy ROS, DDS, micro-ROS hay firmware.
- **Chưa build/launch ROS, chưa load model thật, chưa chạy Pi, chưa bật động cơ.** Môi trường tạo gói không có ROS, Torch/SB3 hoặc checkpoint.
- Firmware thực tế, dấu/đơn vị mô-men và watchdog chưa xác nhận; hardware.example.json mặc định chặn live mode.
- Tần số feedback/controller thật khác mô phỏng; chưa có kết quả sim-to-real hoặc kết quả cải thiện tracking/vibration.
- Hai repo gốc không bị sửa. Không thay checkpoint hoặc nới ngưỡng.

Các lệnh và điều kiện còn cần trên Pi nằm trong README.md. VALIDATION.json là bản máy đọc được của phạm vi này.

## Bổ sung ngày 31/10/2026: hai lỗi timestamp/watchdog

- Bridge giữ nguyên timestamp nguồn IMU; loại timestamp rỗng, quá cũ, tương lai, trùng hoặc lùi, không đổi thành giờ nhận. Mẫu bị loại không làm mới mốc hợp lệ.
- Bit 5 command-timeout của MCU khi đang armed và đã bắt đầu chuyển động làm gửi torque 0, reset PI/residual, disarm và gửi disable. Status healthy không tự rearm. Timeout trong lúc chờ goal đầu tiên được giữ ở standby torque 0.
- Chạy đúng test liên quan: 13 callback/state-machine test + 12 bridge/gate test = **25 đạt**. Các test callback IMU sử dụng message ROS Jazzy thật nhưng không khởi tạo DDS hoặc kết nối thiết bị. Các test controller vẫn dùng API/message stand-in. Không chạy lại toàn bộ suite.
- Gain, observation, limiter, cấu hình Nav2/EKF và hardware contract không đổi. Chưa flash firmware, bật motor hoặc kiểm chứng trên Pi/STM32 trong bước sửa này.

## Bổ sung teleop — 31/10/2026

- 20 test liên quan đạt: 4 keyboard-state test, 16 guard/launch test. Guard callback test dùng ROS message thật và publisher mock, không khởi tạo DDS/hardware. Không chạy lại toàn bộ suite.
- 3 package build thành công trên laptop Ubuntu/ROS Jazzy, build/install ở thư mục tạm. Các entrypoint teleop_guard và teleop_keyboard được cài đúng.
- Kiểm tra bàn phím bằng PTY và ROS thật trên domain 219 LOCALHOST, chỉ topic `/nino_review/keyboard`: phím i/j/k/q, heartbeat, timeout phím, thoát và khôi phục terminal đạt. Thời gian zero quan sát trong lượt này 0,265 s; không phải bảo đảm realtime. Không khởi chạy guard/hardware, không phát cmd_vel/motor_enable trong lượt PTY.
- PI/SAC, observation/model loader, hai sửa lỗi trước, Nav2/EKF/hardware config giữ nguyên. Không mở serial/agent, không bật motor, chưa kiểm chứng tần số/stop/phanh trên Pi/STM32 thật.
