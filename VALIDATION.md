# Kiểm chứng bản tích hợp

- **21 test offline đạt**: observation đúng thứ tự/scale, trọng lực IMU, heading wrap, PI gains, torque/slew/speed limit, stop, timestamp, hardware contract, shadow không actuation, tranh quyền topic, thiếu subscriber firmware, watchdog encoder/command, arm trước goal và lỗi observation khi đang chạy.
- Parse cú pháp toàn bộ Python, XML 3 package và bash scripts: đạt.
- Test callback sử dụng message/API stand-in để kiểm tra quyết định controller; không phải chạy ROS, DDS, micro-ROS hay firmware.
- **Chưa build/launch ROS, chưa load model thật, chưa chạy Pi, chưa bật động cơ.** Môi trường tạo gói không có ROS, Torch/SB3 hoặc checkpoint.
- Firmware thực tế, dấu/đơn vị mô-men và watchdog chưa xác nhận; hardware.example.json mặc định chặn live mode.
- Tần số feedback/controller thật khác mô phỏng; chưa có kết quả sim-to-real hoặc kết quả cải thiện tracking/vibration.
- Hai repo gốc không bị sửa. Không thay checkpoint hoặc nới ngưỡng.

Các lệnh và điều kiện còn cần trên Pi nằm trong README.md. VALIDATION.json là bản máy đọc được của phạm vi này.
