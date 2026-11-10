# PI 3 lượt và PI+SAC80k 3 lượt — xe thật

## Điều kiện trước khi chạy

AUTO trên tay chỉ chọn nguồn ROS. Phiên `scripts/run.sh navigation` hiện tại
đi qua `/cmd_vel` tới vòng vận tốc của STM32: không gọi phiên này là PI của
adapter hoặc PI+SAC. Các nhóm dưới dùng PI Kp=0,30/Ki=0,10 trên Pi và
`/cmd_torque`; chỉ được chạy sau khi hardware contract đã được kiểm chứng.

Ngày 07/11/2026, Pi `192.168.0.109` vẫn có hardware.local.json toàn false/null.
Chưa có bằng chứng xác nhận đường mô-men và watchdog trên bản MCU đang nạp.
Source cũ đã đối chiếu dùng thang RobStride ±60 N.m trong khi motor được người
dùng xác nhận là Xiaomi CyberGear. Không tự bật các cờ để vượt qua bước này.
Firmware ROS-only được build trước đó chỉ bỏ phụ thuộc SBUS, không sửa thang CAN.

Không tự chạy sáu lượt bằng vòng lặp vì phải dừng, disable và đưa xe về điểm
đầu giữa mỗi lượt. Giữ cùng map, start, goal, tốc độ và cấu hình; các đường do
Nav2 replan có thể khác nhau, nên không tuyên bố cùng đường nếu chưa đối chiếu.
Chưa có 6 phiên thật thì không có báo cáo kết quả thật.

## Terminal 1 trên Pi: chạy từng lượt khi hardware contract đã đạt

Dừng launch Nav2 thuần hoặc bringup khác bằng Ctrl+C tại terminal của nó trước.
Không chạy cùng lúc hai agent/controller/driver cùng cổng. Nạp môi trường:

```bash
cd ~/nino-real-test
source scripts/env.sh
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/pi5_slam_bringup/config/fastdds_unicast.xml"
```

Chạy lệnh PI sau **ba lần riêng**, mỗi lần ứng với một lượt:

```bash
./scripts/run_logged_real.sh pi \
  "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  "$HOME/nino-real-test/hardware.local.json"
```

Sau khi PI đã xác minh hoạt động đúng, chạy lệnh SAC80k **ba lần riêng**:

```bash
./scripts/run_logged_real.sh sac80k \
  "$HOME/nino-real-test/maps/phong_that_20261104_122339.yaml" \
  "$HOME/nino-real-test/hardware.local.json"
```

Giữ tên thư mục mỗi lượt mà script in ra; không lấy lượt startup bị chặn làm
một lượt hoàn thành. Không xóa lượt thất bại; ghi nguyên nhân trong outcome.json.

## RViz trên laptop và terminal điều khiển trên Pi

RViz laptop dùng `src/pi5_slam_bringup/rviz/slam.rviz`, cùng domain/RMW/profile
như trên; đặt Fixed Frame=map, initial pose đúng vị trí/hướng và kiểm tra scan
khớp tường. Thêm `nav2_rviz_plugins/GoalTool` bằng dấu + trên toolbar để gửi
Nav2 Goal qua action. Đặt goal sau khi controller đã sẵn sàng và arm thành công.

Mở terminal Pi thứ hai, nạp cùng môi trường ở trên. Tay cũ phải AUTO và có link,
không giữ cần tay làm nguồn lệnh. Enable/arm thủ công, chỉ tiếp tục khi success:

```bash
ros2 service call /set_motor_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /rl/arm std_srvs/srv/SetBool '{data: true}'
```

Khi kết thúc hoặc cần dừng:

```bash
./scripts/stop_real.sh
```

Sau đó Ctrl+C terminal script ghi log, chờ đóng rosbag. Motor disable trước khi
đưa xe về điểm xuất phát. Mỗi lần mở launch phải initial pose lại, không giả TF.
Ghi outcome Nav2, AMCL task pass và lý do dừng từ dữ liệu/quan sát vào
outcome.json của từng lượt; để null nếu không biết. Không suy pass từ RMSE thấp.

## Tải dữ liệu và tạo báo cáo trên laptop

```bash
TRAIN_REPO="$HOME/project/Ninobot_controlled_with_torque-add_rl_2"
PI_HOST=192.168.0.109
mkdir -p "$TRAIN_REPO/real_robot_data/pi" "$TRAIN_REPO/real_robot_data/sac80k"
scp -r "d@$PI_HOST:/home/d/nino-real-test/runs/pi/." "$TRAIN_REPO/real_robot_data/pi/"
scp -r "d@$PI_HOST:/home/d/nino-real-test/real_data/sac80k/." "$TRAIN_REPO/real_robot_data/sac80k/"
```

Chọn đúng sáu thư mục phiên, mỗi thư mục chứa manifest.json, config/, controller/
và rosbag/. Không dùng wildcard lấy cả lượt startup lỗi hoặc trùng một lượt ba lần.

```bash
cd /home/tiendat/Downloads/nino-real-test
read -e -r -p 'Thư mục PI lượt 1: ' PI1
read -e -r -p 'Thư mục PI lượt 2: ' PI2
read -e -r -p 'Thư mục PI lượt 3: ' PI3
read -e -r -p 'Thư mục SAC80k lượt 1: ' SAC1
read -e -r -p 'Thư mục SAC80k lượt 2: ' SAC2
read -e -r -p 'Thư mục SAC80k lượt 3: ' SAC3
OUT="$TRAIN_REPO/benchmark_results/real_PI_SAC80k_3runs_$(date +%Y%m%d_%H%M%S)"
python3 scripts/compare_real_runs.py \
  --pi-dir "$PI1" "$PI2" "$PI3" \
  --sac-dir "$SAC1" "$SAC2" "$SAC3" \
  --output-dir "$OUT"
xdg-open "$OUT/report.html"
```

Báo cáo: `summary.csv` (Mean ± sample Std qua các lượt), `episodes.csv`
(mọi lượt, kể cả thất bại điều khiển), HTML offline. Hash model và số 80.000
timesteps được kiểm tra từ snapshot, không dựa vào tên thư mục.

RMSE theo pose AMCL và trọng số dt; rung là RMS gia tốc ngang/đứng đã trừ trọng
lực tại nhịp logger actor khoảng 10 Hz. Đây không phải phân tích rung cao tần;
raw IMU đầy đủ giữ trong rosbag cho xử lý tiếp. Phần trăm thay đổi =
100 × (trung bình PI − trung bình SAC)/trung bình PI; dương là giảm, âm là tăng.
Không tính khi thiếu lượt hợp lệ hoặc baseline gần zero. Coverage/gap phải được
đọc cùng chỉ số; PI có thể thiếu observation dù vòng điều khiển vẫn chạy.

Chưa tự xác minh terminal outcome, cùng đường hoặc toàn bộ khoảng nhiệm vụ từ
rosbag. Không suy thời gian hoàn thành từ span log. Ghi N/A khi không đủ nguồn.
Ba lượt là đối chiếu ban đầu, không chứng minh cải thiện có ý nghĩa thống kê.
Thư mục SAC thực tế được giữ riêng cho xử lý dữ liệu train sau, chưa là replay
buffer và không được truyền thẳng vào --replay-buffer của trainer.
