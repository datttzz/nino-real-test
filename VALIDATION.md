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

## Kiểm tra bánh kê trên Pi — 08/11/2026

- Người vận hành xác nhận hai bánh kê chắc, có người trực ngắt nguồn; cho phép
  thử phần cứng. Đã kết nối Pi `192.168.0.102`, mở một micro-ROS agent, không
  chạy Nav2, PI hoặc SAC. Các mô tả chưa thử phần cứng ở trên thuộc lượt cũ.
- Đã gửi riêng xung `/cmd_torque` 0,1 Nm trong 0,1 s cho mỗi bánh; sau đó một
  phiên riêng thử `/cmd_vel` 0,03 m/s trong 0,25 s. Có kiểm tra freshness,
  timestamp, tranh quyền publisher và ngưỡng dừng tốc độ/góc quay. Mọi phiên
  đều kết thúc bằng zero + disable; nhận trạng thái disabled `67`; agent đã dừng.
- Encoder/IMU khoảng 48,2 Hz; motor status khoảng 4,83 Hz. MCU phản hồi đổi
  mode MIT/velocity và enable/disable, nhưng góc quay trong xung chỉ dao động
  khoảng 0,0004–0,0012 rad, chưa chứng minh bánh quay theo lệnh.
- Dừng publish lệnh zero làm cờ timeout xuất hiện ở phía Pi sau 0,666 s trong
  một lượt. Đây là độ trễ nhận status gồm truyền thông và chu kỳ status, không
  phải thời gian dừng vật lý; chưa chứng minh watchdog dừng vật lý <=0,5 s.
- Sau khi người vận hành gạt E OFF→ON, màn hình STM32 vẫn báo **RC:LOST**.
  Source `bsp_sbus.c:sbus_get_control_mode()` trả EMERGENCY_STOP khi SBUS
  offline/failsafe; `motor_task.c:motor_select_control_source()` ép effective
  enable về 0. Đây là tầng chặn đã xác định, kể cả `/motor_status` báo enabled.
  Chưa xác định vì sao SBUS không hợp lệ (link RF, nguồn/dây receiver, cấu hình
  ngõ ra hay UART); chưa coi các nguyên nhân còn lại đã được loại trừ.
- Không sửa `hardware.local.json`, không xác nhận giả chiều bánh/đơn vị mô-men,
  không thay firmware/gain/model/limiter. Lỗi nguồn vẫn xuất hiện trong kernel;
  các lượt bánh kê không chứng minh nguồn ổn định hoặc xe đủ điều kiện chạy sàn.
- Log nguyên bản, script dùng trong lượt và kết quả ở
  `runs/hardware_commission_20261108_1030/` và
  `runs/hardware_velocity_20261108_1036/`, giữ cả trên Pi và laptop.

### IMU trong lượt tự thử ngày 08/11

- Phiên `manual_velocity_20261108_105743_CCfjqR` không nhận `/imu/data`, dù
  encoder/status vẫn có dữ liệu. Guard dừng trước xung vận tốc và đã disable.
- Kết nối lại agent, chỉ subscribe 10 giây: nhận 472 mẫu IMU và 472 mẫu encoder
  (~47,17 Hz), 47 mẫu `/imu/debug`. Diagnostic báo init_status=0, read_valid=1,
  sample_valid=1, calibrated=1, calibration_count=3001, SPI_error=0,
  time_synced=1. Chưa đủ dữ liệu chẩn đoán trong lượt cũ để kết luận vì sao
  IMU vắng mặt lúc đó; không suy trạng thái hiện tại thành nguyên nhân quá khứ.
- Bổ sung nhận/lưu `/imu/debug` và mô tả trạng thái vào lỗi thiếu IMU của probe;
  không bỏ guard, không đổi ngưỡng/thời gian xung/firmware/gain. Probe trả mã
  lỗi khác 0 khi chưa hoàn tất. Cả hai bản script trước sửa được giữ lại.
- Parse cú pháp hai script đạt; kiểm tra ROS chỉ đọc nói trên dùng bản firmware
  hiện tại, chưa chạy lại xung motor bằng script mới. Chẩn đoán được lưu ở
  `runs/imu_diagnosis_20261108_1100/` trên Pi và laptop.


## Quy trình 3 nhóm × 5 lượt — 08/11/2026

- Bổ sung nav_only cho run_logged_real.sh; snapshot/logger/rosbag riêng từng phiên, không tự enable/arm. Logger thụ động không tạo actuator publisher/service; giữ PI/model/limiter/hardware JSON.
- Cả ba nhóm đo pose AMCL+odom TF 10 Hz, IMU ở nhịp nguồn đã trừ trọng lực; mỗi goal có cửa sổ thời gian riêng và đường đầu tiên được khóa cho phép đo. Nav2 outcome và AMCL task pass vẫn tách riêng; mục thiếu nguồn là N/A.
- 13 test liên quan đạt (6 report cũ, 6 mission/ba nhóm, 1 signal recorder). Kiểm tra cú pháp Python/Bash và 16 khối lệnh hướng dẫn đạt.
- Một kiểm tra ROS synthetic trên domain 213 LOCALHOST đạt: 30 pose, 147 IMU; RMSE kỳ vọng 0,1 m; RMS ngang/đứng 2 và 3 m/s²; pose reset y=100 m sau terminal không lọt vào chỉ số. Test không có actuator publisher, không kết nối phần cứng và không tạo kết quả thử xe thật.
- Đã kiểm tra CLI/cú pháp trên Pi; PI preflight tiếp tục từ chối JSON chưa xác minh. Chưa chạy end-to-end 15 phiên bằng wrapper mới, chưa có báo cáo số liệu thực của bộ này. Nguồn đo, giới hạn và lệnh từng bước ở docs/CHAY_15_LUOT_VA_BAO_CAO.md.

## Cập nhật hardware contract — sau xác nhận người vận hành 08/11

- Đọc thêm kết quả xung ngắn: cả hai bánh hoàn tất 0,03 Nm/0,04 s; thử dài đạt
  2,002 vòng mỗi bánh. Người vận hành xác nhận chiều tiến về LiDAR và chấp nhận
  feedback thật 48–50 Hz so với bộ nền mô phỏng 1000 Hz.
- JSON trên Pi đã sao lưu và cập nhật các mục có cơ sở: firmware torque,
  đơn vị lệnh Nm/direct output, dấu feedback/command +1/+1, chấp nhận khác rate.
  Giới hạn 2 Nm là giới hạn lệnh đối chiếu spec/giao thức, không phải thử tải
  hoặc hiệu chuẩn mô-men độc lập.
- Chỉ còn watchdog false; hàm load_hardware_contract xác nhận vẫn chặn tại
  watchdog_verified. Không tự đổi thành true từ thời gian nhận status 0,678 s.
- Hồ sơ ở runs/hardware_contract_evidence__ac79ovp trên Pi/laptop; phương án
  hoàn tất ở docs/CHOT_HARDWARE_PI_SAC.md. Chưa sửa/nạp watchdog 400 ms trong
  lượt này, chưa chạy PI/SAC hoặc gửi thêm lệnh motor.

## Nạp watchdog 400 ms — 08/11/2026, 12:51

- Theo yêu cầu nạp trực tiếp và xác nhận nguồn công suất motor đã tắt, đã sao
  lưu đủ 1 MiB flash rồi nạp bằng ST-LINK/SWD. OpenOCD kết thúc mã 0 và báo
  `Verified OK`; profile CyberGear + SBUS (`ROBOT_ROS_ONLY=0`) giữ nguyên.
- Binary SHA256: `aec73dd3212265cc1d64c293977621ec9f488f561e74265c88f008f5369275d3`.
  Hồ sơ, binary/ELF, bản sao flash trước nạp và log nằm trong
  `/home/tiendat/Downloads/Microros_H723_USB-main/artifacts/watchdog400_20261108_1230/`.
- 7 test firmware host và 5 test verifier đạt. Thêm chẩn đoán MCU vào phần
  mở rộng `/imu/debug`: mốc lệnh cuối, phát hiện timeout, enqueue và TXBTO cho
  hai bus CAN. Không thay 22 trường debug cũ, observation, gain hoặc limiter.
- Chưa kiểm chứng watchdog trên phần cứng sau nạp: Pi tại 192.168.0.102 đang
  không truy cập được. `watchdog_verified` vẫn false. Công cụ
  `scripts/verify_watchdog400.py` chỉ gửi torque zero, cần hai sự kiện MCU/CAN
  đạt và xác nhận disable trước khi cho phép ghi hardware.local.json.
- Mốc TXBTO là bằng chứng truyền frame dừng an toàn, không phải thời gian bánh
  hết chuyển động cơ học; chưa chứng minh mọi tình huống lỗi nguồn/CAN.

### Lượt kiểm tra sau khi nối lại Pi

- Chỉ đọc 12 giây: nhận 563 encoder, 559 IMU và 56 mẫu debug/status. Debug xác
  nhận schema 4001, timeout 400 ms, period 2 ms, profile SBUS. Log:
  `runs/watchdog400_readonly_i6viw4e3/` trên Pi và laptop.
- Người vận hành xác nhận bánh kê, motor có nguồn, AUTO sẵn sàng. Phép thử chỉ
  gửi torque zero nhận mode MIT và enable/timeout trong status, nhưng event_seq
  MCU vẫn 0: chưa ghi được sự kiện watchdog với effective AUTO enable hợp lệ.
  Source chỉ ghi sự kiện khi ROS/AUTO thực sự được SBUS cho phép; bit enabled
  trong motor_status riêng lẻ không chứng minh điều đó. Chưa có telemetry RC để
  phân biệt mất SBUS với DISARM/ESTOP trong lượt này.
- Lượt thử **không đạt**, JSON không sửa; đã quan sát disable sau cleanup.
  Giữ nguyên toàn bộ `runs/watchdog400_verify_i55y35p9/`, không tính lượt này
  thành kiểm chứng watchdog thành công. Đang đối chiếu màn hình RC với người
  vận hành trước khi quyết định thử lại.

### Watchdog đạt sau khi SBUS vào AUTO — 08/11/2026

- Người vận hành xác nhận đã vào AUTO và motor khởi động. Dừng agent kiểm tra
  cũ rồi chạy duy nhất verifier, chỉ gửi `/cmd_torque [0,0]`; không chạy PI,
  SAC, Nav2 hoặc lệnh chuyển động khác zero.
- Hai sự kiện watchdog mới: phát hiện sau 401/402 ms; enqueue frame dừng sau
  401/402 ms; quan sát TXBTO cả hai bus sau 403/404 ms tính từ lệnh MCU nhận
  cuối. queued_mask=3 và tx_mask=3; đây là thời gian truyền frame an toàn,
  không phải thời gian bánh dừng cơ học. Không dùng độ trễ nhận status làm
  thời gian watchdog.
- Cleanup đã quan sát motor disabled. Verifier sao lưu JSON cũ rồi cập nhật
  trên Pi `watchdog_verified=true`, `configured_watchdog_seconds=0.4`,
  `verified_watchdog_seconds=0.404`. Hàm `load_hardware_contract` hiện hành
  kiểm tra lại đạt: chặn cấu hình hardware của PI/SAC đã được gỡ.
- Hồ sơ đầy đủ trên Pi và laptop: `runs/watchdog400_verify_srt9tm5y/` gồm
  RESULT.json, samples.jsonl, agent.log, hardware.before.json,
  hardware.after.json và PREFLIGHT.json. Giữ cả lượt không đạt trước đó.
- Không chứng minh chất lượng điều khiển PI/SAC trên sàn từ phép thử zero
  torque này; chưa chạy phiên PI/SAC mới. Agent verifier đã dừng sau cleanup.

## Map/AMCL và dùng lại initial pose — 08/11/2026

- Thêm `scripts/navigation_session.py`: `ready` chờ autostart rồi phục hồi có
  giới hạn map_server/AMCL theo state; chỉ configure/activate node cần thiết,
  xác nhận cả hai active và nhận /map. Không tạo actuator publisher hoặc goal.
- `save-pose` chờ /initialpose mới từ RViz, kiểm tra frame, timestamp,
  quaternion; lưu x/y/yaw và hash map YAML/ảnh. Không dùng AMCL cuối đường làm
  pose đầu, không tự ghi đè file pose. `read-pose` từ chối map đã thay đổi.
- Wrapper dùng initial_pose.json của suite ở các lượt sau, snapshot file pose,
  ghi requested_initial_pose trong manifest và ghi /initialpose vào rosbag.
  Tự chạy readiness có log/result riêng, dừng helper khi đóng phiên.
- Trì hoãn manager localization 5 s và navigation 10 s để giảm tranh chấp
  dịch vụ lúc boot; không thay Nav2 gains, ngưỡng, firmware, model hay limiter.
- 14 test liên quan đạt, gồm vòng quay pose/map hash, dữ liệu sai, trạng thái
  chuyển tiếp, phục hồi tình huống inactive/unconfigured qua dịch vụ ROS thật
  trên domain 218 LOCALHOST cô lập; test launch offline và signal recorder.
  Kiểm tra cú pháp Python/Bash và 29 khối lệnh Bash của hướng dẫn mới đạt.
- Đã chuyển bản sửa lên Pi bằng atomic rename, có backup tại
  `runs/navigation_setup_backup_62otf_ky/`. Installed launch là symlink tới
  source, không cần build lại. Giữ nguyên phiên đang chạy.
- Kiểm tra chỉ đọc bằng helper trên Pi đạt: map_server/AMCL active, nhận /map.
  Chưa chạy một phiên phần cứng mới bằng wrapper đã sửa hoặc xác minh chuyển
  pose qua nhiều lượt trên xe; người vận hành vẫn phải kiểm tra laser và vị
  trí thực. Tài liệu đầy đủ: docs/CHAY_XE_THAT_15_LUOT.md.

### Sửa CLI initial_pose rỗng — 08/11/2026

- Lượt Nav đầu sau sửa wrapper bị ROS 2 từ chối vì `initial_pose:=` rỗng.
  Đã sửa cả nhánh Nav/PI/SAC: chỉ thêm initial_pose khi có giá trị, lượt đầu
  dùng default launch. Không truyền controller rỗng trong nhánh Nav.
- 6 regression test chạy chính khối Bash dựng argv rồi qua parser ros2launch
  thật: Nav/PI/SAC × có/chưa có pose. Đạt trên laptop và kiểm tra lại đủ 6
  trường hợp trên Pi, không chạy node điều khiển. Bash syntax đạt.
- Pi đã cập nhật atomic, backup runs/empty_pose_fix_backup__glxq4l2.
  Giữ nguyên log lỗi runs/nav_only/20261108_161021_iz3hzd; phiên không có goal,
  exit=1 và logger đã đóng. Liên kết được chuyển sang startup_failures/nav_only
  của suite real15_20261108_161019, ghi startup_failures.jsonl để công khai lỗi
  hạ tầng, không tính thành một lượt điều khiển. Chưa tự chạy lại xe.

## Chẩn đoán PI disarm sau arm — 08/11/2026

- Trong runs/pi/20261108_164359_6ZM3e2, log ghi DISARMED vì Missing/faulted
  motor status or CAN feedback. Sau đó nhánh observation PI ghi đè fault bằng
  Missing/stale reference path, khiến status không còn hiển thị lý do disarm.
- Tách lý do status thiếu/quá tuổi, CAN feedback thiếu và bus-off; thêm tuổi
  status/bits vào diagnostic. Latch lý do disarm, chỉ xóa sau manual arm thành
  công; observation_note riêng. Không thay giới hạn 0,25 s, QoS, watchdog,
  gain, model hoặc logic zero/disable. Chưa kết luận lỗi gốc là trễ status hay
  CAN từ thông báo tổng hợp cũ.
- 16 test callback liên quan đạt, gồm giữ nguyên lý do disarm sau observation
  lỗi, phân biệt các nhánh status và manual rearm. Không thử motor thêm.
- Source trên Pi đã cập nhật atomic, backup runs/disarm_diagnostics_backup_06ou8r5i;
  không restart phiên đang chạy. Diagnostic mới có hiệu lực ở phiên tiếp theo.

## Deadline riêng cho motor_status 5 Hz — 08/11/2026

- Lỗi mới xác định bằng diagnostic: age status 0,279 s vượt hạn chung 0,250 s;
  bits 0x33 cuối vẫn gồm fresh hai bánh và enable, MCU timeout khi đang chờ goal.
- Source firmware phát status mỗi 10 mẫu encoder, danh định 5 Hz. Kiểm tra
  thụ động trên Pi trong 12 s: status mean gap 0,21658 s, P95 0,21967 s, max
  0,22233 s; encoder mean 0,02165 s, max 0,03185 s. Hồ sơ:
  runs/status_timing_3i86cyip. Không suy lần đo này phủ mọi đỉnh trễ của controller.
- Tách motor_status_timeout=0,35 s, có chặn cấu hình >0,35 s; đây là tăng hạn
  freshness riêng status từ 250 lên 350 ms, làm phát hiện mất status có thể
  muộn hơn tối đa khoảng 100 ms so với trước. Encoder/IMU và command 250 ms,
  checks bit CAN/enable, timestamp, giới hạn và watchdog MCU 400 ms giữ nguyên.
- 19 test callback đạt: 279 ms không tự disarm standby PI/SAC khi dữ liệu khác
  hợp lệ; 351 ms vẫn disarm/disable; encoder quá 250 ms và lỗi CAN vẫn bị chặn.
  Không tự arm/mở goal để kiểm tra trên xe.
- Đã cập nhật code và YAML trên Pi, backup runs/status350_backup_0gsw08_s.
  Phiên hiện tại vẫn dùng code/param cũ; cần mở lại để áp dụng. Chưa xác minh
  trạng thái armed trên xe sau thay đổi; không bảo đảm không còn lỗi freshness khác.

## Mẫu odometry trễ trong controller — 08/11/2026

- Lượt PI 20261108_165749_Fp0XuC disarm với odom: Stale/future source timestamp.
  Đối chiếu bag quanh ROS time 1794131910.018544: odom nhận ở recorder có age
  khoảng 8–86 ms; không thấy toàn bộ nguồn vượt 250 ms tại đó. Đo thụ động
  12 s mới: odom max age 115 ms, IMU bridge 34 ms, wheel 37 ms; không có mẫu
  nguồn quá 250 ms. Log runs/odom_timing_urocutl3. Đây không phải bằng chứng
  loại mọi trễ, nhưng phù hợp mẫu cũ trong queue controller hoặc lệch lịch callback.
- Subscriber IMU/odom chuyển sang KEEP_LAST depth=1 best effort, không đổi
  nguồn/tốc độ phát cảm biến. Callback bỏ mẫu lỗi, chỉ giữ cache nếu timestamp
  nguồn và tuổi nhận cũ đều còn hợp lệ; không thay stamp hoặc làm mới cache
  bằng mẫu bị loại. Không có cache còn tươi thì vẫn disarm ngay.
- Basic readiness khi arm/đang armed kiểm tra cache IMU và odom cho cả PI/SAC;
  hết hạn 250 ms thì dừng dù không có callback mới. Giữ watchdog, deadline
  status 350 ms, wheel/IMU/odom 250 ms, command timeout, gain và model.
- 37 test core/runtime đạt, gồm lỗi mẫu cũ/tương lai/trùng, cache không được
  kéo dài tuổi, cache hết hạn không có callback và nguyên tắc zero/disable.
  QoS mới kiểm tra được trên Pi. Chưa tự arm, gửi goal hoặc chạy lại xe sau sửa.

## Tách controller khỏi gap IMU relay — 08/11/2026

- Lượt PI 20261108_170531_yu1U6i disarm ở ROS time 1794132405.9976608.
  Raw IMU trong bag liên tục quanh thời điểm này, tuổi khoảng 8–20 ms, chip
  initialized/read_valid/sample_valid/calibrated và SPI error=0 trong debug.
  IMU bridge có lần nhận cuối trước trip ở t-0,2499 s, source_age 0,0093 s;
  sau đó gap 0,351 s và một mẫu bridge tới ở t+0,1011 s đã cũ 0,3393 s.
  Cache IMU hết hạn 250 ms phù hợp lỗi đang thấy; bản đồ không tham gia guard
  timestamp này. Không suy sự dịch chuyển xe là nguyên nhân lỗi thời gian.
- Controller dùng imu_topic=/imu/data, IMU raw vào phải có tuổi 0..50 ms
  như gate bridge (imu_input_max_age=0,05); cache vẫn 250 ms. Frame/mounting,
  quaternion/đơn vị, observation/gain/model/watchdog giữ nguyên. Chỉ queue
  IMU input của bridge giảm depth=1; bridge/EKF vẫn sở hữu odometry/TF.
- Gắn nguồn vào lỗi cached timestamp/receipt/wheel; diagnostic có imu_source.
- 39 test core/runtime, 14 bridge và 1 tích hợp ROS cô lập đạt. Tích hợp dùng
  sensor fixtures trong domain 218 LOCALHOST, controller PI thật giữ armed
  ở mô-men zero khi raw IMU còn tươi, không có publisher IMU bridge; dừng raw
  thì controller disarm theo hạn cache. Không kết nối phần cứng ở test này.
- Đã cập nhật Pi atomic, backup runs/direct_imu_backup_rgxgdy7l. Không tự
  restart/arm/gửi goal; cần phiên mới để áp dụng. Chưa chứng minh chất lượng
  chạy xe thật sau sửa, không bảo đảm mọi nguồn trễ đã được loại trừ.

## Chuẩn bị status 50 Hz, chưa nạp — 08/11/2026

- SAC phiên 20261108_174117_jv3smq disarm ở 1794134520.5970115 vì status age
  0,490 s > 0,350 s. Bag vẫn nhận status ở khoảng t-0,3154 và t-0,1301 s;
  callback controller và recorder có độ trễ khác nhau, không kết luận duy
  nhất do mạng SAC. Probe chỉ đọc sau đó: graph query max 2,12 ms, status
  max gap 0,457 s trong cửa sổ 8 s. Không tăng deadline 350 ms hoặc fake refresh.
- Firmware thay nhịp motor_status từ mỗi 10 wheel cycles thành mỗi cycle
  danh định 20 ms; cùng bit CAN/enable/timeout và cùng nguồn CAN. Debug 40
  double vẫn mỗi 10 cycles, không đổi nhịp IMU/encoder/control/watchdog.
- Bản CyberGear + SBUS watchdog 400 ms đã build, 8 host test liên quan đạt,
  giữ source motor_task.c/h và robot_config.h trùng hash bản đã kiểm chứng.
  Artifact: /home/tiendat/Downloads/Microros_H723_USB-main/artifacts/status50_20261108_i03_joab/.
  BIN SHA256 97cf69a0dc31ee7ce9f89a7d31d0f11162b04527f2f0542766189bc558e9f260.
- Chưa nạp, chưa đo rate mới hoặc kiểm chứng lại watchdog trên board.
  ST-LINK chưa được nhận; đang chờ người vận hành chuẩn bị nguồn motor tắt
  và kết nối SWD. Giữ model, log cũ và hardware JSON hiện tại; không tự arm.

### Nạp status50 qua ST-LINK — 08/11/2026

- Người vận hành yêu cầu nạp trực tiếp, báo STM32 phải tháo khỏi xe để nạp.
  Đã sao lưu đủ 1 MiB flash trước nạp; prefix khớp byte bản watchdog400 cũ.
  Nạp ELF status50 bằng ST-LINK/V2, target STM32H723, OpenOCD exit=0 và
  `Verified OK`, reset thành công. Không gửi lệnh motor hoặc goal.
- Manifest của artifacts/status50_20261108_i03_joab ghi flash_verified=true,
  hardware_watchdog_verified=false. Image giữ CyberGear+SBUS, watchdog400;
  chu kỳ status danh định 20 ms, debug 200 ms. Đây chưa là số đo nhịp thực tế.
- Pi tại 192.168.0.102 chưa truy cập được lúc chuẩn bị nạp. Chờ gắn board lại
  để kiểm tra cảm biến/rate và lặp phép thử watchdog zero-torque. Chưa tự đổi
  JSON trên Pi hay kết luận SAC hết chặn. Bản flash/backup/hash/log được giữ.
