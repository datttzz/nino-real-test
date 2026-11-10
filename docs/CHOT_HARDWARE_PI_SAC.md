# Chốt hardware contract cho PI/SAC

## Trạng thái đã cập nhật ngày 08/11/2026

**Kết quả mới nhất:** watchdog 400 ms đã nạp/verify và hai lượt kiểm chứng MCU
trong AUTO đều đạt. Phát hiện timeout 401/402 ms; cả hai bus CAN truyền frame
dừng được quan sát sau 403/404 ms. Đây không phải thời gian dừng cơ học bánh.
JSON trên Pi đã ghi `watchdog_verified=true`, thời gian xác minh `0.404` s,
ngưỡng cấu hình `0.4` s; `load_hardware_contract` kiểm tra lại đạt. Hồ sơ:
`runs/watchdog400_verify_srt9tm5y/` trên Pi và laptop. Motor đã disable sau thử;
chưa chạy PI/SAC trên sàn trong lượt này. Các phần dưới giữ lịch sử trước khi
hoàn tất kiểm chứng, gồm cả nguyên nhân từng giữ watchdog false.

Đã sao lưu hardware.local.json trên Pi trước khi cập nhật. Hồ sơ:
`runs/hardware_contract_evidence__ac79ovp/EVIDENCE.json` và `hardware.before.json`.

- Firmware torque: đối chiếu source CAN CyberGear, test byte CAN và hồ sơ
  ST-LINK nạp/verify ngày 07/11. Binary SHA256
  `b52e06cf40513571cb6f4cd0c9eee83988698b209bb5dc959e98834ca6747271`.
  Cơ sở này giả định không có lần thay firmware nào khác chưa được ghi nhận.
- Đơn vị lệnh: CyberGear ±12 Nm, bánh gắn trực tiếp đầu ra actuator theo người
  vận hành. Giới hạn phần mềm PI/SAC 2 Nm được đối chiếu giao thức và thông số
  output của motor. Đây không phải đo mô-men độc lập hay thử tải 2 Nm trên giá kê.
- Cả hai bánh đã phản hồi xung 0,03 Nm/0,04 s; thử vận tốc riêng đạt khoảng 2 vòng
  mỗi bánh. Người vận hành xác nhận lệnh dương làm xe tiến về đầu LiDAR.
  Encoder đã có dấu dương tương ứng; command sign ở Pi chọn +1 cho cả hai.
- Người vận hành chấp nhận thử có giám sát với feedback 48–50 Hz, khác 1000 Hz
  của bộ nền mô phỏng. Không suy ra model chắc chắn ổn định/tốt hơn trên xe thật.

Bốn cờ tương ứng đã được ghi true. Chỉ còn `watchdog_verified=false` và
`verified_watchdog_seconds=0.0`; preflight PI/SAC vẫn chặn đúng tại watchdog.
Không đổi gain, model, limiter hoặc hủy kiểm tra hardware.

## Vì sao chưa chốt watchdog

**Cập nhật sau đó, 08/11 lúc 12:51:** firmware watchdog 400 ms đã được nạp và
OpenOCD verify thành công, giữ CyberGear + SBUS. Hồ sơ ở
`/home/tiendat/Downloads/Microros_H723_USB-main/artifacts/watchdog400_20261108_1230/`.
Chưa chạy kiểm chứng sau nạp vì Pi chưa online ở IP cũ; vẫn giữ cờ watchdog
false. Phần dưới ghi lý do và quy trình của bản 500 ms trước khi nạp.

Source hiện tại dùng `CMD_VEL_TIMEOUT_MS=500` và so sánh `age > timeout` trong
vòng motor 2 ms. Ngưỡng danh định 500 ms không tạo biên cho xử lý/truyền lệnh.
`/motor_status` chỉ khoảng 5 Hz; thời gian Pi nhận cờ 0,678–0,701 s bao gồm chu kỳ
status và transport. Nó không phải số đo MCU chuyển sang lệnh an toàn và không
được điền vào `verified_watchdog_seconds` hoặc làm tròn thành 0,5 s để qua chặn.

## Cách hoàn tất

1. Chuẩn bị một profile firmware có ngưỡng watchdog 400 ms, giữ SBUS, cùng thang
   CyberGear và các giới hạn/gain hiện tại. 400 ms là đề xuất chưa được sửa/nạp
   trong lượt cập nhật JSON này; không sửa giới hạn Python thành 0,7 s.
2. Thêm chẩn đoán timestamp MCU: nhận lệnh hợp lệ cuối, watchdog kích hoạt,
   bắt đầu/kết thúc gửi lệnh zero/damping an toàn, lỗi enqueue CAN nếu có.
   Dùng cùng đồng hồ MCU và phép trừ unsigned để xử lý wrap; không trừ clock Pi
   với clock STM32. Phân biệt frame được enqueue và frame đã truyền/được ACK.
3. Test/build firmware, lưu binary/hash và backup flash cũ. Ngắt nguồn công suất
   motor, nối ST-LINK rồi nạp/verify đúng profile. Không flash tự động khi xe
   đang chạy Nav2. Sau đó nối lại Pi, giữ bánh kê, người trực tại xe.
4. Chạy kiểm tra mất luồng lệnh có chủ đích; không có publisher khác làm mới
   watchdog. Đối chiếu sự kiện MCU với dữ liệu CAN/feedback; xác nhận firmware
   chuyển sang đầu ra an toàn trong giới hạn 500 ms. Không dùng thời điểm bánh
   hết quán tính làm đồng nghĩa với timeout phần mềm.
5. Chỉ sau khi có bằng chứng đạt, cập nhật `watchdog_verified=true` và thời gian
   watchdog đã xác minh, ghi rõ ngưỡng cấu hình và độ trễ xử lý đo được trong hồ
   sơ. Nếu chưa đạt, giữ false và sửa nguyên nhân; không ép pass.
6. Kiểm tra profile bằng hàm hiện có, rồi khởi động PI disarmed trước khi thử SAC.

```bash
cd ~/nino-real-test
source scripts/env.sh
PYTHONPATH="$PWD/src/nino_real_rl:${PYTHONPATH:-}" /usr/bin/python3 - <<'PY'
from nino_real_rl.core import load_hardware_contract
load_hardware_contract('hardware.local.json')
print('Hardware contract hợp lệ về cấu hình; xem hồ sơ kiểm chứng đi kèm.')
PY
```

Đơn vị và thông số tham chiếu:
[CyberGear Micromotor Instruction Manual](https://rosmo-robot.github.io/assets/img/CyberGear%E5%BE%AE%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6%20en_compressed.pdf),
mục 1.3 và 4.1.2/4.1.3. Đây là xác minh giao diện lệnh, không phải chứng nhận độ
chính xác mô-men hay bảo đảm an toàn cho mọi trạng thái nguồn/CAN/cơ khí.
