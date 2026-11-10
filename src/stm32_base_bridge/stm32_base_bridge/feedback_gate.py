"""Changes for pi5-stm32-slam: distinguish real CAN feedback from zero placeholders."""
from dataclasses import dataclass
import math


def imu_source_stamp_ns(sec, nanosec, now_ns, previous_ns=None, max_age_s=0.05):
    """Giữ timestamp nguồn; mẫu lỗi không được đổi thành giờ nhận hoặc làm mới gate."""
    if not math.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError('Invalid IMU stamp tolerance')
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError('Invalid IMU source timestamp')
    stamp_ns = sec * 1_000_000_000 + nanosec
    if stamp_ns <= 0 or not 0 <= now_ns - stamp_ns <= max_age_s * 1e9:
        raise ValueError('Missing, stale or future IMU source timestamp')
    if previous_ns is not None and stamp_ns <= previous_ns:
        raise ValueError('Duplicate or backwards IMU source timestamp')
    return stamp_ns


@dataclass
class FeedbackGate:
    timeout_s: float = 0.6
    status: int = 0
    received_at: float | None = None

    def update(self, status: int, now: float) -> None:
        self.status = status
        self.received_at = now

    def ready(self, now: float) -> bool:
        return not self.block_reason(now)

    def block_reason(self, now: float) -> str:
        """Lý do ngừng xuất odometry; gate này không chặn /cmd_vel."""
        if self.received_at is None:
            return 'motor status not received'
        if not math.isfinite(now):
            return 'invalid clock'
        age = now - self.received_at
        if not math.isfinite(age) or age < 0:
            return 'invalid or backwards status clock'
        if age > self.timeout_s:
            return f'motor status stale: age={age:.3f}s, limit={self.timeout_s:.3f}s'
        if self.status & 0x0C:
            return f'CAN bus-off: status=0x{self.status:02x}'
        if self.status & 0x03 != 0x03:
            return f'missing feedback from one or both wheels: status=0x{self.status:02x}'
        return ''
