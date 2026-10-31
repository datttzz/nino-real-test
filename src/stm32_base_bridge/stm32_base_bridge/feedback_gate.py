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
        if self.received_at is None or not math.isfinite(now):
            return False
        age = now - self.received_at
        return (0 <= age <= self.timeout_s and
                self.status & 0x03 == 0x03 and self.status & 0x0C == 0)
