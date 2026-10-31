"""Changes for pi5-stm32-slam: distinguish real CAN feedback from zero placeholders."""
from dataclasses import dataclass
import math


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
