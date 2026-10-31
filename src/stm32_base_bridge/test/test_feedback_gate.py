"""A dead CAN link must never turn placeholder zeros into wheel odometry."""
import pytest
from stm32_base_bridge.feedback_gate import FeedbackGate, imu_source_stamp_ns


def test_can_loss_and_recovery():
    gate = FeedbackGate(timeout_s=.6)
    assert not gate.ready(1.)
    gate.update(32, 1.)  # Firmware connected, but motors have no power.
    assert not gate.ready(1.1)
    gate.update(1, 1.2)  # Only one wheel is responding.
    assert not gate.ready(1.3)
    gate.update(3, 2.)
    assert gate.ready(2.2)
    assert not gate.ready(2.7)  # Status publication was lost.
    gate.update(3 | 4, 3.)  # Both fresh flags with CAN1 bus-off is invalid.
    assert not gate.ready(3.1)
    gate.update(3 | 32, 4.)  # Command timeout does not invalidate passive feedback.
    assert gate.ready(4.1)
    assert not gate.ready(3.9)  # Clock reset requires new status.


def test_imu_gate_preserves_original_sample_time():
    assert imu_source_stamp_ns(9,980_000_000,10_000_000_000)==9_980_000_000


@pytest.mark.parametrize('sec,nsec,previous',[
    (0,0,None), (1,0,None), (10,1,None),
    (9,980_000_000,9_980_000_000), (9,980_000_000,9_990_000_000),
    (-1,0,None), (9,1_000_000_000,None),
])
def test_imu_gate_rejects_invalid_time_without_restamping(sec,nsec,previous):
    with pytest.raises(ValueError):
        imu_source_stamp_ns(sec,nsec,10_000_000_000,previous)
