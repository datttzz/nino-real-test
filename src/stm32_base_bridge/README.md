# STM32 base bridge

Derived from [datttzz/linoafterfix](https://github.com/datttzz/linoafterfix), Apache-2.0.
See the repository [README](../../README.md) for launch and installation.

Changes for this project:

- `/imu/data` -> `/imu/bridge`, with no input/output topic loop.
- Wheel radius/separation supplied by the shared robot profile.
- Odometry and wheel joints require recent status with both CAN feedback flags
  set and neither CAN bus-off flag set. Losing feedback resets the encoder
  reference; reconnecting does not integrate the missing interval as a jump.
- Explicit `allow_motor_enable` parameter, false by default, controls the
  `/set_motor_enable` service. Disabling motors remains allowed.
- Joint output matches the two-wheel URDF; no artificial caster joints.
- Reject non-finite position and velocity feedback.
- Preserve the original IMU source timestamp. Missing, stale (default >50 ms),
  future, duplicate or backwards timestamps are dropped, not restamped at receipt.
  Invalid samples do not refresh the last valid IMU timestamp.

This bridge does not filter the IMU or publish velocity commands. Firmware still
owns the watchdog and the direct `/motor_enable` topic.
