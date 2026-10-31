# Nino Real Test scope

- ROS 2 Jazzy, real Pi5/STM32; do not introduce Gazebo, GT or training.
- Keep PI gains .30/.10/Kd0 and the imported checkpoint observation contract.
- Never start motors or flash firmware as an automatic validation step.
- Shadow must not publish on firmware command topics or motor_enable.
- Preserve independent bridge/EKF ownership of odometry/TF.
- Run only tests related to changed code. Offline unit tests do not prove ROS,
  firmware or real-robot performance. State verification limits accurately.
- Preserve user maps/checkpoints/hardware configuration and logs.
- Use the user's explicit instructions to define any further scope.
