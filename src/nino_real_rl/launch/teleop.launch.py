"""Hardware + guarded firmware-velocity teleop, without Nav2, map or SAC."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pi=Path(get_package_share_directory('pi5_slam_bringup'))
    own=Path(get_package_share_directory('nino_real_rl'))
    return LaunchDescription([
        DeclareLaunchArgument('robot_config',default_value=str(pi/'config/robot.yaml')),
        DeclareLaunchArgument('stm32_port',default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pi/'launch/robot.launch.py')),
            launch_arguments={'mode':'teleop','robot_config':LaunchConfiguration('robot_config'),
                              'stm32_port':LaunchConfiguration('stm32_port'),
                              'start_agent':'true','start_lidar':'false','rviz':'false',
                              'use_sim_time':'false','allow_motor_enable':'true'}.items()),
        Node(package='nino_real_rl',executable='teleop_guard',name='teleop_guard',output='screen',
             parameters=[str(own/'config/teleop.yaml'),{'use_sim_time':False}]),
    ])
