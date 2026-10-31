"""One bringup: hardware navigation plus PI/SAC, isolated from firmware cmd_vel."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda k: LaunchConfiguration(k).perform(context)
    pi = Path(get_package_share_directory('pi5_slam_bringup'))
    own = Path(get_package_share_directory('nino_real_rl'))
    robot_path = Path(value('robot_config')).expanduser().resolve()
    robot = yaml.safe_load(robot_path.read_text())['robot']
    bridge = yaml.safe_load((pi/'config/bridge.yaml').read_text())['stm32_base_bridge']['ros__parameters']
    mode = value('controller')
    if mode in ('pi','pi_sac') and not value('hardware_file'):
        raise RuntimeError('Live modes require verified hardware_file; start shadow first')
    if mode == 'pi_sac' and not value('checkpoint'):
        raise RuntimeError('PI+SAC requires imported 50k checkpoint directory')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pi/'launch/robot.launch.py')),
            launch_arguments={'mode':'navigation', 'map':value('map'),
                'robot_config':str(robot_path), 'rviz':value('rviz'),
                'initial_pose':value('initial_pose'), 'use_sim_time':'false',
                'allow_motor_enable':'false' if mode == 'shadow' else 'true',
                'command_output':'/rl/cmd_vel_reference',
                'stm32_port':value('stm32_port'), 'lidar_port':value('lidar_port')}.items()),
        Node(package='nino_real_rl', executable='real_controller', name='real_controller', output='screen',
             parameters=[str(own/'config/real.yaml'), dict(
                 mode=mode, checkpoint=value('checkpoint'), hardware_file=value('hardware_file'),
                 wheel_radius=float(robot['wheel_radius']), wheel_separation=float(robot['wheel_separation']),
                 left_feedback_sign=float(bridge['left_wheel_sign']),
                 right_feedback_sign=float(bridge['right_wheel_sign']),
                 log_dir=value('log_dir'), use_sim_time=False)]),
    ]


def generate_launch_description():
    pi = Path(get_package_share_directory('pi5_slam_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='shadow', choices=['shadow','pi','pi_sac']),
        DeclareLaunchArgument('map', description='Real map YAML; required'),
        DeclareLaunchArgument('checkpoint', default_value='', description='Imported 50k bundle directory'),
        DeclareLaunchArgument('hardware_file', default_value=''),
        DeclareLaunchArgument('robot_config', default_value=str(pi/'config/robot.yaml')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('initial_pose', default_value=''),
        DeclareLaunchArgument('stm32_port', default_value=''),
        DeclareLaunchArgument('lidar_port', default_value=''),
        DeclareLaunchArgument('log_dir', default_value='runs'),
        OpaqueFunction(function=setup),
    ])
