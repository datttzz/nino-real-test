"""Thử đường thẳng odom không map/AMCL; mặc định shadow, không auto arm."""
from math import isfinite
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value=lambda key:LaunchConfiguration(key).perform(context)
    pi=Path(get_package_share_directory('pi5_slam_bringup'))
    own=Path(get_package_share_directory('nino_real_rl'))
    mode=value('controller');speed=float(value('speed'));maximum=float(value('max_distance'))
    if not isfinite(speed) or not 0<speed<=.25:
        raise ValueError('speed phải >0 và <=0,25 m/s')
    if not isfinite(maximum) or maximum<.05:
        raise ValueError('max_distance phải >=0,05 m')
    if mode!='shadow' and not value('hardware_file'):
        raise ValueError('Live mode cần hardware_file đã kiểm chứng')
    if mode=='pi_sac' and not value('checkpoint'):
        raise ValueError('PI+SAC cần bundle checkpoint')
    robot_path=Path(value('robot_config')).expanduser().resolve()
    robot=yaml.safe_load(robot_path.read_text())['robot']
    bridge=yaml.safe_load((pi/'config/bridge.yaml').read_text())['stm32_base_bridge']['ros__parameters']
    params=[str(pi/'config/nav2.yaml'),{'use_sim_time':False}]
    nodes=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pi/'launch/robot.launch.py')),
        launch_arguments={'mode':'path_test','robot_config':str(robot_path),
            'rviz':'false','use_sim_time':'false','allow_motor_enable':'false' if mode=='shadow' else 'true',
            'stm32_port':value('stm32_port'),'lidar_port':value('lidar_port')}.items()),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',
             output='screen',parameters=params+[{'FollowPath.desired_linear_vel':speed,
                 'FollowPath.regulated_linear_scaling_min_speed':min(.1,speed),
                 'FollowPath.min_approach_linear_velocity':min(.05,speed)}],
             remappings=[('cmd_vel','cmd_vel_nav')]),
        Node(package='nav2_velocity_smoother',executable='velocity_smoother',name='velocity_smoother',
             output='screen',parameters=params,remappings=[('cmd_vel','cmd_vel_nav')]),
        Node(package='nav2_collision_monitor',executable='collision_monitor',name='collision_monitor',
             output='screen',parameters=params+[{'cmd_vel_out_topic':'/rl/cmd_vel_reference'}]),
        TimerAction(period=5.,actions=[Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
             name='lifecycle_manager_straight',output='screen',parameters=[{'autostart':True,
                 'use_sim_time':False,'node_names':['controller_server','velocity_smoother','collision_monitor']}])]),
        Node(package='nino_real_rl',executable='real_controller',name='real_controller',output='screen',
             parameters=[str(own/'config/real.yaml'),dict(mode=mode,checkpoint=value('checkpoint'),
                 hardware_file=value('hardware_file'),map_frame='odom',path_topic='/straight/reference_path',
                 wheel_radius=float(robot['wheel_radius']),wheel_separation=float(robot['wheel_separation']),
                 left_feedback_sign=float(bridge['left_wheel_sign']),right_feedback_sign=float(bridge['right_wheel_sign']),
                 log_dir=value('log_dir'),use_sim_time=False)]),
        Node(package='nino_real_rl',executable='straight_path',name='straight_path',output='screen',
             parameters=[{'use_sim_time':False,'max_distance':maximum}])]
    if value('rviz')=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',output='screen',
                          arguments=['-d',str(own/'rviz/straight_line.rviz')]))
    return nodes


def generate_launch_description():
    pi=Path(get_package_share_directory('pi5_slam_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('controller',default_value='shadow',choices=['shadow','pi','pi_sac']),
        DeclareLaunchArgument('checkpoint',default_value=''),
        DeclareLaunchArgument('hardware_file',default_value=''),
        DeclareLaunchArgument('robot_config',default_value=str(pi/'config/robot.yaml')),
        DeclareLaunchArgument('stm32_port',default_value=''),
        DeclareLaunchArgument('lidar_port',default_value=''),
        DeclareLaunchArgument('speed',default_value='0.10',description='Vận tốc Nav2 tham chiếu m/s'),
        DeclareLaunchArgument('max_distance',default_value='3.0',description='Khoảng cách goal tối đa m'),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('log_dir',default_value='runs'),
        OpaqueFunction(function=setup)])
