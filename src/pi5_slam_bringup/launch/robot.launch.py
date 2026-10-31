"""Explicit bench, synthetic demo, real mapping and navigation modes for a headless Pi."""
from pathlib import Path
import os
import stat
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV2_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
              'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor']


def navigation_nodes(share, map_yaml, sim_time, dry_run, initial_pose, command_output=''):
    """AMCL on a saved map plus the Nav2 servers, without docking or route servers."""
    params = [str(share / 'config/nav2.yaml'), {'use_sim_time': sim_time}]
    to_nav = [('cmd_vel', 'cmd_vel_nav')]

    def server(package, executable, remappings=(), extra=None):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=params + ([extra] if extra else []), remappings=list(remappings))

    def manager(name, node_names):
        return Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name=name,
                    output='screen', parameters=[{'use_sim_time': sim_time, 'autostart': True,
                                                  'node_names': node_names}])

    return [
        server('nav2_map_server', 'map_server', extra={'yaml_filename': map_yaml}),
        server('nav2_amcl', 'amcl', extra=initial_pose),
        manager('lifecycle_manager_localization', ['map_server', 'amcl']),
        server('nav2_controller', 'controller_server', to_nav),
        server('nav2_smoother', 'smoother_server'),
        server('nav2_planner', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', to_nav),
        server('nav2_bt_navigator', 'bt_navigator'),
        server('nav2_waypoint_follower', 'waypoint_follower'),
        server('nav2_velocity_smoother', 'velocity_smoother', to_nav),
        # dry_run keeps the whole stack running but sends the final command nowhere.
        server('nav2_collision_monitor', 'collision_monitor',
               extra={'cmd_vel_out_topic': command_output} if command_output else
                     ({'cmd_vel_out_topic': 'cmd_vel_dry'} if dry_run else None)),
        # Allow lifecycle services to be discovered before the first transition.
        TimerAction(period=5.0, actions=[manager('lifecycle_manager_navigation', NAV2_NODES)]),
    ]


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    mode = value('mode')
    share = Path(get_package_share_directory('pi5_slam_bringup'))
    robot = yaml.safe_load(Path(value('robot_config')).read_text())['robot']

    def flag(key, automatic):
        return automatic if value(key) == 'auto' else value(key) == 'true'

    agent = flag('start_agent', mode != 'demo')
    lidar = flag('start_lidar', mode in ('mapping', 'navigation'))
    if mode == 'demo' and (agent or lidar):
        raise RuntimeError('Demo mode cannot open STM32 or LiDAR devices')
    if mode == 'demo' and int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        raise RuntimeError('Demo requires an unused nonzero ROS domain. Use ./scripts/run.sh demo (default domain 77).')
    if mode == 'bench' and lidar:
        raise RuntimeError('Use mapping or navigation mode for the LiDAR')
    stm32_port = value('stm32_port') or robot['stm32_port']
    lidar_port = value('lidar_port') or robot['lidar_port']
    for enabled, device in ((agent, stm32_port), (lidar, lidar_port)):
        if enabled and (not Path(device).exists() or not stat.S_ISCHR(os.stat(device).st_mode)):
            raise RuntimeError(f'Serial device is unavailable: {device}. Run scripts/setup_udev.sh or set the port argument.')
        if enabled and not os.access(device, os.R_OK | os.W_OK):
            raise RuntimeError(f'No serial access to {device}; log in again after joining dialout.')

    mappings = {key: (' '.join(map(str, val)) if isinstance(val, list) else str(val))
                for key, val in robot.items()}
    description = xacro.process_file(str(share / 'urdf/robot.urdf.xacro'), mappings=mappings).toxml()
    permit_motor_enable = mode in ('mapping', 'navigation', 'teleop') and value('allow_motor_enable') == 'true'
    sim_time = value('use_sim_time') == 'true'
    slam_params = value('slam_params_file') or str(share / 'config/slam.yaml')
    nodes = [
        LogInfo(msg=f'pi5-stm32-slam mode={mode}; motor enable permitted={permit_motor_enable}'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': sim_time}], output='screen'),
        Node(package='stm32_base_bridge', executable='stm32_base_bridge', name='stm32_base_bridge',
             parameters=[str(share / 'config/bridge.yaml'),
                         {'wheel_radius': float(robot['wheel_radius']),
                          'wheel_separation': float(robot['wheel_separation']),
                          'allow_motor_enable': permit_motor_enable,
                          'use_sim_time': sim_time}],
             output='screen'),
    ]
    if agent:
        nodes.append(Node(package='micro_ros_agent', executable='micro_ros_agent',
                          arguments=['serial', '--dev', stm32_port, '--baudrate', '921600', '-v', '3'],
                          output='screen'))
    if mode == 'demo':
        nodes.append(LogInfo(msg='SYNTHETIC DEMO: scans and wheel data are simulated; the map is not a real measurement.'))
        nodes.append(Node(package='pi5_slam_bringup', executable='demo_sensors',
                          parameters=[{'wheel_radius': float(robot['wheel_radius']),
                                       'wheel_separation': float(robot['wheel_separation']),
                                       'laser_x': float(robot['laser_xyz'][0]),
                                       'laser_y': float(robot['laser_xyz'][1]),
                                       'laser_yaw': float(robot['laser_rpy'][2])}], output='screen'))
    if mode not in ('bench', 'teleop'):
        nodes.append(Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                          parameters=[str(share / 'config/ekf.yaml'), {'use_sim_time': sim_time}],
                          remappings=[('odometry/filtered', '/odom')], output='screen'))
    if mode == 'navigation':
        map_yaml = str(Path(value('map')).expanduser().resolve()) if value('map') else ''
        if not map_yaml or not Path(map_yaml).is_file():
            raise RuntimeError('Navigation needs map:=path/to/map.yaml (saved maps are in maps/).')
        dry_run = value('dry_run') == 'true'
        initial_pose = None
        if value('initial_pose'):
            x, y, yaw = (float(v) for v in value('initial_pose').split(','))
            initial_pose = {'set_initial_pose': True, 'initial_pose.x': x, 'initial_pose.y': y,
                            'initial_pose.yaw': yaw}
        command_output = value('command_output')
        nodes.append(LogInfo(msg=f'Nav2 on {map_yaml}; final command topic: '
                                 f'{command_output or ("/cmd_vel_dry" if dry_run else "/cmd_vel")}'))
        nodes += navigation_nodes(share, map_yaml, sim_time, dry_run, initial_pose, command_output)
    elif mode in ('demo', 'mapping'):
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('slam_toolbox')) /
                                              'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': str(sim_time).lower(), 'autostart': 'true',
                              'slam_params_file': slam_params}.items()))
    deskew = mode in ('mapping', 'navigation') and bool(robot.get('scan_deskew', False))
    if deskew:
        nodes.append(Node(package='pi5_slam_bringup', executable='scan_deskew', name='scan_deskew',
                          parameters=[{'laser_x': float(robot['laser_xyz'][0]),
                                       'laser_y': float(robot['laser_xyz'][1]),
                                       'laser_yaw': float(robot['laser_rpy'][2]),
                                       'use_sim_time': sim_time}], output='screen'))
    if lidar:
        nodes.append(Node(package='rplidar_ros', executable='rplidar_composition', name='rplidar_node',
                          remappings=[('scan', '/scan_raw')] if deskew else [],
                          parameters=[{'channel_type': 'serial', 'serial_port': lidar_port,
                                       'serial_baudrate': int(robot['lidar_baudrate']),
                                       'frame_id': 'laser', 'inverted': False,
                                       'angle_compensate': True,
                                       'scan_mode': str(robot.get('lidar_scan_mode', 'Standard'))}], output='screen'))
    if value('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(share / 'rviz/slam.rviz')], output='screen'))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('pi5_slam_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='bench', choices=['bench', 'demo', 'mapping', 'navigation', 'teleop']),
        DeclareLaunchArgument('robot_config', default_value=str(share / 'config/robot.yaml')),
        DeclareLaunchArgument('start_agent', default_value='auto', choices=['auto', 'true', 'false']),
        DeclareLaunchArgument('start_lidar', default_value='auto', choices=['auto', 'true', 'false']),
        DeclareLaunchArgument('stm32_port', default_value=''),
        DeclareLaunchArgument('lidar_port', default_value=''),
        DeclareLaunchArgument('allow_motor_enable', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='true for rosbag replay with --clock'),
        DeclareLaunchArgument('slam_params_file', default_value='',
                              description='SLAM Toolbox parameters; empty uses config/slam.yaml'),
        DeclareLaunchArgument('map', default_value='',
                              description='Map YAML for navigation mode'),
        DeclareLaunchArgument('initial_pose', default_value='',
                              description='navigation: x,y,yaw (m, m, rad) in the map frame for AMCL'),
        DeclareLaunchArgument('dry_run', default_value='false', choices=['true', 'false'],
                              description='navigation: publish the final command on /cmd_vel_dry'),
        DeclareLaunchArgument('command_output', default_value='',
                              description='Optional final collision-monitor output topic; RL uses a non-firmware reference'),
        OpaqueFunction(function=setup),
    ])
