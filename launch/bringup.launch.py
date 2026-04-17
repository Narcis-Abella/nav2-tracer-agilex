import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('nav2_tracer_agilex')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    collision_monitor_params = LaunchConfiguration('collision_monitor_params')
    bt_xml_file = LaunchConfiguration('bt_xml_file')

    # Static TF: base_link → lidar_frame
    # VALIDATE: measure actual sensor offset after mounting
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to map YAML file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'collision_monitor_params',
            default_value=os.path.join(pkg, 'config', 'collision_monitor.yaml')),
        DeclareLaunchArgument(
            'bt_xml_file',
            default_value=os.path.join(pkg, 'bt_xml',
                                        'navigate_w_replanning_and_recovery.xml')),
        # Lidar mount offset — update after measuring real installation
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.25'),   # ~25cm above base
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        # Static TF: base_link → lidar_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=[
                lidar_x, lidar_y, lidar_z,
                lidar_yaw, lidar_pitch, lidar_roll,
                'base_link', 'lidar_frame'
            ],
        ),

        # PointCloud2 → LaserScan (2D projection for AMCL + local costmap obstacle layer)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc_to_laserscan',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 2.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00436,  # ~0.25deg resolution
                'scan_time': 0.1,            # 10Hz
                'range_min': 0.45,
                'range_max': 50.0,
                'use_inf': True,
            }],
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan'),
            ],
        ),

        # Nav2 bringup (localization + navigation)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_lifecycle_mgr': 'true',
            }.items(),
        ),

        # CollisionMonitor (independent of Nav2 lifecycle)
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[collision_monitor_params],
        ),

        # VelocitySmoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[collision_monitor_params],  # shares same file
        ),
    ])
