from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoints_file = LaunchConfiguration('waypoints_file')
    estop_gpio_pin = LaunchConfiguration('estop_gpio_pin')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            description='Path to waypoints JSON/YAML file (or leave empty for hardcoded)'),
        DeclareLaunchArgument(
            'estop_gpio_pin',
            default_value='18',
            description='Jetson GPIO pin number for E-stop button'),

        Node(
            package='nav2_tracer_agilex',
            executable='estop_node',
            name='estop_node',
            output='screen',
            parameters=[{
                'gpio_pin': estop_gpio_pin,
                'pause_service': '/pause_navigation',
                'resume_service': '/resume_navigation',
            }],
        ),

        Node(
            package='nav2_tracer_agilex',
            executable='mission_runner',
            name='mission_runner',
            output='screen',
            parameters=[{
                'waypoints_file': waypoints_file,
            }],
        ),
    ])
