"""ROS 2 launch file for the Drone mission node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone')
    default_config = pkg_dir + '/config'

    return LaunchDescription([
        DeclareLaunchArgument(
            'mavros_ns',
            default_value='/mavros/',
            description='MAVROS namespace prefix',
        ),
        DeclareLaunchArgument(
            'config_dir',
            default_value=default_config,
            description='Path to config directory containing YAML files',
        ),
        Node(
            package='drone',
            executable='drone_node',
            name='drone_node',
            parameters=[{
                'mavros_ns': LaunchConfiguration('mavros_ns'),
                'config_dir': LaunchConfiguration('config_dir'),
            }],
            output='screen',
        ),
    ])
