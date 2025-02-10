import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    tap_config_dir = os.path.join(get_package_share_directory('tap'), 'config')

    print("Using config directory:", tap_config_dir)  # Debugging

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        arguments=[
            '-configuration_directory', tap_config_dir,
            '-configuration_basename', 'cartographer_noodom_x3.lua'
        ],
        output='screen'
    )

    return LaunchDescription([cartographer_node])
