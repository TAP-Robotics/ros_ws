import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Hardcoded package path for Windows
ROS_WS = r"C:\Users\josti\Documents\ros_ws"
PKG_PATH = os.path.join(ROS_WS, "src", "tap")

def generate_launch_description():
    # Hardcoded configuration file path
    configuration_directory = os.path.join(PKG_PATH, "config")
    configuration_basename = "cartographer_noodom_x3.lua"

    # Nodes
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-configuration_directory", configuration_directory,
            "-configuration_basename", configuration_basename
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-resolution", "0.05",
            "-publish_period_sec", "1.0"
        ],
    )

    # Launch description
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld