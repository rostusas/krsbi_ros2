from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_movement_auto_node = Node(
        package="krsbi_pkg",
        executable="robot_movement_auto_node"
    )

    auto_ui_node = Node(
        package="krsbi_pkg",
        executable="auto_ui_node"
    )

    ld.add_action(robot_movement_auto_node)
    ld.add_action(auto_ui_node)

    return ld