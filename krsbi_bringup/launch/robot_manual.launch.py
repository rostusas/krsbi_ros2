from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_movement_manual_node = Node(
        package="krsbi_pkg",
        executable="robot_movement_manual_node"
    )

    manual_ui_node = Node(
        package="krsbi_pkg",
        executable="manual_ui_node"
    )

    ld.add_action(robot_movement_manual_node)
    ld.add_action(manual_ui_node)

    return ld