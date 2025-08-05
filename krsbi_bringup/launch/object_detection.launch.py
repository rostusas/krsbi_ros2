from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    yolo_node = Node(
        package="krsbi_pkg",
        executable="object_detection_node"
    )

    camera_node  = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node"
    )

    ld.add_action(yolo_node)
    ld.add_action(camera_node)

    return ld