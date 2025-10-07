from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Buat argumen agar bisa diatur lewat command line
    camera_index = LaunchConfiguration('camera_index')

    declare_camera_index = DeclareLaunchArgument(
        'camera_index',
        default_value='/dev/video0',  # default kamera
        description='Video device path'
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[{'video_device': camera_index}]
    )

    yolo_node = Node(
        package='krsbi_pkg',
        executable='object_detection_node'
    )

    ld = LaunchDescription()
    ld.add_action(declare_camera_index)
    ld.add_action(camera_node)
    ld.add_action(yolo_node)

    return ld
