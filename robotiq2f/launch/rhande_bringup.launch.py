from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gripper_node = Node(
        package="robotiq2f",
        executable="robotiqhande_service",
        name="robotiq_hande_node",
        output="screen",
    )

    return LaunchDescription([gripper_node])
