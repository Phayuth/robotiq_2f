import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory("robotiq2f_description"), "urdf", "robotiq85", "robotiq_85.xacro"))
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robotiq2f_description"),
            "config",
            "r85_controllers.yaml",
        ]
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq85_forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            control_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            robot_state_pub_node,
        ]
    )
