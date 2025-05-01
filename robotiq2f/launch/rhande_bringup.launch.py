import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[ "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "tool0", "gripper_adapter_link"])

    gripper_node = Node(
        package='robotiq2f',
        executable='robotiqhande_service',
        name='robotiq_hande_node',
        output='screen')

    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory("robotiq2f_description"), "urdf", "robotiqhande", "robotiq_hande.urdf.xacro"))
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description])

    return LaunchDescription([static_tf, gripper_node, robot_state_pub_node])