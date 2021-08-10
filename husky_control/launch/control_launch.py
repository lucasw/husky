

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    robot_description_path = os.path.join(
         get_package_share_directory('husky_description'),
        'urdf',
        'husky.urdf.xacro',
    )

    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    husky_diff_drive_controller = os.path.join(
        get_package_share_directory("husky_control"),
        "config",
        "control.yaml",
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, husky_diff_drive_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # husky_base_node = Node(
    #     package="husky_base",
    #     executable="husky_node",
    #     output="screen",
    #     parameters=[robot_description],    
    # )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_dd_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["husky_velocity_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            controller_manager_node,
            # husky_base_node,
            spawn_controller,
            spawn_dd_controller,
        ]
    )
