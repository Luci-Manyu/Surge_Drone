"""Publish the drone URDF via robot_state_publisher (base_link -> camera_link -> optical TF).

Can be run standalone with RViz + joint_state_publisher_gui to eyeball the model:
    ros2 launch drone_description description.launch.py rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("drone_description")
    xacro_file = os.path.join(pkg, "urdf", "drone.urdf.xacro")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    gui = LaunchConfiguration("gui")

    robot_description = {
        # value_type=str so the (large) xacro XML isn't misparsed as YAML.
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]), value_type=str),
        "use_sim_time": use_sim_time,
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="joint_state_publisher_gui (only useful standalone)"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        # Only needed when viewing the model standalone (no Gazebo/SLAM driving TF).
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(rviz),
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("drone_description"), "rviz", "slam.rviz"])],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
