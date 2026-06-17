"""Top-level bringup: drive the drone with the keyboard and watch RTAB-Map build the map.

What this launches (the ROS 2 side):
  * robot_state_publisher  (base_link -> camera_link -> camera_depth_optical_frame TF)
  * MAVROS                 (PX4 <-> ROS 2 bridge)
  * offboard_keyboard      (/cmd_vel -> PX4 OFFBOARD velocity; auto arm + takeoff)
  * RTAB-Map               (rgbd_odometry + rtabmap visual SLAM)
  * RViz2                  (map cloud + 2D grid + robot + camera)

Run these TWO things in their own terminals (each needs its own process/TTY):

  Terminal A  -- PX4 SITL + Gazebo (or pass start_px4:=true here):
      cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_s500_depth__slam_world

  Terminal B  -- keyboard teleop (needs an interactive terminal for keystrokes):
      ros2 run teleop_twist_keyboard teleop_twist_keyboard

  Terminal C  -- this bringup:
      ros2 launch drone_bringup slam_sim.launch.py

Teleop keys:  i/, = fwd/back   j/l = yaw   t/b = up/down   (hold shift for strafe)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(pkg, launch_file, condition=None, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", launch_file])),
        condition=condition,
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_px4 = LaunchConfiguration("start_px4")
    start_mavros = LaunchConfiguration("start_mavros")
    start_slam = LaunchConfiguration("start_slam")
    start_rviz = LaunchConfiguration("start_rviz")

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("drone_description"), "rviz", "slam.rviz"])

    return LaunchDescription([
        # MAVROS crashes on rmw_fastrtps_cpp (CompanionProcessStatus bug); pin CycloneDDS for
        # every node this launch spawns. The Gazebo/PX4 terminal must export the same (see
        # ROS_2/source_env.sh) so the camera topics are on the same RMW vendor.
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),

        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_px4", default_value="false",
                              description="Also run the PX4 'make' sim (usually a separate terminal)."),
        DeclareLaunchArgument("start_mavros", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),

        # PX4 SITL + Gazebo (opt-in; long first build)
        _include("drone_sim", "px4_sim.launch.py", condition=IfCondition(start_px4)),

        # URDF / TF
        _include("drone_description", "description.launch.py",
                 args={"use_sim_time": use_sim_time}),

        # MAVROS bridge
        _include("drone_control", "mavros.launch.py",
                 condition=IfCondition(start_mavros),
                 args={"use_sim_time": use_sim_time}),

        # keyboard -> OFFBOARD velocity bridge (arms + takes off automatically)
        Node(
            package="drone_control", executable="offboard_keyboard", name="offboard_keyboard",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # RTAB-Map visual SLAM
        _include("drone_slam", "rtabmap.launch.py",
                 condition=IfCondition(start_slam),
                 args={"use_sim_time": use_sim_time}),

        # RViz
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            condition=IfCondition(start_rviz),
            arguments=["-d", rviz_cfg],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
