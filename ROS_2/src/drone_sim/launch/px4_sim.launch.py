"""Start PX4 SITL + Gazebo Classic with the s500_depth model in slam_world.

This shells out to PX4's own launcher
(`make px4_sitl_nolockstep gazebo-classic_s500_depth__slam_world`), which renders models,
sets all GAZEBO_* paths, starts gzserver/gzclient, and launches the px4 binary that connects
to the model's gazebo_mavlink_interface. PX4 then exposes its offboard MAVLink API on
UDP 14540 for MAVROS.

LOCKSTEP: we use the `px4_sitl_nolockstep` build variant. PX4 then runs on the real monotonic
system clock instead of pacing off Gazebo sensor timestamps, which keeps the EKF height/velocity
estimate stable (constant IMU dt) so the vehicle can arm, and lets Gazebo free-run so the heavy
ROS 2 stack can't deadlock the lockstep handshake. The matching Gazebo flag (iris
<enable_lockstep>0) is set by drone_sim/px4/install_px4_assets.sh.

Prereqs (one-time):
  1. Build PX4 deps (needs sudo): cd ~/PX4-Autopilot && bash Tools/setup/ubuntu.sh
  2. Install our assets:          src/drone_sim/px4/install_px4_assets.sh
  3. ROS 2 sourced (ROS_VERSION=2) so the depth camera renders its ROS2 plugin.

The FIRST run compiles PX4 (slow). Later runs are fast.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    px4_dir = LaunchConfiguration("px4_dir")
    headless = LaunchConfiguration("headless")

    return LaunchDescription([
        DeclareLaunchArgument(
            "px4_dir",
            # Self-contained checkout under Surge/. Override with the PX4_DIR env var
            # or  ros2 launch ... px4_dir:=/some/path  if you keep PX4 elsewhere.
            default_value=os.environ.get(
                "PX4_DIR", os.path.expanduser("~/Surge/PX4-Autopilot")),
            description="Path to the PX4-Autopilot checkout."),
        DeclareLaunchArgument(
            "headless", default_value="0",
            description="1 = no Gazebo GUI (gzclient); faster, useful on a remote box."),

        SetEnvironmentVariable("PX4_SIM_MODEL", "gazebo-classic_s500_depth"),
        SetEnvironmentVariable("HEADLESS", headless),
        # Gazebo's ROS camera plugins must use the same RMW vendor as the rest of the
        # stack (CycloneDDS) or RTAB-Map can't see /camera/* topics. See ROS_2/source_env.sh.
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),

        ExecuteProcess(
            cmd=["make", "px4_sitl_nolockstep",
                 "gazebo-classic_s500_depth__slam_world"],
            cwd=px4_dir,
            output="screen",
            shell=False,
        ),
    ])
