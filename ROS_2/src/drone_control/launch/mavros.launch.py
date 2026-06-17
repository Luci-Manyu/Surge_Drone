"""MAVROS bridge to PX4 SITL.

PX4 SITL publishes its offboard/onboard MAVLink API on UDP 14540. MAVROS connects there and
exposes /mavros/* topics + services (state, arming, set_mode, setpoint_raw, local_position).

TF NOTE: MAVROS does NOT publish odom->base_link by default (the *.tf.send plugin params are
false). We keep it that way so RTAB-Map's rgbd_odometry is the sole owner of that TF edge.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    tgt_system = LaunchConfiguration("tgt_system")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        # PX4 SITL instance 0 onboard MAVLink: binds udp 14580, streams to 14540 (px4-rc.mavlink:
        # udp_offboard_port_local=14580, remote=14540). So MAVROS must LISTEN on 14540 and SEND to
        # PX4's listen port 14580. The old default sent to 14557 (nothing bound there) — inbound
        # heartbeats still arrived on 14540 so the link looked "connected", but outbound COMMAND_LONG
        # (arming, autopilot-version) went to the dead port and never reached PX4, so arming hung.
        DeclareLaunchArgument("fcu_url", default_value="udp://:14540@127.0.0.1:14580"),
        DeclareLaunchArgument("gcs_url", default_value=""),
        DeclareLaunchArgument("tgt_system", default_value="1"),
        # use_sim_time=FALSE: we run the px4_sitl_nolockstep build, so PX4 keeps its own real
        # monotonic system clock (it does NOT publish sim-time pacing). If MAVROS ran on sim time
        # it would stamp setpoints/commands in a clock that disagrees with PX4's, so PX4 sees the
        # offboard setpoints as stale -> offboard_control_signal_lost -> drops OFFBOARD -> can't
        # arm. Wall time keeps MAVROS, the teleop node, and nolockstep PX4 on one clock.
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(
            package="mavros",
            executable="mavros_node",
            # NO name= override. mavros_node spawns an internal UAS sub-node whose plugin topics
            # are RELATIVE to the node's fully-qualified name. Setting name="mavros" on top of
            # namespace="mavros" double-prefixes them to /mavros/mavros/* (e.g.
            # /mavros/mavros/setpoint_raw/local), so a publisher on /mavros/setpoint_raw/local
            # has NO subscriber -> setpoints never reach PX4 -> offboard_control_signal_lost ->
            # cannot arm. The official mavros launch sets only the namespace; match it so topics
            # resolve to /mavros/* as the rest of the stack expects.
            namespace="mavros",
            output="screen",
            parameters=[{
                "fcu_url": fcu_url,
                "gcs_url": gcs_url,
                "tgt_system": tgt_system,
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
                "use_sim_time": use_sim_time,
                # Frames (ROS side, ENU/FLU). Keep TF emission OFF; rtabmap owns odom->base_link.
                "local_position.frame_id": "odom",
                "local_position.tf.send": False,
                "global_position.tf.send": False,
                "imu.frame_id": "base_link",
            }],
        ),
    ])
