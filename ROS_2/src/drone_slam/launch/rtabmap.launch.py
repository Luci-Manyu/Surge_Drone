"""RTAB-Map RGB-D visual SLAM for the drone's depth camera.

Two nodes:
  rgbd_odometry : visual odometry from RGB-D  -> owns TF  odom -> base_link, publishes /odom
  rtabmap       : graph SLAM + map building    -> owns TF  map  -> odom

Camera topics (published by the Gazebo libgazebo_ros_camera depth plugin):
  /camera/image_raw            (rgb)
  /camera/camera_info
  /camera/depth/image_raw      (32FC1 metric depth, aligned to color)

Outputs consumed by RViz:
  /rtabmap/cloud_map  (3D point cloud map)
  /rtabmap/grid_map   (2D occupancy grid, projected from depth)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    delete_db = LaunchConfiguration("delete_db")
    rtabmap_viz = LaunchConfiguration("rtabmap_viz")

    # Topic remaps shared by both nodes.
    remaps = [
        ("rgb/image", "/camera/image_raw"),
        ("rgb/camera_info", "/camera/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
    ]

    # Common subscription/sync settings.
    common = {
        "frame_id": "base_link",
        "use_sim_time": use_sim_time,
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "approx_sync": True,          # tolerate tiny stamp differences between rgb & depth
        "approx_sync_max_interval": 0.02,
        # BEST_EFFORT (2) image subscriptions: rgbd_odometry/rtabmap process each frame slowly,
        # and as RELIABLE readers they made Gazebo's RELIABLE camera writer block on their ACKs
        # (writer cache filling), which stalled the PX4 lockstep. Best-effort = fire-and-forget,
        # so Gazebo never waits — correct QoS for high-rate sensor streams anyway.
        "qos": 2,
    }

    # Namespace both nodes under /rtabmap so their topics match the documented design
    # (/rtabmap/cloud_map, /rtabmap/grid_map) and what RViz subscribes to. Without this,
    # rtabmap publishes /cloud_map at the ROOT while RViz listens on /rtabmap/cloud_map, so
    # the map is never assembled (lazy publish, no real subscriber) nor displayed. odom/odom_info
    # stay matched between the two nodes since both share the namespace; the camera remaps below
    # are absolute (/camera/*) so they are unaffected; TF frame_ids are names, not topics.
    rgbd_odometry = Node(
        package="rtabmap_odom", executable="rgbd_odometry", name="rgbd_odometry",
        namespace="rtabmap",
        output="screen",
        parameters=[common | {
            "odom_frame_id": "odom",
            "publish_tf": True,             # rgbd_odometry OWNS odom -> base_link
            "wait_imu_to_init": False,
            "Odom/ResetCountdown": "1",     # auto-reset if VO is lost (feature-poor view)
            "Vis/MinInliers": "12",
        }],
        remappings=remaps,
    )

    rtabmap = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap",
        namespace="rtabmap",
        output="screen",
        parameters=[common | {
            "subscribe_odom_info": True,
            "publish_tf": True,             # rtabmap OWNS map -> odom
            "Rtabmap/DetectionRate": "1.0",
            "RGBD/NeighborLinkRefining": "true",
            "RGBD/ProximityBySpace": "true",
            "Reg/Force3DoF": "false",       # drone moves in full 3D, not planar
            "Reg/Strategy": "1",            # 0=Vis, 1=ICP, 2=Vis+ICP
            # ---- occupancy grid projected from the downward depth camera ----
            "Grid/FromDepth": "true",
            "Grid/3D": "true",
            "Grid/CellSize": "0.05",
            "Grid/RangeMax": "6.0",
            "Grid/RayTracing": "true",
            "Grid/MaxGroundHeight": "0.1",
            "Grid/MaxObstacleHeight": "2.0",
            "Grid/NormalsSegmentation": "true",
        }],
        remappings=remaps,
        # "-d" deletes the previous database on start for a clean map each run.
        arguments=[PythonExpression(["'-d' if '", delete_db, "' == 'true' else ''"])],
    )

    rtabmap_viz_node = Node(
        package="rtabmap_viz", executable="rtabmap_viz", name="rtabmap_viz",
        output="screen",
        condition=IfCondition(rtabmap_viz),
        parameters=[common],
        remappings=remaps,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("delete_db", default_value="true"),
        DeclareLaunchArgument("rtabmap_viz", default_value="false",
                              description="RTAB-Map's own GUI (separate from RViz)"),
        rgbd_odometry,
        rtabmap,
        rtabmap_viz_node,
    ])
