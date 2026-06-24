# SLAM Drone — ROS 2 Simulation Workspace

Keyboard-fly an S500-class quadcopter in **PX4 SITL + Gazebo**, and build a live map of the
ground below it with **RTAB-Map visual SLAM** (Intel RealSense depth camera, tilted 15° down),
visualized in **RViz2**.

## Packages
| Package | Role |
|---------|------|
| `drone_description` | Simplified URDF + RViz config; publishes `base_link → camera_link → camera_depth_optical_frame` TF |
| `drone_sim` | PX4 SITL Gazebo model `s500_depth` (iris + 15°-down ROS2 depth camera), `slam_world`, sim launch, PX4 asset installer |
| `drone_control` | MAVROS bridge + `offboard_keyboard` (teleop `/cmd_vel` → PX4 OFFBOARD velocity, auto arm/takeoff) + `mapping_sweep` (hands-free autonomous mapping flight) |
| `drone_slam` | RTAB-Map `rgbd_odometry` + `rtabmap` visual SLAM launch/params (IMU fusion, anti-fragmentation tuning, configurable `database_path`) |
| `drone_bringup` | Top-level launch tying it all together |

## TF tree
```
map ─(rtabmap)→ odom ─(rgbd_odometry)→ base_link ─(rsp)→ camera_link ─(rsp)→ camera_depth_optical_frame
```
`rgbd_odometry` owns `odom→base_link`; `rtabmap` owns `map→odom`; MAVROS TF is disabled to
avoid a double publisher.

## One-time setup
The easiest path is `~/Surge/setup.sh` (runs every stage). Manually it is:
```bash
# 1. System deps (NEED SUDO). CycloneDDS is required — MAVROS crashes on fastrtps.
sudo apt update
sudo apt install -y ros-humble-rtabmap-ros ros-humble-mavros ros-humble-mavros-extras \
     ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins gazebo ros-humble-rmw-cyclonedds-cpp
sudo geographiclib-get-geoids egm96-5 || \
  sudo bash /opt/ros/humble/share/mavros/install_geographiclib_datasets.sh

# 2. Kernel UDP buffers (NEED SUDO) — CycloneDDS blocks on big camera frames otherwise:
sudo sysctl -w net.core.rmem_max=134217728 net.core.wmem_max=134217728

# 3. PX4 toolchain (NEED SUDO the first time):
cd ~/Surge/PX4-Autopilot && bash Tools/setup/ubuntu.sh    # then reboot/relogin

# 4. Link our sim assets into PX4, register the airframe, disable lockstep:
~/Surge/ROS_2/src/drone_sim/px4/install_px4_assets.sh

# 5. Build this workspace:
cd ~/Surge/ROS_2 && colcon build --symlink-install
```

## Run (3 terminals — `source source_env.sh` in EACH)
`source_env.sh` sources ROS 2 + this workspace **and** pins CycloneDDS + DDS buffer tuning.
Do NOT use the bare `install/setup.bash` — MAVROS will crash on the default RMW.
```bash
# A) PX4 SITL + Gazebo — headless (recommended on integrated GPUs):
source ~/Surge/ROS_2/source_env.sh
cd ~/Surge/PX4-Autopilot && HEADLESS=1 make px4_sitl_nolockstep gazebo-classic_s500_depth__slam_world

# B) keyboard teleop  — OR skip this and use mapping_sweep (autonomous, below):
source ~/Surge/ROS_2/source_env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# C) MAVROS + SLAM + RViz:
source ~/Surge/ROS_2/source_env.sh
ros2 launch drone_bringup slam_sim.launch.py
```
The drone auto-arms, takes off to ~2 m, then flies from the keyboard
(`i/,` fwd/back, `j/l` yaw, `t/b` up/down, shift to strafe). Fly around and the map grows in RViz.

**Autonomous mapping** (instead of B's teleop): `ros2 run drone_control mapping_sweep
--ros-args -p target_altitude:=1.8 -p forward_speed:=0.3 -p yaw_rate:=0.1` flies a hands-free
low orbit while RTAB-Map maps.

**Save the map** after a flight: `~/Surge/scripts/save_map.sh` exports the live RTAB-Map
database to `maps/surge_map_{cloud.ply,mesh.ply,poses.txt}`. Point it at any db:
`save_map.sh ~/.ros/surge_fresh2.db`.

## Verify
```bash
ros2 topic hz /camera/depth/image_raw     # camera streaming (needs a subscriber — lazy publish)
ros2 topic echo /mavros/state             # connected + armed + mode
ros2 run tf2_tools view_frames            # map→odom→base_link→camera_link→optical
ros2 topic hz /rtabmap/odom               # visual odometry tracking (~10 Hz = VO locked)
ros2 topic hz /rtabmap/cloud_map          # map growing
grep -c "poll timeout" /tmp/px4_sitl.log  # 0 = sim healthy; climbing = lockstep stalled
```
See the **Troubleshooting** table in `../README.md` for the bring-up gotchas (Gazebo plugins,
CycloneDDS, socket buffers, lockstep) and their permanent fixes.
