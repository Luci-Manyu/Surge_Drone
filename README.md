# Surge — Autonomous SLAM Drone

A self-contained project for an autonomous **visual-SLAM quadcopter** (S500 frame, Pixhawk 6C
Mini, NVIDIA Jetson, Intel RealSense depth camera). The current milestone is a **ROS 2 +
PX4 SITL simulation**: keyboard-fly the drone in Gazebo and watch **RTAB-Map** build a live
map of the ground below it (camera tilted 15° down) in **RViz2**, using the depth camera for
**visual odometry**.

Everything — the ROS 2 workspace, the PX4 checkout, the CAD, and the setup scripts — lives
**inside this `Surge/` directory** so the project is easy to back up, relocate, and upgrade.

---

## Directory layout
```
Surge/
├── README.md                ← this file
├── setup.sh                 ← one-shot installer (deps + PX4 + workspace build)
├── scripts/                 ← save_map.sh (export) · gen_world.py (build worlds) · render_*.py (map PNGs)
├── maps/                    ← exported SLAM maps (cloud, mesh, trajectory, PNG previews) + README
├── CAD/                     ← Fusion 360 design (Drone_SLAM.f3z, S500-C1_ASM, Pixhawk 6C mini)
├── PX4-Autopilot/           ← PX4 flight stack (v1.15.4) — SITL firmware + Gazebo plugins
└── ROS_2/                   ← colcon workspace
    └── src/
        ├── drone_description/  URDF + RViz + TF (base_link → camera_link → optical)
        ├── drone_sim/          PX4 SITL model s500_depth; worlds: slam_world, town_world; asset installer
        ├── drone_control/      MAVROS + offboard_keyboard (teleop) · mapping_sweep · coverage_path (lawnmower)
        ├── drone_slam/         RTAB-Map rgbd_odometry + rtabmap (IMU fusion + gravity leveling, loop closure)
        └── drone_bringup/      top-level launch wiring it all together
```

---

## System / external dependencies
| Dependency | How it's provided | Notes |
|------------|-------------------|-------|
| Ubuntu 22.04 + ROS 2 Humble | system | base platform |
| Gazebo Classic 11 + `gazebo_ros_pkgs` | apt | simulator (matches PX4 `gazebo-classic` target) |
| `rtabmap_ros` | apt (`setup.sh deps`) | visual SLAM |
| `mavros` + `mavros_extras` + geoids | apt (`setup.sh deps`) | PX4 ↔ ROS 2 bridge |
| **`rmw_cyclonedds_cpp`** | apt (`setup.sh deps`) | **DDS middleware — MAVROS crashes on the default fastrtps** |
| `teleop_twist_keyboard`, `xacro`, Nav2 | apt | control / utils |
| Kernel UDP socket buffers raised | sysctl (`setup.sh buffers`) | CycloneDDS needs big buffers for camera frames |
| **PX4-Autopilot v1.15.4** | **git checkout inside `Surge/`** | SITL firmware + Gazebo model plugins |

> ROS packages (`rtabmap`, `mavros`, …) are installed system-wide via apt — that's how ROS 2
> distributes binaries. The one heavy, version-pinned dependency we keep **in-tree** is
> PX4-Autopilot, since that's what benefits most from being self-contained for upgrades.
>
> **Always source `ROS_2/source_env.sh`** in every terminal (not the raw `install/setup.bash`).
> It sources ROS 2 + the workspace **and** pins the settings the stack needs: CycloneDDS
> (`RMW_IMPLEMENTATION`), `ROS_DOMAIN_ID`, and the DDS buffer-tuning config. MAVROS crashes on
> the Humble-default fastrtps (`invalid allocator … CompanionProcessStatus`), so CycloneDDS is
> mandatory and every node must share it.

---

## Quick start
```bash
# From a fresh machine (ROS 2 Humble already installed):
git clone https://github.com/Luci-Manyu/Surge_Drone.git ~/Surge
cd ~/Surge
./setup.sh            # deps + PX4 toolchain + asset install + gazebo-classic + buffers + ws build
#   (or run stages individually:  ./setup.sh deps | px4 | gazebo | buffers | ws )
```
> **Note:** `PX4-Autopilot/` (~6.7 GB) and the colcon `build/ install/ log/` artifacts are
> **not** committed to this repo — `./setup.sh px4` clones PX4 @ `v1.15.4` and `./setup.sh ws`
> rebuilds the workspace. Our custom PX4 sim assets live in `ROS_2/src/drone_sim/px4/` and are
> linked into the PX4 tree by `install_px4_assets.sh`, so nothing custom is lost.
`setup.sh` runs a few **sudo** steps (apt, the socket-buffer sysctl). On a machine with no
passwordless sudo, run those stages yourself in a real terminal (see **Troubleshooting**).

Then run the simulation in **three terminals**. **Source `source_env.sh` in each** (it pins
CycloneDDS + buffers — the plain `install/setup.bash` is not enough):
```bash
# Terminal A — PX4 SITL + Gazebo (first run compiles PX4 — slow; later runs are fast).
#   HEADLESS=1 is recommended: on integrated GPUs the Gazebo GUI + depth-camera render
#   starves the sim. Visualize in RViz instead (Terminal C).
source ~/Surge/ROS_2/source_env.sh
cd ~/Surge/PX4-Autopilot && HEADLESS=1 make px4_sitl_nolockstep gazebo-classic_s500_depth__slam_world

# Terminal B — keyboard teleop (needs an interactive terminal)
source ~/Surge/ROS_2/source_env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal C — MAVROS + RTAB-Map + RViz
source ~/Surge/ROS_2/source_env.sh
ros2 launch drone_bringup slam_sim.launch.py
```
The drone auto-arms, climbs to ~2 m, then flies from the keyboard. Fly around → the 3D cloud
and 2D occupancy grid grow live in RViz.

**Teleop keys:** `i`/`,` forward/back · `j`/`l` yaw · `t`/`b` up/down · hold **Shift** to strafe · `k` stop.

> **Tip — start PX4 from the same bringup.** `slam_sim.launch.py start_px4:=true` will also
> launch the PX4 sim, but keeping it in its own terminal is usually clearer for debugging.

### Autonomous mapping (no keyboard)
Instead of Terminal B's teleop, fly hands-free. Two options:

```bash
# (a) systematic COVERAGE — lawnmower over a rectangle + return to start (best, even maps):
ros2 run drone_control coverage_path \
  --ros-args -p x_min:=-7.0 -p x_max:=7.0 -p y_min:=-7.0 -p y_max:=7.0 \
             -p lane_spacing:=3.0 -p altitude:=2.5 -p cruise_speed:=0.7

# (b) simple sweep — slow low orbit:
ros2 run drone_control mapping_sweep \
  --ros-args -p target_altitude:=1.8 -p forward_speed:=0.3 -p yaw_rate:=0.1
```
Both publish `/cmd_vel`, which `offboard_keyboard` turns into PX4 OFFBOARD velocity, and hold
altitude with a P-controller (the downward depth cam needs the ground in view). **`coverage_path`**
flies a boustrophedon path so the camera swaths overlap evenly — complete maps, and the lane
overlap + return-to-start let RTAB-Map close loops and correct drift.

### Different environments
The drone drops into any Gazebo-classic world by name — the make target is
`gazebo-classic_s500_depth__<world>`:
```bash
cd ~/Surge/PX4-Autopilot
HEADLESS=1 make px4_sitl_nolockstep gazebo-classic_s500_depth__town_world   # detailed custom town
HEADLESS=1 make px4_sitl_nolockstep gazebo-classic_s500_depth__slam_world   # original sparse world
```
- **`town_world`** — a structured "town": ~60 varied-height/colored buildings on streets, corner
  landmarks, and a dense feature field. Generated by **`scripts/gen_world.py`** (edit the
  generator, not the `.world`). Built for detailed coverage mapping.
- **`slam_world`** — the original minimal world.
- New worlds: drop a `.world` in `drone_sim/worlds/` and re-run `install_px4_assets.sh` — it
  links every world into PX4 and registers the make target automatically.

### Generate & save a map
While `rtabmap` runs it continuously writes the live map to a SQLite database
(`~/.ros/rtabmap.db` by default; override with `database_path:=…`). After a flight, export it
to portable files:
```bash
scripts/save_map.sh                          # ~/.ros/rtabmap.db  -> maps/surge_map_*
scripts/save_map.sh ~/.ros/surge_fresh2.db   # any database
```
This writes the point cloud (`…_cloud.ply`), surface mesh (`…_mesh.ply`), trajectory
(`…_poses.txt`), and two **PNG previews** — a true-color top-down (`…_topdown.png`) and a
**height/contour map** (`…_contour.png`, colors = height above ground, so object heights read
straight off the depth data). The renders use `scripts/render_map.py` / `render_contour.py`
(numpy only). Sample maps for **two environments** ship in `maps/` (`surge_town_*`,
`surge_map_*`). The multi-hundred-MB `.db` is git-ignored; the light exports are committed.
See [`maps/README.md`](maps/README.md).

---

## Architecture

### TF tree (the spine)
```
map ──(rtabmap)──► odom ──(rgbd_odometry)──► base_link ──(rsp)──► camera_link ──(rsp)──► camera_depth_optical_frame
```
- `rgbd_odometry` owns `odom → base_link` (visual odometry from the depth camera).
- `rtabmap` owns `map → odom` (graph SLAM + loop closure).
- **MAVROS TF for `odom→base_link` is disabled** so there's a single publisher per edge.
- Camera frames mirror the real Intel RealSense driver, so the same SLAM config will work on
  hardware later.

### Key design decisions
- **PX4 SITL flight stack** (realistic flight, sim GPS) over a kinematic shortcut.
- **Visual odometry** drives the SLAM (true camera-based mapping).
- **Control and SLAM are decoupled** for this milestone: PX4 flies on simulated GPS; RTAB-Map
  runs alongside purely to map/visualize. (Fusing VO into PX4's EKF for GPS-denied flight is a
  later phase.)
- **Gazebo Classic 11** (matches the installed simulator; PX4 `gazebo-classic` target).
- **Simplified primitive geometry** for the body. The Fusion `.f3z` holds only Autodesk BREP
  (no mesh Gazebo can load); real meshes are an optional later drop-in via Fusion → STL export.
- The custom `s500_depth` Gazebo model **reuses PX4's `iris`** (rotors, mavlink, IMU/GPS/baro/mag)
  and adds a ROS2 `libgazebo_ros_camera` depth sensor on a boom at **15° down**. Our sim assets
  are **symlinked** into the PX4 tree by `drone_sim/px4/install_px4_assets.sh`, so this repo
  stays the source of truth.
- **Lockstep is disabled — the correct way: build `px4_sitl_nolockstep`.** There are *two*
  lockstep switches and they must agree: PX4's compile-time `ENABLE_LOCKSTEP_SCHEDULER` (set by
  the board variant) and Gazebo's iris `<enable_lockstep>` flag. We use the stock
  `px4_sitl_nolockstep` board (so PX4 runs on the **real monotonic system clock**) and set
  Gazebo `enable_lockstep=0` (free-run) to match. This fixes two problems at once:
    1. **EKF stable / vehicle arms.** With lockstep ON, Gazebo's single update thread blocks on
       PX4 each step; the heavy ROS 2 stack stalls that thread and the handshake never recovers
       (endless `simulator_mavlink poll timeout`). The previous *broken* state was the asymmetric
       hybrid — PX4 still built **with** the lockstep scheduler but Gazebo free-running — which
       jittered PX4's clock and produced `Preflight Fail: height estimate not stable` so the
       vehicle couldn't arm. nolockstep gives PX4 a constant IMU `dt`, so the EKF height/velocity
       estimate is stable.
    2. **No sim stall.** Free-running Gazebo self-heals from transient slowdowns; on a capable GPU
       it holds real-time, which is all the EKF needs.
  `install_px4_assets.sh` sets `enable_lockstep=0` in `iris.sdf.jinja`; the launch + Quick-start
  use the `px4_sitl_nolockstep` make target.
- **Depth camera is QVGA @ 10 Hz** and the SLAM nodes subscribe **best-effort** — high-rate
  RELIABLE camera readers made Gazebo's writer block, another sim staller. QVGA is ample for
  mapping.
- **SLAM robustness (anti-fragmentation).** The downward camera over the sim world's sparse
  ground often saw too few features, so visual odometry kept resetting and `rtabmap` started a
  **new map** each time — one autonomous flight came out as **69 disconnected sub-maps**.
  Fixes in `drone_slam/launch/rtabmap.launch.py`: **fuse the IMU** (`/mavros/imu/data`, keeps
  roll/pitch observable and rides through motion blur), pull **more/cheaper features**
  (`Vis/MaxFeatures`, lower `GFTT/QualityLevel`/`MinDistance`, `Vis/MinInliers` 12→8), and
  **ride out brief dropouts** (`Odom/ResetCountdown` 1→5) before declaring VO lost. Result: the
  same flight now resolves into **one coherent map (~245 keyframes)**.
- **Gravity leveling (drift fix).** Raw maps tilted ~9° because monocular-ish VO drifts in roll/
  pitch. `rtabmap` now also takes the IMU (`imu→/mavros/imu/data`) with `Optimizer/GravitySigma`,
  adding a gravity prior to every node so the optimized map stays level — visible as flat ground
  in the height/contour renders. Spatial loop closure (`RGBD/ProximityBySpace`) over the
  overlapping coverage lanes links revisited ground to bound horizontal drift. Grid resolution
  was also tightened (`Grid/CellSize` 0.05→0.03) for a more detailed map.

---

## Status

### Done
- [x] Self-contained workspace + PX4 checkout under `Surge/`
- [x] 5-package ROS 2 workspace — **builds clean** (`colcon build` ✓)
- [x] URDF/TF model with correct optical frames (RealSense-compatible naming)
- [x] PX4 SITL `s500_depth` Gazebo model (iris + 15°-down ROS2 depth camera) + airframe + world
- [x] MAVROS launch + `offboard_keyboard` auto arm/takeoff/velocity node
- [x] RTAB-Map visual-SLAM launch (rgbd_odometry + rtabmap, 3D cloud + 2D grid)
- [x] RViz config + single top-level bringup
- [x] `setup.sh` reproducible installer + per-package READMEs

### Done (continued)
- [x] `./setup.sh deps` — rtabmap/mavros/geoids + **CycloneDDS** installed
- [x] `./setup.sh px4` — PX4 toolchain + asset link + **PX4 SITL compiles** (`bin/px4` + Gazebo
      plugins; `gazebo-classic_s500_depth__slam_world` target exists; lockstep disabled)
- [x] `gazebo_ros_pkgs` restored, **kernel UDP buffers raised** (`setup.sh buffers`)
- [x] **Sim runs end-to-end headless**: Gazebo + PX4 stable (no lockstep stalls), camera streams
      (QVGA @ ~12 Hz), `/clock` advances, MAVROS connects and **holds**, OFFBOARD engages
- [x] Full stack (MAVROS + offboard + RTAB-Map) runs together without stalling the sim

### Done (continued)
- [x] **EKF `height estimate not stable` FIXED** — root cause was the asymmetric lockstep hybrid
      (PX4 built with the lockstep scheduler, Gazebo free-running). Switched to the
      `px4_sitl_nolockstep` build + Gazebo `enable_lockstep=0`. PX4 now reaches **`Ready for
      takeoff!`** with **zero** `height estimate not stable` warnings.
- [x] **Sim stall FIXED** — sustained **0** `poll timeout` across multi-minute runs; `/clock`
      advances; Gazebo + PX4 stable under the full ROS 2 load. (Confirmed the lockstep-ON
      alternative *does* stall: 337 poll timeouts the instant MAVROS connects.)
- [x] **OFFBOARD mode engages** through MAVROS (`mode: OFFBOARD`).
- [x] **Arming + auto-takeoff WORK** — drone arms, climbs to ~2 m, and hands off to keyboard
      control (`Armed by external command` / `Takeoff detected`; local z ≈ 1.93 m). The blocker
      was a MAVROS launch **double-namespace** bug: `name="mavros"` on top of `namespace="mavros"`
      pushed the UAS plugin topics to `/mavros/mavros/setpoint_raw/local`, while the teleop node
      published to `/mavros/setpoint_raw/local` (0 subscribers) — so offboard setpoints never
      reached PX4 (`offboard_control_signal_lost`) and the OFFBOARD arming check failed. Fix:
      drop the `name=` override (match the official mavros launch). Also corrected `fcu_url` to
      PX4's real onboard port `14580` and set MAVROS `use_sim_time=false` (nolockstep PX4 is on
      the real clock). The "autopilot version service timeout" warning is benign and unrelated.

### Done (continued)
- [x] **RTAB-Map builds a live map end-to-end** — confirmed the 3D cloud grows while flying
      (`/rtabmap/cloud_map`, `/rtabmap/grid_map`).
- [x] **Autonomous mapping flight** — `mapping_sweep` node (altitude-hold P-controller + slow
      orbit) flies hands-free, no teleop, while RTAB-Map maps.
- [x] **IMU fusion + anti-fragmentation tuning** — VO no longer shatters the map over sparse
      ground; an autonomous flight now resolves into **one coherent ~245-keyframe map** (was 69
      fragments). See *SLAM robustness* above.
- [x] **Map export pipeline** — persistent/configurable `database_path`, `scripts/save_map.sh`,
      and committed sample maps in `maps/` (cloud + mesh + trajectory + PNG previews).
- [x] **Systematic coverage flight** — `coverage_path` flies a lawnmower + return-to-start for
      complete, even maps (replaces the open-loop orbit).
- [x] **Detailed multi-environment maps** — custom `town_world` (`scripts/gen_world.py`), IMU
      **gravity leveling** (flat ground, drift fixed), finer grid; height/**contour** renders show
      per-building heights. Validated in `town_world` (~570k-point map) and `slam_world`.

### Next phases (planned)
- [ ] Frontier-based **autonomous exploration** (goal-driven, replaces the fixed coverage box)
- [ ] **Obstacle avoidance** using the live occupancy grid / depth
- [ ] Fix the export crop: get RTAB-Map's global optimization to bridge all sub-graphs so the
      drift-corrected map keeps full extent (currently use `--opt 3` for complete coverage)
- [ ] Fuse visual odometry into PX4 EKF2-vision for GPS-denied flight
- [ ] Nav2 autonomous waypoint navigation on the RTAB-Map costmap
- [ ] Real hardware bring-up (RealSense + Jetson + Pixhawk) — topics/frames already match
- [ ] High-fidelity CAD meshes from Fusion 360

---

## Verify a run
```bash
source ~/Surge/ROS_2/source_env.sh
ros2 topic hz /camera/depth/image_raw     # camera streaming (needs a subscriber — lazy publish)
ros2 topic echo /mavros/state             # connected + armed + mode
ros2 run tf2_tools view_frames            # map→odom→base_link→camera_link→optical
ros2 topic hz /rtabmap/cloud_map          # map growing while you fly
grep -c "poll timeout" /tmp/px4_sitl.log  # should stay 0 (climbing = sim stalled)
```

---

## Troubleshooting (hard-won)

These are the bring-up issues we hit and fixed; each has a permanent fix in `setup.sh`,
`source_env.sh`, the model, or `install_px4_assets.sh`.

| Symptom | Root cause | Fix |
|---|---|---|
| No `/camera/*` or `/clock` topics | PX4's `ubuntu.sh` removed `gazebo`, which cascaded and removed `ros-humble-gazebo-ros-pkgs` (the ROS↔Gazebo plugins) | `sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins` (`setup.sh gazebo`) |
| Need Gazebo Classic binaries | `ubuntu.sh` installs new `gz` and removes classic `gzserver/gzclient` | `sudo apt install -y gazebo` (`setup.sh gazebo`) |
| MAVROS crashes: `invalid allocator … CompanionProcessStatus` | fastrtps bug on the `sys_status` plugin (can't be disabled — it publishes `/mavros/state`) | use **CycloneDDS** (`source_env.sh` sets `RMW_IMPLEMENTATION`) |
| Sim stalls when GUI is shown | integrated GPU can't render GUI + depth camera in real time | run **headless** (`HEADLESS=1`); visualize in RViz |
| 100+ MB `px4_sitl` log, CPU spin | PX4's `pxh>` shell spins on EOF when stdin is closed | keep stdin open: `sleep infinity \| make px4_sitl …` |
| gzserver core-dumps on start | CycloneDDS `SocketReceiveBufferSize min` exceeded kernel `rmem_max` | raise buffers first (`setup.sh buffers`); config only applied when limits are high enough |
| Camera frames stall the sim | CycloneDDS UDP send/recv buffers too small for ~300 KB depth frames → writer blocks Gazebo | `sysctl net.core.rmem_max=net.core.wmem_max=128M` (`setup.sh buffers`) + best-effort camera QoS |
| MAVROS heartbeat drops, endless `poll timeout`, `vehicle_command_ack lost` | PX4 **lockstep** (true, both sides ON): Gazebo's update thread blocks on PX4; the heavy ROS 2 stack stalls it and the handshake never recovers | build **`px4_sitl_nolockstep`** (PX4 on real clock) **and** Gazebo `enable_lockstep=0` — both done/patched by `install_px4_assets.sh` + the launch target |
| `Preflight Fail: height estimate not stable`, can't arm | asymmetric hybrid: PX4 built **with** the lockstep scheduler but Gazebo free-running → jittery PX4 clock → inconsistent IMU `dt` | use the **`px4_sitl_nolockstep`** build so PX4 uses the real system clock (constant `dt`); don't run the default `px4_sitl` with `enable_lockstep=0` |
| Build error: `generation would overwrite changes to iris.sdf` | edited the generated `iris.sdf` directly | edit `iris.sdf.jinja` and delete `iris.sdf` so it regenerates |
| Repeated `poll timeout` climbing | the sim genuinely stalled (vs. a one-time startup transient that freezes at a constant count) | check `/clock` is advancing; restart the sim — lockstep desync doesn't self-recover |

> **No passwordless sudo?** Run the sudo bits yourself in a real terminal, then re-run the rest:
> ```bash
> sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins gazebo \
>      ros-humble-rmw-cyclonedds-cpp
> sudo sysctl -w net.core.rmem_max=134217728 net.core.wmem_max=134217728
> ```

See `ROS_2/README.md` for package-level detail and `ROS_2/src/drone_slam/config/TUNING.md`
for SLAM tuning knobs.
