#!/usr/bin/env bash
#
# Link this repo's PX4 SITL assets (model, airframe, world) into a PX4-Autopilot checkout,
# and register the airframe so `make px4_sitl gazebo-classic_s500_depth` works.
#
# Source of truth stays in THIS repo (symlinks, not copies), so editing the model/world here
# updates the sim. Re-run safely; it is idempotent.
#
# Usage:   ./install_px4_assets.sh [/path/to/PX4-Autopilot]
# Default PX4 dir: $1, else $PX4_DIR, else <Surge>/PX4-Autopilot (self-contained checkout).
set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # .../drone_sim/px4
SIM_DIR="$(cd "$THIS_DIR/.." && pwd)"                              # .../drone_sim
SURGE_ROOT="$(cd "$THIS_DIR/../../../.." && pwd)"                  # .../Surge
PX4_DIR="${1:-${PX4_DIR:-$SURGE_ROOT/PX4-Autopilot}}"

GZ_ROOT="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
MODELS_DST="$GZ_ROOT/models"
WORLDS_DST="$GZ_ROOT/worlds"
AIRFRAMES_DST="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
AIRFRAME="1020_gazebo-classic_s500_depth"

if [ ! -d "$PX4_DIR" ]; then
  echo "ERROR: PX4-Autopilot not found at: $PX4_DIR" >&2
  echo "Pass the path explicitly:  $0 /path/to/PX4-Autopilot" >&2
  exit 1
fi
for d in "$MODELS_DST" "$WORLDS_DST" "$AIRFRAMES_DST"; do
  [ -d "$d" ] || { echo "ERROR: expected PX4 dir missing: $d" >&2; exit 1; }
done

echo "PX4-Autopilot : $PX4_DIR"
echo "Repo assets   : $SIM_DIR"

# 1) model  ---------------------------------------------------------------
ln -sfn "$SIM_DIR/models/s500_depth" "$MODELS_DST/s500_depth"
echo "  linked model    -> $MODELS_DST/s500_depth"

# 2) world  ---------------------------------------------------------------
ln -sfn "$SIM_DIR/worlds/slam_world.world" "$WORLDS_DST/slam_world.world"
echo "  linked world    -> $WORLDS_DST/slam_world.world"

# 3) airframe  ------------------------------------------------------------
ln -sfn "$THIS_DIR/airframes/$AIRFRAME" "$AIRFRAMES_DST/$AIRFRAME"
echo "  linked airframe -> $AIRFRAMES_DST/$AIRFRAME"

# 4) register airframe in CMakeLists (idempotent insert after the iris depth cam entries) --
CML="$AIRFRAMES_DST/CMakeLists.txt"
if grep -q "$AIRFRAME" "$CML"; then
  echo "  airframe already registered in CMakeLists.txt"
else
  # insert our entry right after the 1016 downward depth camera line, preserving tab indent.
  sed -i "/1016_gazebo-classic_iris_downward_depth_camera/a\\\t$AIRFRAME" "$CML"
  echo "  registered airframe in CMakeLists.txt"
fi

# 5) register model + world in the make-target generator (two hardcoded lists). The
#    `gazebo-classic_<model>__<world>` make target only exists if BOTH are listed here. --
TAB=$'\t'
SITL_TARGETS="$PX4_DIR/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
if [ ! -f "$SITL_TARGETS" ]; then
  echo "ERROR: sitl target generator not found: $SITL_TARGETS" >&2; exit 1
fi
if grep -q "s500_depth" "$SITL_TARGETS"; then
  echo "  model s500_depth already in target generator"
else
  sed -i "/^${TAB}${TAB}uuv_hippocampus$/a\\${TAB}${TAB}s500_depth" "$SITL_TARGETS"
  echo "  added model s500_depth to target generator"
fi
if grep -q "slam_world" "$SITL_TARGETS"; then
  echo "  world slam_world already in target generator"
else
  sed -i "/^${TAB}${TAB}yosemite$/a\\${TAB}${TAB}slam_world" "$SITL_TARGETS"
  echo "  added world slam_world to target generator"
fi

# 6) free-run Gazebo (enable_lockstep=0) to match the PX4 *nolockstep* build ----------------
#    There are TWO independent lockstep switches and they MUST agree:
#      (a) PX4 build flag ENABLE_LOCKSTEP_SCHEDULER  -> we build `px4_sitl_nolockstep`, so PX4
#          runs on the real monotonic system clock (NOT sim-time).
#      (b) Gazebo iris mavlink plugin <enable_lockstep> -> set 0 here so Gazebo free-runs and
#          never waits for a lockstep "step now" signal that a nolockstep PX4 never sends.
#    Why nolockstep at all: with lockstep ON, Gazebo's single update thread blocks on PX4 each
#    step; the heavy ROS 2 stack (MAVROS param burst + camera render/DDS) stalls that thread and
#    the handshake never recovers ("simulator_mavlink poll timeout", vehicle_command_ack lost).
#    With nolockstep, PX4 timestamps sensors with the real clock -> constant IMU dt -> the EKF
#    height/vertical-velocity estimate is stable and arming succeeds; and Gazebo free-running
#    self-heals from transient stalls. (The earlier BROKEN state was the asymmetric hybrid:
#    PX4 built WITH the lockstep scheduler but Gazebo free-running -> jittery PX4 clock -> the
#    "height estimate not stable" preflight failure.) On a capable GPU Gazebo holds real-time,
#    so the EKF stays happy. We edit the JINJA template (not the generated iris.sdf, which has
#    overwrite protection) and delete the generated copy so it regenerates on the next build.
IRIS="$GZ_ROOT/models/iris"
if [ -f "$IRIS/iris.sdf.jinja" ]; then
  if grep -q "enable_lockstep>1<" "$IRIS/iris.sdf.jinja"; then
    sed -i "s#enable_lockstep>1<#enable_lockstep>0<#" "$IRIS/iris.sdf.jinja"
    rm -f "$IRIS/iris.sdf" "$IRIS/iris.sdf.last_generated"   # force clean regen from jinja
    echo "  set Gazebo free-run (enable_lockstep=0) in iris.sdf.jinja (regenerates on next build)"
  else
    echo "  Gazebo already free-run (enable_lockstep=0) in iris.sdf.jinja"
  fi
fi

cat <<EOF

Done. Now build + launch the sim (ROS 2 must be sourced so ROS_VERSION=2):

  cd "$PX4_DIR"
  make px4_sitl gazebo-classic_s500_depth__slam_world

(First build is long. Subsequent launches are fast.)
EOF
