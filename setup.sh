#!/usr/bin/env bash
#
# One-shot environment setup for the Surge SLAM-drone project.
# Everything lives under this Surge directory so the project is self-contained and
# easy to upgrade/relocate later. Run from anywhere; paths are resolved from this script.
#
#   ./setup.sh            # do everything
#   ./setup.sh deps       # only apt/system deps (needs sudo)
#   ./setup.sh px4        # only PX4 toolchain + asset install
#   ./setup.sh ws         # only build the ROS 2 workspace
#
set -euo pipefail

SURGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$SURGE_ROOT/PX4-Autopilot"
WS_DIR="$SURGE_ROOT/ROS_2"
ROS_DISTRO="${ROS_DISTRO:-humble}"

log() { printf "\n\033[1;36m==> %s\033[0m\n" "$*"; }

install_deps() {
  log "Installing system / ROS 2 dependencies (sudo required)"
  sudo apt update
  sudo apt install -y \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-xacro
  log "Fetching MAVROS GeographicLib datasets"
  sudo geographiclib-get-geoids egm96-5 || \
    sudo bash /opt/ros/${ROS_DISTRO}/share/mavros/install_geographiclib_datasets.sh
}

setup_px4() {
  if [ ! -d "$PX4_DIR" ]; then
    log "Cloning PX4-Autopilot into Surge"
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b v1.15.4 "$PX4_DIR"
  fi
  log "Installing PX4 toolchain deps (sudo required; relogin afterwards)"
  bash "$PX4_DIR/Tools/setup/ubuntu.sh"
  log "Linking Surge sim assets (model/world/airframe) into PX4"
  "$WS_DIR/src/drone_sim/px4/install_px4_assets.sh" "$PX4_DIR"
}

ensure_gazebo_classic() {
  # PX4's Tools/setup/ubuntu.sh installs the NEW Gazebo (gz) and removes the Gazebo Classic
  # `gazebo` package. Because `ros-humble-gazebo-ros-pkgs` DEPENDS on `gazebo`, apt removes
  # the ROS<->Gazebo bridge plugins too (libgazebo_ros_camera/init/factory/state.so). Without
  # them the SITL camera publishes NO ROS 2 topics and there is no /clock. So we must restore
  # BOTH the classic binaries AND the ROS bridge packages. Run this AFTER setup_px4.
  if command -v gzserver >/dev/null 2>&1; then
    log "Gazebo Classic binaries present"
  else
    log "Restoring Gazebo Classic 11 binaries (removed by PX4 ubuntu.sh; sudo required)"
    sudo apt install -y gazebo
  fi
  if [ -f "/opt/ros/${ROS_DISTRO}/lib/libgazebo_ros_camera.so" ] 2>/dev/null \
     || dpkg -s "ros-${ROS_DISTRO}-gazebo-ros-pkgs" >/dev/null 2>&1; then
    log "gazebo_ros_pkgs (ROS<->Gazebo bridge plugins) present"
  else
    log "Restoring ROS<->Gazebo bridge plugins (removed with gazebo; sudo required)"
    sudo apt install -y "ros-${ROS_DISTRO}-gazebo-ros-pkgs" "ros-${ROS_DISTRO}-gazebo-plugins"
  fi
}

ensure_socket_buffers() {
  # CycloneDDS uses UDP on localhost; Gazebo's depth frames overflow the default ~200KB UDP
  # socket buffer and block the writer, stalling the PX4 lockstep. Raise the kernel limit and
  # persist it. (fastrtps used shared memory and avoided this, but it crashes MAVROS.)
  local want=134217728
  local r="$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)"
  local w="$(cat /proc/sys/net/core/wmem_max 2>/dev/null || echo 0)"
  if [ "$r" -ge "$want" ] && [ "$w" -ge "$want" ]; then
    log "net.core.{rmem,wmem}_max already >= ${want} bytes"
  else
    log "Raising net.core.rmem_max + wmem_max to ${want} (sudo required) and persisting it"
    sudo sysctl -w net.core.rmem_max=$want net.core.wmem_max=$want
    printf 'net.core.rmem_max=%s\nnet.core.wmem_max=%s\n' "$want" "$want" \
      | sudo tee /etc/sysctl.d/60-cyclonedds.conf >/dev/null
  fi
}

build_ws() {
  log "Building the ROS 2 workspace"
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  ( cd "$WS_DIR" && colcon build --symlink-install )
  log "Done. Source it with:  source $WS_DIR/install/setup.bash"
}

case "${1:-all}" in
  deps)   install_deps ;;
  px4)    setup_px4 ;;
  gazebo)  ensure_gazebo_classic ;;
  buffers) ensure_socket_buffers ;;
  ws)      build_ws ;;
  # NOTE: gazebo-classic is restored AFTER px4 setup, because PX4's ubuntu.sh removes it.
  all)    install_deps; setup_px4; ensure_gazebo_classic; ensure_socket_buffers; build_ws ;;
  *)      echo "usage: $0 [all|deps|px4|gazebo|buffers|ws]"; exit 1 ;;
esac
