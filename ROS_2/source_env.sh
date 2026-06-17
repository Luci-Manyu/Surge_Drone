# Source this in EVERY terminal you use for the Surge SLAM-drone project:
#     source ~/Surge/ROS_2/source_env.sh
#
# It sources ROS 2 + this workspace and pins the DDS settings the stack needs.
#
# Why CycloneDDS: MAVROS 2 (Humble) crashes on the default rmw_fastrtps_cpp with
#   "could not create subscription: invalid allocator ... CompanionProcessStatus"
# (a fastrtps type-checking bug on the sys_status plugin, which also publishes
# /mavros/state, so it can't simply be disabled). CycloneDDS avoids it and is the
# RMW PX4/MAVROS officially recommend. Every node — Gazebo's ROS camera plugins,
# MAVROS, RTAB-Map, RViz — must share one RMW vendor to interoperate, so we set it
# globally here.
source /opt/ros/humble/setup.bash
_THIS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$_THIS/install/setup.bash" ] && source "$_THIS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
# Enlarge Cyclone's UDP receive buffer so Gazebo's depth frames don't block the writer and
# stall the PX4 lockstep. Needs net.core.rmem_max raised (setup.sh `buffers` stage); if it
# isn't, Cyclone aborts, so only apply the config when the kernel limit is high enough.
if [ "$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)" -ge 16777216 ] \
   && [ "$(cat /proc/sys/net/core/wmem_max 2>/dev/null || echo 0)" -ge 16777216 ]; then
  export CYCLONEDDS_URI="file://$_THIS/config/cyclonedds.xml"
fi
