#!/usr/bin/env bash
#
# Bridge + drift-correct a recorded RTAB-Map database.
#
# A live mapping flight can leave the graph split into disconnected sub-maps (visual
# odometry resets at startup/shutdown, or briefly mid-flight, each start a new map).
# RTAB-Map's export then globally optimizes only the largest connected component, so the
# map gets cropped — or you fall back to `--opt 3` (raw odometry) and keep drift.
#
# This re-runs RTAB-Map's loop-closure detection over the WHOLE database offline, more
# thoroughly than real time: all nodes kept in working memory so revisits can match, and
# VISUAL (not ICP) loop-closure verification — ICP is degenerate on the near-flat ground the
# downward camera sees and rejects every loop. The result is a single connected, loop-closed
# graph, so `rtabmap-export --opt 0` gives the FULL map, drift-corrected.
#
# Usage:
#   scripts/bridge_map.sh ~/.ros/surge_town.db                 # -> ~/.ros/surge_town_bridged.db
#   scripts/bridge_map.sh in.db out.db
#   scripts/save_map.sh   ~/.ros/surge_town_bridged.db surge_town   # then export the bridged db
#
set -euo pipefail

IN="${1:?usage: bridge_map.sh input.db [output.db]}"
OUT="${2:-${IN%.db}_bridged.db}"

command -v rtabmap-reprocess >/dev/null || {
  echo "error: rtabmap-reprocess not found (sudo apt install ros-${ROS_DISTRO:-humble}-rtabmap-ros)" >&2; exit 1; }
[ -f "$IN" ] || { echo "error: database not found: $IN" >&2; exit 1; }

echo "==> Bridging $IN -> $OUT"
rm -f "$OUT"
rtabmap-reprocess \
  --Rtabmap/MemoryThr 0 \
  --Rtabmap/TimeThr 0 \
  --Rtabmap/LoopThr 0.05 \
  --Reg/Strategy 0 \
  --Vis/MinInliers 6 \
  --Vis/MaxFeatures 1500 \
  --RGBD/LoopClosureReextractFeatures true \
  --RGBD/ProximityBySpace true \
  "$IN" "$OUT" 2>&1 | sed -e 's/\x1b\[[0-9;]*m//g' | grep -iE "Total loop closures|Processed 271|Processed [0-9]+/[0-9]+ nodes" | tail -2

echo
echo "==> Done. Now export the FULL drift-corrected map with:"
echo "    scripts/save_map.sh \"$OUT\" $(basename "${IN%.db}")"
