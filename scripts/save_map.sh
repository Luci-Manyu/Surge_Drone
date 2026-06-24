#!/usr/bin/env bash
#
# Export a built RTAB-Map SLAM map to portable files you can open anywhere
# (CloudCompare, MeshLab, RViz) and commit to the repo.
#
# RTAB-Map continuously writes the live map to a SQLite database while the
# `rtabmap` node runs (default: ~/.ros/rtabmap.db). This script turns that
# database into:
#   <name>_cloud.ply   assembled 3D point cloud (voxel-filtered, with normals)
#   <name>_mesh.ply    triangulated + colored surface mesh
#   <name>_poses.txt   optimized trajectory (TUM format: timestamp x y z qx qy qz qw)
#
# Usage:
#   scripts/save_map.sh                          # db=~/.ros/rtabmap.db, name=surge_map
#   scripts/save_map.sh /path/to/map.db          # custom database
#   scripts/save_map.sh /path/to/map.db myname   # custom database + output name
#
set -euo pipefail

DB="${1:-$HOME/.ros/rtabmap.db}"
NAME="${2:-surge_map}"
OUT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/maps"

if ! command -v rtabmap-export >/dev/null 2>&1; then
  echo "error: rtabmap-export not found. Install it with: sudo apt install ros-${ROS_DISTRO:-humble}-rtabmap-ros" >&2
  exit 1
fi
if [ ! -f "$DB" ]; then
  echo "error: database not found: $DB" >&2
  echo "       Run a mapping session first (see README), then re-run this script." >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
echo "==> Exporting map from $DB"
echo "    output dir: $OUT_DIR   name: $NAME"

# Point cloud (voxel 1 cm, normals computed and flipped toward camera viewpoints).
rtabmap-export --cloud --output "$NAME" --output_dir "$OUT_DIR" "$DB"
# Colored surface mesh.
rtabmap-export --mesh  --output "$NAME" --output_dir "$OUT_DIR" "$DB"
# Optimized trajectory (RGBD-SLAM / TUM-style format).
rtabmap-export --poses --poses_format 1 --output "$NAME" --output_dir "$OUT_DIR" "$DB"

# Render PNG previews (top-down true-color + height/contour). Optional: skipped if the
# cloud or python/numpy are unavailable; never fails the export.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if command -v python3 >/dev/null 2>&1 && python3 -c "import numpy" 2>/dev/null; then
  python3 "$HERE/render_map.py"     "$OUT_DIR/${NAME}_cloud.ply" "$OUT_DIR/${NAME}_topdown.png" 2>/dev/null \
    && echo "  rendered ${NAME}_topdown.png" || echo "  (top-down render skipped)"
  python3 "$HERE/render_contour.py" "$OUT_DIR/${NAME}_cloud.ply" "$OUT_DIR/${NAME}_contour.png" 2>/dev/null \
    && echo "  rendered ${NAME}_contour.png" || echo "  (contour render skipped)"
fi

echo
echo "==> Done. Files written:"
ls -lh "$OUT_DIR/${NAME}"_*.ply "$OUT_DIR/${NAME}_poses.txt" "$OUT_DIR/${NAME}"_*.png 2>/dev/null || true
