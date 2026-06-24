# Maps

Exported SLAM maps produced by RTAB-Map from the drone's downward depth camera
during **autonomous** flights in PX4 SITL + Gazebo. Two environments are included.

Each map ships as a point cloud (`*_cloud.ply`), a colored surface mesh
(`*_mesh.ply`), a trajectory (`*_poses.txt`, TUM `timestamp x y z qx qy qz qw`),
and two rendered previews (`*_topdown.png` true-color, `*_contour.png` height map).

### `surge_town_*` — custom `town_world`, systematic coverage flight  *(latest / best)*
~570k points over an ~18 × 21 m "town" of ~60 varied-height buildings, flown with
the **`coverage_path`** lawnmower. The **`*_contour.png`** colors every building by
height (cyan ≈0.3 m → red ≈1.1 m) on flat, level ground — the IMU-gravity leveling
removed the odometry tilt. This is the detailed, even map the coverage path + richer
world produce.

### `surge_map_*` — `slam_world`, `mapping_sweep` orbit
~718k points / 245 keyframes from ~83 m of flight in the original sparse world. Kept
as a second environment for comparison.

## How these are generated

While `rtabmap` runs it continuously writes the live map to a SQLite database
(default `~/.ros/rtabmap.db`; set with `database_path:=`). After a flight:

```bash
scripts/save_map.sh                       # ~/.ros/rtabmap.db -> maps/surge_map_*  (+ PNGs)
scripts/save_map.sh ~/.ros/surge_town.db surge_town   # custom db + output name
```

`save_map.sh` exports the cloud/mesh/poses and renders the two PNG previews
(`scripts/render_map.py`, `scripts/render_contour.py` — numpy only, no matplotlib).
The `*.db` (hundreds of MB) is **git-ignored**; only these light exports are committed.

## Notes

- Maps are in the `map` frame, Z up. A fresh run wipes the db by default
  (`delete_db:=true`); pass `delete_db:=false` to append across flights.
- The town cloud is exported with `rtabmap-export --opt 3` (assemble on raw
  odometry poses) to keep the **full** coverage in one piece. RTAB-Map's default
  global optimization currently only bridges the largest loop-closed sub-graph, so
  it would crop the map; `--opt 3` trades a little residual drift for complete extent.
- `*_contour.png` detrends the ground plane and shows **height above ground**, so
  object heights are readable straight from the depth data.
