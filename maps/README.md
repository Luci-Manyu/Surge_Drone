# Maps

Exported SLAM maps produced by RTAB-Map from the drone's downward depth camera
during **autonomous** flights in PX4 SITL + Gazebo. Two environments are included.

Each map ships as a point cloud (`*_cloud.ply`), a colored surface mesh
(`*_mesh.ply`), a trajectory (`*_poses.txt`, TUM `timestamp x y z qx qy qz qw`),
and two rendered previews (`*_topdown.png` true-color, `*_contour.png` height map).

### `surge_town_*` — custom `town_world`, systematic coverage flight  *(latest / best)*
~435k points over an ~18 × 20 m "town" of ~60 varied-height buildings, flown with the
**`coverage_path`** lawnmower. The **`*_contour.png`** colors every building by height
(cyan ≈0.3 m → red ≈1.1 m) on flat, level ground. With the visual loop-closure fix this
flight stayed a **single connected map online** (203 poses, 277 optimized links) — no
offline bridging needed — and the residual vertical drift is only ~1.8 m (was ~9.6 m raw).
A **full-extent, drift-corrected** map straight from the live run.

### `surge_map_*` — `slam_world`, `mapping_sweep` orbit
~718k points / 245 keyframes from ~83 m of flight in the original sparse world. Kept
as a second environment for comparison.

## How these are generated

While `rtabmap` runs it continuously writes the live map to a SQLite database
(default `~/.ros/rtabmap.db`; set with `database_path:=`). After a flight:

```bash
# full-extent, drift-corrected: bridge the sub-graphs (offline loop closure) then export
scripts/bridge_map.sh ~/.ros/surge_town.db                 # -> ~/.ros/surge_town_bridged.db
scripts/save_map.sh   ~/.ros/surge_town_bridged.db surge_town

scripts/save_map.sh                                        # quick export of ~/.ros/rtabmap.db
```

`save_map.sh` exports the cloud/mesh/poses and renders the two PNG previews
(`scripts/render_map.py`, `scripts/render_contour.py` — numpy only, no matplotlib).
The `*.db` (hundreds of MB) is **git-ignored**; only these light exports are committed.

## Notes

- Maps are in the `map` frame, Z up. A fresh run wipes the db by default
  (`delete_db:=true`); pass `delete_db:=false` to append across flights.
- The town cloud is exported with full global optimization (`--opt 0`) from the
  **bridged** database (`scripts/bridge_map.sh`), so it is both full-extent and
  drift-corrected. (Without bridging, RTAB-Map optimizes only the largest connected
  sub-graph and crops the map — hence the bridge step re-detects loop closures across
  the flight's sub-maps using visual, not ICP, verification.)
- `*_contour.png` detrends the ground plane and shows **height above ground**, so
  object heights are readable straight from the depth data.
