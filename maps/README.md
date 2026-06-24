# Maps

Exported SLAM maps produced by RTAB-Map from the drone's downward depth camera
during an autonomous `mapping_sweep` flight in PX4 SITL + Gazebo.

| File | What it is |
|------|------------|
| `surge_map_cloud.ply` | Assembled 3D point cloud (voxel-filtered to 1 cm, with normals). Open in CloudCompare / MeshLab. |
| `surge_map_mesh.ply`  | Triangulated + colored surface mesh of the same map. |
| `surge_map_poses.txt` | Optimized flight trajectory, TUM format: `timestamp x y z qx qy qz qw`. |

**This map:** 245 optimized keyframe poses forming a single coherent graph,
~718k points, from ~83 m of autonomous flight. (An earlier run fragmented into
69 disconnected sub-maps because visual odometry kept losing the sparse ground;
the IMU fusion + feature/reset tuning in `drone_slam/launch/rtabmap.launch.py`
fixed that — the whole flight now resolves into one map.)

## How these are generated

While the `rtabmap` node runs it continuously writes the live map to a SQLite
database (default `~/.ros/rtabmap.db`). After a mapping flight, export it with:

```bash
scripts/save_map.sh                 # uses ~/.ros/rtabmap.db -> maps/surge_map_*
scripts/save_map.sh /path/to.db foo # custom database + output name
```

The database itself (`*.db`, often hundreds of MB) is **git-ignored** — only the
lightweight exports above are committed. Re-run `save_map.sh` to regenerate them.

## Notes

- The map is built in the `map` frame (RTAB-Map's optimized graph), Z up.
- A fresh run starts a clean map by default (`delete_db:=true`). Pass
  `delete_db:=false` to keep appending to an existing database across flights.
