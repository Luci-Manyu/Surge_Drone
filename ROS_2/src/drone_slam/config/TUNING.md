# RTAB-Map tuning notes

The live parameters are set inline in `launch/rtabmap.launch.py` (the ROS2 rtabmap wrapper
takes its `Group/Param` settings most reliably as a Python dict, so they live there rather
than a YAML to avoid name-nesting ambiguity on the slashed keys).

Knobs you'll most likely touch:

| Param | Where | Effect |
|-------|-------|--------|
| `Grid/CellSize` | rtabmap | 2D/3D occupancy resolution (m). Smaller = finer, heavier. |
| `Grid/RangeMax` | rtabmap | Max depth range used for the grid (m). |
| `Grid/MaxGroundHeight` | rtabmap | Below this height = ground (for a downward cam). |
| `Vis/MinInliers` | rgbd_odometry | Lower if VO drops out over bland ground; raise if it drifts. |
| `Odom/ResetCountdown` | rgbd_odometry | Frames of lost tracking before auto-reset. |
| `Rtabmap/DetectionRate` | rtabmap | Loop-closure / map update rate (Hz). |

If visual odometry keeps failing over a feature-poor floor, add more texture/objects to
`drone_sim/worlds/slam_world.world`, or fall back to Gazebo ground-truth odom for debugging.
