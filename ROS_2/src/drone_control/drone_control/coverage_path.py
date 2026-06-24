#!/usr/bin/env python3
"""Autonomous *systematic* coverage flight (lawnmower) for complete, even SLAM maps.

`mapping_sweep` flies an open-loop orbit, so coverage is uneven and the map fragments.
This node instead flies a **boustrophedon (lawnmower) path** over a bounded rectangle at a
fixed altitude, then returns to the start — giving overlapping camera swaths (dense, even
map) and a revisit of the origin (helps RTAB-Map close a loop and correct drift).

Pipeline (same as mapping_sweep): publishes `/cmd_vel`, which `offboard_keyboard` turns into
PX4 OFFBOARD body-velocity. We hold yaw fixed and rotate the desired *world*-frame velocity
into the body frame using the drone's measured yaw, so the path is correct regardless of the
takeoff heading. A P-controller holds altitude (the downward depth cam needs to see the ground).

Run after the stack is up and the drone has taken off:
    ros2 run drone_control coverage_path
    ros2 run drone_control coverage_path --ros-args -p x_max:=10.0 -p lane_spacing:=2.5
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped


def _yaw_from_quat(q):
    """Z-yaw from a geometry_msgs quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class CoveragePath(Node):
    def __init__(self):
        super().__init__("coverage_path")

        # ---- coverage rectangle + flight params ----
        self.declare_parameter("x_min", -8.0)
        self.declare_parameter("x_max", 8.0)
        self.declare_parameter("y_min", -8.0)
        self.declare_parameter("y_max", 8.0)
        self.declare_parameter("lane_spacing", 3.0)     # m between lawnmower lanes
        self.declare_parameter("altitude", 2.5)         # m AGL to hold
        self.declare_parameter("cruise_speed", 0.6)     # m/s horizontal clamp
        self.declare_parameter("wp_tol", 0.45)          # m radius to count a waypoint reached
        self.declare_parameter("xy_p_gain", 0.8)        # P gain on horizontal position error
        self.declare_parameter("alt_p_gain", 1.2)       # P gain on altitude error
        self.declare_parameter("max_z_vel", 1.0)
        self.declare_parameter("settle_band", 0.4)      # only translate once near target altitude
        self.declare_parameter("return_to_start", True) # final leg back to origin (loop closure)

        g = self.get_parameter
        self.alt = g("altitude").value
        self.cruise = g("cruise_speed").value
        self.tol = g("wp_tol").value
        self.kxy = g("xy_p_gain").value
        self.kz = g("alt_p_gain").value
        self.max_z = g("max_z_vel").value
        self.band = g("settle_band").value

        self.waypoints = self._lawnmower(
            g("x_min").value, g("x_max").value, g("y_min").value, g("y_max").value,
            g("lane_spacing").value, g("return_to_start").value)
        self.idx = 0
        self.pose = None

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, sensor_qos)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self._loop)   # 10 Hz
        self.get_logger().info(
            f"coverage_path up: {len(self.waypoints)} waypoints, hold {self.alt:.1f} m, "
            f"cruise {self.cruise:.2f} m/s. Waiting for pose...")

    @staticmethod
    def _lawnmower(x0, x1, y0, y1, spacing, return_start):
        """Boustrophedon waypoints over [x0,x1]x[y0,y1]; snake along x, step in y."""
        lanes = []
        y = y0
        flip = False
        while y <= y1 + 1e-6:
            lanes.append((x1, y) if flip else (x0, y))
            lanes.append((x0, y) if flip else (x1, y))
            flip = not flip
            y += spacing
        if return_start:
            lanes.append((0.0, 0.0))
        return lanes

    def _on_pose(self, msg: PoseStamped):
        self.pose = msg.pose

    def _loop(self):
        if self.pose is None:
            return
        cmd = Twist()
        z = self.pose.position.z
        err_z = self.alt - z
        cmd.linear.z = max(-self.max_z, min(self.max_z, self.kz * err_z))

        # Only translate once altitude has settled (depth cam needs the ground in view).
        if abs(err_z) < self.band and self.idx < len(self.waypoints):
            tx, ty = self.waypoints[self.idx]
            dx, dy = tx - self.pose.position.x, ty - self.pose.position.y
            dist = math.hypot(dx, dy)
            if dist < self.tol:                       # reached -> next waypoint
                self.idx += 1
                if self.idx >= len(self.waypoints):
                    self.get_logger().info("Coverage complete — holding position.")
            else:
                # world-frame desired velocity (P, clamped to cruise), then -> body frame.
                v = min(self.cruise, self.kxy * dist)
                vx_w, vy_w = v * dx / dist, v * dy / dist
                yaw = _yaw_from_quat(self.pose.orientation)
                c, s = math.cos(yaw), math.sin(yaw)
                cmd.linear.x = c * vx_w + s * vy_w     # rotate world -> body (yaw held fixed)
                cmd.linear.y = -s * vx_w + c * vy_w
                if self.idx and self.idx % 2 == 0:
                    self.get_logger().info(
                        f"waypoint {self.idx}/{len(self.waypoints)} "
                        f"-> ({tx:.1f},{ty:.1f})  dist {dist:.1f} m", throttle_duration_sec=3.0)
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())     # leave a hover setpoint
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
