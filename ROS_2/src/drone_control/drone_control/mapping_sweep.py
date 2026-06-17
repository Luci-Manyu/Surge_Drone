#!/usr/bin/env python3
"""Autonomous low-altitude sweep for the SLAM mapping demo (alternative to keyboard teleop).

`offboard_keyboard` auto-arms and climbs, then hands control to `/cmd_vel`. Keyboard teleop
(`teleop_twist_keyboard`) needs an interactive terminal; this node is a hands-free substitute
that publishes `/cmd_vel` so the drone flies a slow, low sweep while RTAB-Map builds the map.

Why a closed-loop altitude hold: PX4 OFFBOARD *velocity* control does not tightly hold altitude
(a zero vz setpoint still drifts), so this node runs a simple P-controller on altitude from
`/mavros/local_position/pose` and keeps the drone at `target_altitude` while sweeping. The
downward depth camera (~52 deg) only gets valid depth (and the ground stays inside the 10 m
range) at low altitude, so holding ~2.5 m is what keeps visual odometry tracking and the map
growing.

Run after the stack is up and the drone has taken off:
    ros2 run drone_control mapping_sweep
    ros2 run drone_control mapping_sweep --ros-args -p target_altitude:=3.0 -p forward_speed:=0.4
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped


class MappingSweep(Node):
    def __init__(self):
        super().__init__("mapping_sweep")

        self.declare_parameter("target_altitude", 2.5)   # m, where the depth cam sees the ground
        self.declare_parameter("forward_speed", 0.4)     # m/s body-forward while sweeping
        self.declare_parameter("yaw_rate", 0.12)         # rad/s gentle turn (wide arc, new ground)
        self.declare_parameter("alt_p_gain", 1.2)        # P gain on altitude error
        self.declare_parameter("max_z_vel", 1.0)         # m/s climb/descend clamp
        self.declare_parameter("settle_band", 0.4)       # m; only sweep once within this of target

        self.target = self.get_parameter("target_altitude").value
        self.fwd = self.get_parameter("forward_speed").value
        self.yaw = self.get_parameter("yaw_rate").value
        self.kp = self.get_parameter("alt_p_gain").value
        self.max_z = self.get_parameter("max_z_vel").value
        self.band = self.get_parameter("settle_band").value

        self.alt = None

        # MAVROS publishes local_position/pose RELIABLE+TRANSIENT_LOCAL; a best-effort sensor
        # subscription is compatible and fine for altitude feedback.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, sensor_qos)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self._loop)   # 10 Hz
        self.get_logger().info(
            f"mapping_sweep up: hold {self.target:.1f} m, fwd {self.fwd:.2f} m/s, "
            f"yaw {self.yaw:.2f} rad/s. Waiting for altitude...")

    def _on_pose(self, msg: PoseStamped):
        self.alt = msg.pose.position.z

    def _loop(self):
        if self.alt is None:
            return
        cmd = Twist()
        err = self.target - self.alt
        cmd.linear.z = max(-self.max_z, min(self.max_z, self.kp * err))
        if abs(err) < self.band:          # only translate once near target altitude
            cmd.linear.x = self.fwd
            cmd.angular.z = self.yaw
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MappingSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())         # leave a zero (hover) setpoint on exit
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
