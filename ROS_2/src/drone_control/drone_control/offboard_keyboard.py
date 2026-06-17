#!/usr/bin/env python3
"""Bridge teleop_twist_keyboard -> PX4 OFFBOARD velocity control via MAVROS.

Pipeline:
    teleop_twist_keyboard  --(/cmd_vel, geometry_msgs/Twist)-->  this node
    this node  --(/mavros/setpoint_raw/local, PositionTarget @ rate)-->  PX4

Flight sequence (automatic):
    1. wait for FCU connection (/mavros/state.connected)
    2. stream zero-velocity setpoints (PX4 requires a stream BEFORE OFFBOARD)
    3. switch to OFFBOARD + ARM
    4. climb to `takeoff_altitude`
    5. hand control to /cmd_vel (keyboard)

Body-frame mapping (FRAME_BODY_NED on the wire; FLU on the ROS side via MAVROS):
    cmd_vel.linear.x  -> forward (+)        teleop: i / ,
    cmd_vel.linear.y  -> left    (+)        teleop holonomic (shift): J / L
    cmd_vel.linear.z  -> up      (+)        teleop: t / b
    cmd_vel.angular.z -> yaw CCW (+)        teleop: j / l
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode


# PositionTarget: use velocity (vx,vy,vz) + yaw_rate, ignore everything else.
_IGNORE_POS = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
_IGNORE_ACC = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
TYPE_MASK_VEL_YAWRATE = _IGNORE_POS | _IGNORE_ACC | PositionTarget.IGNORE_YAW


class OffboardKeyboard(Node):
    def __init__(self):
        super().__init__("offboard_keyboard")

        # ---- params ----
        self.declare_parameter("takeoff_altitude", 2.0)
        self.declare_parameter("setpoint_rate", 20.0)     # Hz, must be > 2 for OFFBOARD
        self.declare_parameter("max_xy_vel", 1.5)         # m/s clamp
        self.declare_parameter("max_z_vel", 1.0)          # m/s clamp
        self.declare_parameter("max_yaw_rate", 1.0)       # rad/s clamp
        self.declare_parameter("auto_arm", True)

        self.takeoff_alt = self.get_parameter("takeoff_altitude").value
        self.rate_hz = self.get_parameter("setpoint_rate").value
        self.max_xy = self.get_parameter("max_xy_vel").value
        self.max_z = self.get_parameter("max_z_vel").value
        self.max_yaw = self.get_parameter("max_yaw_rate").value
        self.auto_arm = self.get_parameter("auto_arm").value

        # ---- state ----
        self.state = State()
        self.cur_alt = 0.0
        self.cmd = Twist()
        self.phase = "wait_fcu"     # wait_fcu -> stream -> takeoff -> fly
        self._last_req = self.get_clock().now()

        # MAVROS publishes /mavros/state with TRANSIENT_LOCAL-ish best-effort; use sensor QoS.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ---- interfaces ----
        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, sensor_qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        self.sp_pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)

        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        self.timer = self.create_timer(1.0 / self.rate_hz, self._loop)
        self.get_logger().info("offboard_keyboard up. Waiting for FCU connection...")

    # ---- callbacks ----
    def _on_state(self, msg: State):
        self.state = msg

    def _on_pose(self, msg: PoseStamped):
        self.cur_alt = msg.pose.position.z

    def _on_cmd(self, msg: Twist):
        self.cmd = msg

    # ---- helpers ----
    @staticmethod
    def _clamp(v, lim):
        return max(-lim, min(lim, v))

    def _make_sp(self, vx, vy, vz, yaw_rate) -> PositionTarget:
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_BODY_NED
        sp.type_mask = TYPE_MASK_VEL_YAWRATE
        sp.velocity.x = self._clamp(vx, self.max_xy)
        sp.velocity.y = self._clamp(vy, self.max_xy)
        sp.velocity.z = self._clamp(vz, self.max_z)
        sp.yaw_rate = self._clamp(yaw_rate, self.max_yaw)
        return sp

    def _request_offboard_and_arm(self):
        """Throttled (1 Hz) requests so we don't spam the FCU services."""
        now = self.get_clock().now()
        if (now - self._last_req).nanoseconds < 1e9:
            return
        self._last_req = now
        if self.state.mode != "OFFBOARD" and self.mode_cli.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            self.mode_cli.call_async(req)
            self.get_logger().info("Requesting OFFBOARD...")
        elif self.auto_arm and not self.state.armed and self.arm_cli.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arm_cli.call_async(req)
            self.get_logger().info("Requesting ARM...")

    # ---- main loop ----
    def _loop(self):
        if self.phase == "wait_fcu":
            if self.state.connected:
                self.get_logger().info("FCU connected. Streaming setpoints...")
                self.phase = "stream"
            self.sp_pub.publish(self._make_sp(0.0, 0.0, 0.0, 0.0))
            return

        if self.phase == "stream":
            # Keep streaming zero setpoints while we flip OFFBOARD + ARM.
            self.sp_pub.publish(self._make_sp(0.0, 0.0, 0.0, 0.0))
            self._request_offboard_and_arm()
            if self.state.mode == "OFFBOARD" and self.state.armed:
                self.get_logger().info(
                    f"OFFBOARD + ARMED. Climbing to {self.takeoff_alt:.1f} m...")
                self.phase = "takeoff"
            return

        if self.phase == "takeoff":
            if self.cur_alt < self.takeoff_alt - 0.15:
                self.sp_pub.publish(self._make_sp(0.0, 0.0, 0.5, 0.0))
            else:
                self.get_logger().info("Takeoff complete. Keyboard control active "
                                       "(use teleop_twist_keyboard: i/, t/b j/l).")
                self.phase = "fly"
            return

        if self.phase == "fly":
            # Safety: if user disarmed/exited OFFBOARD, fall back to streaming.
            if not self.state.armed or self.state.mode != "OFFBOARD":
                self.get_logger().warn("Lost OFFBOARD/ARM; re-acquiring.")
                self.phase = "stream"
                return
            c = self.cmd
            self.sp_pub.publish(self._make_sp(
                c.linear.x, c.linear.y, c.linear.z, c.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = OffboardKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
