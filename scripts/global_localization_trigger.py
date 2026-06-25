#!/usr/bin/env python3
"""
global_localization_trigger.py — Automatic global localisation for ZEKO

PURPOSE
-------
Eliminates the need to manually click "2D Pose Estimate" in RViz.

On startup:
  1. Waits for AMCL to become active (lifecycle state)
  2. Calls /reinitialize_global_localization — spreads particles uniformly across map
  3. Drives the robot in a slow rotation so LiDAR sees all directions (convergence aid)
  4. Monitors /amcl_pose covariance; declares "localised" when uncertainty < threshold
  5. In kidnapped mode: detects sudden covariance spikes and re-triggers global loc

LAUNCH
------
Add to your navigation.launch.py or run standalone:

    ros2 run zeko_description global_localization_trigger

Or from launch:
    Node(
        package='zeko_description',
        executable='global_localization_trigger',
        name='global_localization_trigger',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

PARAMETERS (declare via ros2 param or launch)
----------
    use_sim_time              : bool  = True   # Must match sim
    covariance_threshold      : float = 0.5    # Pose uncertainty (tr(Σ_xy)) → localised
    max_wait_for_amcl         : float = 60.0   # s, abort if AMCL not active
    rotation_speed            : float = 0.3    # rad/s during convergence spin
    rotation_duration         : float = 15.0   # s, how long to spin (AMCL converges faster with motion)
    kidnap_covariance_threshold: float = 2.0   # If pose uncertainty exceeds this → re-trigger
    check_kidnap              : bool  = True   # Enable kidnapped-robot recovery
    kidnap_recheck_interval   : float = 5.0    # s, how often to check for kidnapping after localised
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty


class GlobalLocalisationTrigger(Node):
    """
    ROS2 node that performs automatic global localisation without manual 2D Pose Estimate.
    """

    _STATE_WAITING_FOR_AMCL   = "waiting_for_amcl"
    _STATE_TRIGGERING          = "triggering"
    _STATE_SPINNING            = "spinning"
    _STATE_MONITORING          = "monitoring"
    _STATE_LOCALISED           = "localised"
    _STATE_KIDNAP_DETECTED     = "kidnap_detected"

    def __init__(self):
        super().__init__("global_localization_trigger")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("use_sim_time",                True)
        self.declare_parameter("covariance_threshold",        0.5)
        self.declare_parameter("max_wait_for_amcl",           60.0)
        self.declare_parameter("rotation_speed",              0.3)
        self.declare_parameter("rotation_duration",           15.0)
        self.declare_parameter("kidnap_covariance_threshold", 2.0)
        self.declare_parameter("check_kidnap",                True)
        self.declare_parameter("kidnap_recheck_interval",     5.0)

        self._cov_threshold   = self.get_parameter("covariance_threshold").value
        self._max_wait_amcl   = self.get_parameter("max_wait_for_amcl").value
        self._rot_speed       = self.get_parameter("rotation_speed").value
        self._rot_duration    = self.get_parameter("rotation_duration").value
        self._kidnap_thresh   = self.get_parameter("kidnap_covariance_threshold").value
        self._check_kidnap    = self.get_parameter("check_kidnap").value
        self._kidnap_interval = self.get_parameter("kidnap_recheck_interval").value

        # ── State ─────────────────────────────────────────────────────────
        self._state           = self._STATE_WAITING_FOR_AMCL
        self._latest_pose     = None
        self._spin_start      = None
        self._start_time      = self.get_clock().now()
        self._last_kidnap_check = self.get_clock().now()

        # ── Publishers ───────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── Subscribers ──────────────────────────────────────────────────
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            amcl_qos,
        )

        # ── Service clients ──────────────────────────────────────────────
        self._global_loc_client = self.create_client(
            Empty, "/reinitialize_global_localization"
        )

        # ── Main loop timer (10 Hz) ──────────────────────────────────────
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("[GlobalLoc] Node started — waiting for AMCL...")

    # ── ROS2 callbacks ──────────────────────────────────────────────────────

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Receive latest pose estimate from AMCL."""
        self._latest_pose = msg

    # ── Main state machine tick ─────────────────────────────────────────────

    def _tick(self) -> None:
        now = self.get_clock().now()

        if self._state == self._STATE_WAITING_FOR_AMCL:
            self._tick_waiting(now)

        elif self._state == self._STATE_TRIGGERING:
            self._tick_triggering()

        elif self._state == self._STATE_SPINNING:
            self._tick_spinning(now)

        elif self._state == self._STATE_MONITORING:
            self._tick_monitoring(now)

        elif self._state == self._STATE_LOCALISED:
            self._tick_localised(now)

        elif self._state == self._STATE_KIDNAP_DETECTED:
            self._tick_kidnap_detected()

    def _tick_waiting(self, now) -> None:
        """Wait for /reinitialize_global_localization service to appear (= AMCL active)."""
        if self._global_loc_client.service_is_ready():
            self.get_logger().info("[GlobalLoc] AMCL active — triggering global localisation.")
            self._state = self._STATE_TRIGGERING
            return

        elapsed = (now - self._start_time).nanoseconds / 1e9
        if elapsed > self._max_wait_amcl:
            self.get_logger().error(
                f"[GlobalLoc] AMCL not ready after {self._max_wait_amcl:.0f} s. "
                "Check that amcl is in lifecycle ACTIVE state."
            )
            self._timer.cancel()

    def _tick_triggering(self) -> None:
        """Send global localisation request to AMCL."""
        req = Empty.Request()
        future = self._global_loc_client.call_async(req)
        future.add_done_callback(self._on_global_loc_response)
        self.get_logger().info(
            "[GlobalLoc] Called /reinitialize_global_localization — "
            "particles now uniformly distributed across the map."
        )
        self._state = self._STATE_SPINNING
        self._spin_start = self.get_clock().now()

    def _tick_spinning(self, now) -> None:
        """
        Rotate the robot slowly in place to expose LiDAR to all directions.
        AMCL particle weights update as the robot moves; rotation accelerates convergence.

        IMPORTANT: This only works if Nav2's controller_server is NOT active yet
        (or is paused). If Nav2 is trying to follow a goal simultaneously, cmd_vel
        will conflict. Start navigation goals only AFTER localisation is complete.
        """
        elapsed = (now - self._spin_start).nanoseconds / 1e9

        if elapsed < self._rot_duration:
            msg = Twist()
            msg.angular.z = self._rot_speed
            self._cmd_vel_pub.publish(msg)

            # Check if already converged (saves time if map is distinctive)
            if self._latest_pose is not None:
                cov = self._pose_uncertainty(self._latest_pose)
                if cov < self._cov_threshold:
                    self.get_logger().info(
                        f"[GlobalLoc] Converged during spin! Uncertainty = {cov:.3f}"
                    )
                    self._stop_robot()
                    self._state = self._STATE_LOCALISED
                    return

            if elapsed > 5.0 and elapsed % 5.0 < 0.15:
                self.get_logger().info(
                    f"[GlobalLoc] Spinning for convergence... {elapsed:.0f}/{self._rot_duration:.0f} s"
                )
        else:
            self._stop_robot()
            self.get_logger().info("[GlobalLoc] Rotation complete — monitoring convergence.")
            self._state = self._STATE_MONITORING

    def _tick_monitoring(self, now) -> None:
        """Wait for the particle cloud to converge on a single pose."""
        if self._latest_pose is None:
            return

        cov = self._pose_uncertainty(self._latest_pose)
        pose = self._latest_pose.pose.pose.position

        self.get_logger().info(
            f"[GlobalLoc] Monitoring: uncertainty={cov:.3f} (threshold={self._cov_threshold:.2f}) "
            f"pose=({pose.x:.2f}, {pose.y:.2f})",
            throttle_duration_sec=3.0
        )

        if cov < self._cov_threshold:
            self.get_logger().info(
                f"[GlobalLoc] ✓ LOCALISED! Position: ({pose.x:.2f}, {pose.y:.2f}) "
                f"Uncertainty: {cov:.4f}"
            )
            self._state = self._STATE_LOCALISED

    def _tick_localised(self, now) -> None:
        """
        Robot is localised. Periodically check for kidnapping.
        (A sudden increase in covariance indicates the particle filter has lost tracking.)
        """
        if not self._check_kidnap:
            return

        elapsed = (now - self._last_kidnap_check).nanoseconds / 1e9
        if elapsed < self._kidnap_interval:
            return

        self._last_kidnap_check = now

        if self._latest_pose is None:
            return

        cov = self._pose_uncertainty(self._latest_pose)
        if cov > self._kidnap_thresh:
            self.get_logger().warn(
                f"[GlobalLoc] ⚠ Kidnap detected! Uncertainty={cov:.3f} > {self._kidnap_thresh:.2f}. "
                "Re-triggering global localisation."
            )
            self._state = self._STATE_KIDNAP_DETECTED

    def _tick_kidnap_detected(self) -> None:
        """Re-trigger global localisation after kidnap event."""
        self._stop_robot()
        self._state = self._STATE_TRIGGERING

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _pose_uncertainty(self, msg: PoseWithCovarianceStamped) -> float:
        """
        Compute scalar uncertainty from the covariance matrix.

        Returns trace(Σ_xy) = cov[0,0] + cov[1,1] (position variance in x + y).
        Lower is more certain. A converged filter typically reaches < 0.1 m².
        """
        cov = msg.pose.covariance   # 6×6 matrix as flat array (36 elements)
        # cov[0]  = Var(x)
        # cov[7]  = Var(y)
        # cov[35] = Var(yaw)
        return math.sqrt(cov[0] ** 2 + cov[7] ** 2)

    def _on_global_loc_response(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(
                f"[GlobalLoc] /reinitialize_global_localization call failed: {exc}"
            )

    def _stop_robot(self) -> None:
        """Publish a zero-velocity Twist to stop any rotation."""
        self._cmd_vel_pub.publish(Twist())


# ── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalisationTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
