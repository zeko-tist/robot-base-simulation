"""
isaac_sim_ros2_bridge.py — Isaac Sim 6.0 OmniGraph ROS2 bridge setup for ZEKO

PURPOSE
-------
Sets up the full ROS2 communication bridge from Isaac Sim to Nav2:
  Publishes:  /odom, /tf (odom→base_link), /joint_states, /scan_base, /clock
  Subscribes: /cmd_vel → drives left/right wheel joints via articulation

USAGE
-----
Run this script from inside Isaac Sim (Script Editor or startup extension):

    import sys
    sys.path.insert(0, '/path/to/your/zeko_description/scripts')
    from isaac_sim_ros2_bridge import setup_zeko_ros2_bridge
    setup_zeko_ros2_bridge()

Or call it from your UIBuilder.on_timeline_event() when the simulation starts.

REQUIREMENTS
------------
- Isaac Sim 6.0 with isaacsim.ros2.bridge extension enabled
- ROS2 Humble sourced in the environment that launched Isaac Sim
- The robot articulation root prim at:  /World/zeko  (or update ROBOT_PRIM below)
- An RTX LiDAR prim at:               /World/zeko/Geometry/base_link/lidar_link
  (or update LIDAR_PRIM below)

SCENE SETUP IN ISAAC SIM
-------------------------
Before running this script:
1. Your world must be loaded with the ZEKO robot articulation
2. The RTX LiDAR sensor must be attached to the lidar_link prim
3. The simulation timeline must be running (or about to start)

PRIM PATHS (update to match your scene)
---------
These prim paths must match your Isaac Sim USD scene structure.
Inspect your Stage panel to confirm.
"""

import carb

# ── Prim paths — UPDATE THESE to match your Isaac Sim scene ────────────────
ROBOT_ROOT_PRIM = "/zeko/Geometry/base_link"
BASE_LINK_PRIM  = "/zeko/Geometry/base_link"
LIDAR_PRIM      = "/zeko/Geometry/base_link/support_frame_link/Lidar"
GRAPH_PATH      = "/World/ActionGraph"

# ── Robot geometry (must match urdf/base.xacro + config/ros2_control.yaml) ──
WHEEL_SEPARATION  = 0.60      # m, centre-to-centre of left and right wheels
WHEEL_RADIUS      = 0.1524    # m, radius of drive wheels
LEFT_WHEEL_JOINT  = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"

# ── ROS2 topic / frame names ─────────────────────────────────────────────────
ODOM_TOPIC        = "odom"
CMD_VEL_TOPIC     = "cmd_vel"
JOINT_STATE_TOPIC = "joint_states"
SCAN_TOPIC        = "scan_base"
CLOCK_TOPIC       = "clock"
BASE_FRAME        = "base_link"
ODOM_FRAME        = "odom"
LIDAR_FRAME       = "lidar_link"    # MUST match the joint name in lidar.xacro


def setup_zeko_ros2_bridge():
    """
    Create the OmniGraph that bridges Isaac Sim physics to ROS2 Nav2.

    This function creates a single Execution graph with:
      - OnPlaybackTick (runs every physics step)
      - ROS2 context
      - Sim time → /clock
      - Odometry compute → /odom + TF odom→base_link
      - Joint state read → /joint_states
      - Twist subscribe /cmd_vel → differential controller → articulation
      - RTX LiDAR → /scan_base LaserScan

    Call this function once after the stage is loaded and before Play.
    """
    try:
        import omni.graph.core as og
        from omni.isaac.core.utils.stage import get_current_stage
    except ImportError:
        carb.log_error("[ZEKO Bridge] omni.graph.core not available. "
                       "Ensure Isaac Sim is fully loaded before calling setup.")
        return False

    carb.log_info("[ZEKO Bridge] Creating OmniGraph ROS2 bridge ...")

    # ── Helper: safely set a node attribute ──────────────────────────────────
    def _set(keys_values: dict):
        """Batch-set node attribute values via og.Controller.Keys.SET_VALUES."""
        return [(k, v) for k, v in keys_values.items()]

    try:
        # ── Graph definition ─────────────────────────────────────────────────
        og.Controller.edit(
            {
                "graph_path":    GRAPH_PATH,
                "evaluator_name": "execution",
            },
            {
                # ── Node creation ────────────────────────────────────────────
                og.Controller.Keys.CREATE_NODES: [
                    # Execution tick (fires every physics step while sim is running)
                    ("OnPlaybackTick",   "omni.graph.action.OnPlaybackTick"),

                    # ROS2 context — one per graph
                    ("ROS2Context",      "isaacsim.ros2.bridge.ROS2Context"),

                    # Simulation time → ROS2 /clock
                    ("SimTime",          "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishClock",     "isaacsim.ros2.bridge.ROS2PublishClock"),

                    # Odometry: compute from articulation, then publish
                    ("ComputeOdom",      "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("PublishOdom",      "isaacsim.ros2.bridge.ROS2PublishOdometry"),

                    # TF publisher — publishes odom→base_link transform
                    ("PublishTF",        "isaacsim.ros2.bridge.ROS2PublishTransformTree"),

                    # Joint states: read from articulation, publish for robot_state_publisher
                    ("ReadJointState",   "isaacsim.core.nodes.IsaacReadJointState"),
                    ("PublishJointState","isaacsim.ros2.bridge.ROS2PublishJointState"),

                    # cmd_vel subscriber → differential controller → articulation
                    ("SubTwist",         "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ("DiffController",   "isaacsim.robot.wheeled_robots.HolonomicRobotUsdSetup"),
                    ("ArtController",    "isaacsim.core.nodes.IsaacArticulationController"),

                    # RTX LiDAR → LaserScan publisher
                    ("LidarHelper",      "isaacsim.sensor.rtx.ROS2RTXLidarHelper"),
                ],

                # ── Attribute values ─────────────────────────────────────────
                og.Controller.Keys.SET_VALUES: [
                    # ROS2 context — use domain ID 0 (default)
                    ("ROS2Context.inputs:domain_id", 0),

                    # Clock
                    ("PublishClock.inputs:topicName", CLOCK_TOPIC),

                    # Odometry
                    ("ComputeOdom.inputs:chassisPrim",  [BASE_LINK_PRIM]),
                    ("PublishOdom.inputs:topicName",    ODOM_TOPIC),
                    ("PublishOdom.inputs:odomFrameId",  ODOM_FRAME),
                    ("PublishOdom.inputs:chassisFrameId", BASE_FRAME),

                    # TF tree broadcaster
                    ("PublishTF.inputs:topicName",           "tf"),
                    ("PublishTF.inputs:parentFrameId",       ODOM_FRAME),
                    ("PublishTF.inputs:childFrameId",        BASE_FRAME),
                    ("PublishTF.inputs:targetPrims",         [BASE_LINK_PRIM]),

                    # Joint states
                    ("ReadJointState.inputs:targetPrim",    [ROBOT_ROOT_PRIM]),
                    ("PublishJointState.inputs:topicName",  JOINT_STATE_TOPIC),
                    ("PublishJointState.inputs:targetPrim", [ROBOT_ROOT_PRIM]),

                    # cmd_vel subscriber
                    ("SubTwist.inputs:topicName",           CMD_VEL_TOPIC),

                    # Articulation controller — applies joint velocities
                    ("ArtController.inputs:targetPrim",     [ROBOT_ROOT_PRIM]),
                    ("ArtController.inputs:jointNames",
                     [LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT]),
                    ("ArtController.inputs:usePath",        True),

                    # RTX LiDAR helper
                    ("LidarHelper.inputs:lidarPrimPath",    LIDAR_PRIM),
                    ("LidarHelper.inputs:topicName",        SCAN_TOPIC),
                    ("LidarHelper.inputs:frameId",          LIDAR_FRAME),  # CRITICAL
                    ("LidarHelper.inputs:renderProductPath", ""),           # Auto-detect
                    ("LidarHelper.inputs:type",             "laser_scan"),  # LaserScan msg type
                ],

                # ── Connections ───────────────────────────────────────────────
                og.Controller.Keys.CONNECT: [
                    # Tick → all publishers
                    ("OnPlaybackTick.outputs:tick",   "SimTime.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "PublishClock.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "ComputeOdom.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "PublishTF.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "ReadJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "SubTwist.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "ArtController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "LidarHelper.inputs:execIn"),

                    # ROS2 context → all ROS2 nodes
                    ("ROS2Context.outputs:context",   "PublishClock.inputs:context"),
                    ("ROS2Context.outputs:context",   "PublishOdom.inputs:context"),
                    ("ROS2Context.outputs:context",   "PublishTF.inputs:context"),
                    ("ROS2Context.outputs:context",   "PublishJointState.inputs:context"),
                    ("ROS2Context.outputs:context",   "SubTwist.inputs:context"),
                    ("ROS2Context.outputs:context",   "LidarHelper.inputs:context"),

                    # Sim time → publishers that need timestamps
                    ("SimTime.outputs:simulationTime","PublishClock.inputs:timeStamp"),
                    ("SimTime.outputs:simulationTime","PublishOdom.inputs:timeStamp"),
                    ("SimTime.outputs:simulationTime","PublishJointState.inputs:timeStamp"),
                    ("SimTime.outputs:simulationTime","PublishTF.inputs:timeStamp"),

                    # Odometry pipeline
                    ("ComputeOdom.outputs:execOut",   "PublishOdom.inputs:execIn"),
                    ("ComputeOdom.outputs:position",  "PublishOdom.inputs:position"),
                    ("ComputeOdom.outputs:orientation","PublishOdom.inputs:orientation"),
                    ("ComputeOdom.outputs:linearVelocity","PublishOdom.inputs:linearVelocity"),
                    ("ComputeOdom.outputs:angularVelocity","PublishOdom.inputs:angularVelocity"),

                    # TF
                    ("ComputeOdom.outputs:position",  "PublishTF.inputs:translation"),
                    ("ComputeOdom.outputs:orientation","PublishTF.inputs:rotation"),

                    # cmd_vel → articulation controller
                    # NOTE: SubTwist outputs linear/angular separately.
                    # The DifferentialController converts twist → [v_left, v_right].
                    # We wire: SubTwist → DiffController → ArtController
                    ("SubTwist.outputs:execOut",                "DiffController.inputs:execIn"),
                    ("SubTwist.outputs:linearVelocity",         "DiffController.inputs:linearVelocity"),
                    ("SubTwist.outputs:angularVelocity",        "DiffController.inputs:angularVelocity"),
                    ("DiffController.outputs:execOut",          "ArtController.inputs:execIn"),
                    ("DiffController.outputs:velocityCommands", "ArtController.inputs:velocityCommands"),
                ],
            }
        )

        # ── Post-creation: set differential controller geometry ───────────────
        #    These attributes are only settable after the node is created
        _set_diff_controller_params()

        carb.log_info("[ZEKO Bridge] OmniGraph created successfully at " + GRAPH_PATH)
        carb.log_info("[ZEKO Bridge] Publishing: /odom /tf /joint_states /scan_base /clock")
        carb.log_info("[ZEKO Bridge] Subscribing: /cmd_vel")
        return True

    except Exception as exc:
        carb.log_error(f"[ZEKO Bridge] Failed to create OmniGraph: {exc}")
        import traceback
        traceback.print_exc()
        return False


def _set_diff_controller_params():
    """
    Set wheel geometry on the differential controller node after graph creation.
    The node type may vary by Isaac Sim version — handle both known variants.
    """
    try:
        import omni.graph.core as og
        graph = og.get_graph_by_path(GRAPH_PATH)
        if graph is None:
            return

        node = graph.get_node(GRAPH_PATH + "/DiffController")
        if node is None or not node.is_valid():
            return

        # Try the standard attribute names for the differential controller
        for attr_name, val in [
            ("wheelDistance",     WHEEL_SEPARATION),
            ("wheelRadius",       WHEEL_RADIUS),
            ("wheel_distance",    WHEEL_SEPARATION),   # alternate naming
            ("wheel_radius",      WHEEL_RADIUS),
            ("maxLinearSpeed",    0.5),
            ("maxAngularSpeed",   1.0),
        ]:
            try:
                attr = node.get_attribute(f"inputs:{attr_name}")
                if attr and attr.is_valid():
                    attr.set(val)
            except Exception:
                pass  # Attribute may not exist on this node type

    except Exception as exc:
        carb.log_warn(f"[ZEKO Bridge] Could not set diff controller params: {exc}")


def setup_cmd_vel_via_python(robot_prim_path: str = ROBOT_ROOT_PRIM):
    """
    ALTERNATIVE to OmniGraph cmd_vel: subscribe to /cmd_vel using rclpy
    and drive the articulation directly from Python.

    Use this if the OmniGraph DifferentialController node is not available
    in your Isaac Sim version.

    Call this from your UIBuilder or from the Python REPL inside Isaac Sim.

    Args:
        robot_prim_path: USD path to the articulation root prim.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from isaacsim.core.prims import Articulation
        import numpy as np

        if not rclpy.ok():
            rclpy.init()

        class CmdVelBridge(Node):
            """Subscribes to /cmd_vel and drives ZEKO articulation."""

            def __init__(self):
                super().__init__('zeko_cmd_vel_bridge')
                self._robot = None
                self._left_idx  = None
                self._right_idx = None

                self.sub = self.create_subscription(
                    Twist,
                    CMD_VEL_TOPIC,
                    self._on_cmd_vel,
                    10
                )
                self.get_logger().info(
                    '[ZEKO] cmd_vel bridge node started — '
                    f'listening on /{CMD_VEL_TOPIC}'
                )

            def connect_articulation(self):
                """Call after the simulation is running."""
                robot = Articulation(prim_paths_expr=robot_prim_path, name="zeko_ctrl")
                robot.initialize()
                self._left_idx  = robot.dof_names.index(LEFT_WHEEL_JOINT)
                self._right_idx = robot.dof_names.index(RIGHT_WHEEL_JOINT)
                self._robot = robot
                self.get_logger().info('[ZEKO] Articulation connected.')

            def _on_cmd_vel(self, msg: Twist):
                if self._robot is None:
                    return

                # Differential drive inverse kinematics
                lx = msg.linear.x   # m/s
                az = msg.angular.z  # rad/s
                v_left  = (lx - az * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
                v_right = (lx + az * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS

                from isaacsim.core.utils.types import ArticulationActions
                vel = np.zeros((1, self._robot.num_dof))
                vel[0, self._left_idx]  = v_left
                vel[0, self._right_idx] = v_right

                self._robot.apply_action(
                    ArticulationActions(joint_velocities=vel)
                )

        node = CmdVelBridge()
        return node

    except ImportError as exc:
        carb.log_error(f"[ZEKO Bridge] rclpy not available: {exc}")
        return None


# ── Allow direct execution for testing ───────────────────────────────────────
if __name__ == "__main__":
    print("[ZEKO Bridge] Running setup_zeko_ros2_bridge()...")
    result = setup_zeko_ros2_bridge()
    print(f"[ZEKO Bridge] Result: {'OK' if result else 'FAILED'}")
