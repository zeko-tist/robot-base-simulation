"""
Isaac Sim OmniGraph ROS2 bridge — publishes to raw topics for restamper.

Publishes to /tf_isaac, /scan_raw, /odom_raw, /joint_states_raw instead of
/tf, /scan, /odom, /joint_states.
The isaac_restamper node picks these up and republishes with wall clock time.
"""
import omni.kit.app
import omni.graph.core as og
import omni.usd

stage = omni.usd.get_context().get_stage()
if stage.GetPrimAtPath("/World/ActionGraph").IsValid():
    stage.RemovePrim("/World/ActionGraph")

ROBOT_ROOT_PRIM  = "/zeko/Geometry/base_link"
LIDAR_PRIM       = "/zeko/Geometry/base_link/support_frame_link/Lidar"
GRAPH_PATH       = "/World/ActionGraph"
WHEEL_SEPARATION = 0.60
WHEEL_RADIUS     = 0.1524
LEFT_WHEEL       = "left_wheel_joint"
RIGHT_WHEEL      = "right_wheel_joint"

manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["omni.graph.core","omni.graph.action","isaacsim.ros2.bridge",
            "isaacsim.ros2.nodes","isaacsim.core.nodes","isaacsim.sensors.physx",
            "isaacsim.sensors.physics","isaacsim.sensors.physics.nodes",
            "isaacsim.robot.wheeled_robots","isaacsim.robot.wheeled_robots.nodes"]:
    try:
        manager.set_extension_enabled_immediate(ext, True)
    except:
        pass

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick",    "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context",       "isaacsim.ros2.bridge.ROS2Context"),
            ("ComputeOdom",       "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdom",       "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PublishTF",         "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("ReadJointState",    "isaacsim.sensors.physics.IsaacReadJointState"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubTwist",          "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("DiffController",    "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("ArtController",     "isaacsim.core.nodes.IsaacArticulationController"),
            ("ReadLidarBeams",    "isaacsim.sensors.physx.IsaacReadLidarBeams"),
            ("PublishScan",       "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
            # Twist linear/angular arrive as vector3d; DiffController wants
            # scalar doubles (forward speed = x, yaw rate = z).
            ("BreakLinear",       "omni.graph.nodes.BreakVector3"),
            ("BreakAngular",      "omni.graph.nodes.BreakVector3"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ROS2Context.inputs:domain_id",          0),
            ("ComputeOdom.inputs:chassisPrim",         [ROBOT_ROOT_PRIM]),
            # Publish to raw topics — restamper will forward with wall clock
            ("PublishOdom.inputs:topicName",           "odom_raw"),
            ("PublishOdom.inputs:odomFrameId",         "odom"),
            ("PublishOdom.inputs:chassisFrameId",      "base_link"),
            ("PublishTF.inputs:topicName",             "tf_isaac"),
            ("PublishTF.inputs:parentFrameId",         "odom"),
            ("PublishTF.inputs:childFrameId",          "base_link"),
            ("ReadJointState.inputs:prim",             [ROBOT_ROOT_PRIM]),
            ("PublishJointState.inputs:topicName",     "joint_states_raw"),
            ("PublishJointState.inputs:targetPrim",    [ROBOT_ROOT_PRIM]),
            ("SubTwist.inputs:topicName",              "cmd_vel"),
            ("DiffController.inputs:wheelDistance",    WHEEL_SEPARATION),
            ("DiffController.inputs:wheelRadius",      WHEEL_RADIUS),
            ("DiffController.inputs:maxLinearSpeed",   1.0),
            ("ArtController.inputs:robotPath",         ROBOT_ROOT_PRIM),
            ("ArtController.inputs:jointNames",        [LEFT_WHEEL, RIGHT_WHEEL]),
            ("ReadLidarBeams.inputs:lidarPrim",        [LIDAR_PRIM]),
            ("PublishScan.inputs:topicName",           "scan_raw"),
            ("PublishScan.inputs:frameId",             "lidar_link"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick",  "ComputeOdom.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",  "ReadJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",  "SubTwist.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",  "ArtController.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",  "ReadLidarBeams.inputs:execIn"),
            ("ROS2Context.outputs:context",  "PublishOdom.inputs:context"),
            ("ROS2Context.outputs:context",  "PublishTF.inputs:context"),
            ("ROS2Context.outputs:context",  "PublishJointState.inputs:context"),
            ("ROS2Context.outputs:context",  "SubTwist.inputs:context"),
            ("ROS2Context.outputs:context",  "PublishScan.inputs:context"),
            ("ComputeOdom.outputs:execOut",           "PublishOdom.inputs:execIn"),
            ("ComputeOdom.outputs:position",          "PublishOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation",       "PublishOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity",    "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity",   "PublishOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:execOut",           "PublishTF.inputs:execIn"),
            ("ComputeOdom.outputs:position",          "PublishTF.inputs:translation"),
            ("ComputeOdom.outputs:orientation",       "PublishTF.inputs:rotation"),
            ("ReadJointState.outputs:execOut",             "PublishJointState.inputs:execIn"),
            ("ReadJointState.outputs:jointNames",          "PublishJointState.inputs:jointNames"),
            ("ReadJointState.outputs:jointPositions",      "PublishJointState.inputs:jointPositions"),
            ("ReadJointState.outputs:jointVelocities",     "PublishJointState.inputs:jointVelocities"),
            ("ReadJointState.outputs:jointEfforts",        "PublishJointState.inputs:jointEfforts"),
            ("ReadJointState.outputs:jointDofTypes",       "PublishJointState.inputs:jointDofTypes"),
            ("ReadJointState.outputs:sensorTime",          "PublishJointState.inputs:sensorTime"),
            ("ReadJointState.outputs:stageMetersPerUnit",  "PublishJointState.inputs:stageMetersPerUnit"),
            ("SubTwist.outputs:execOut",                   "DiffController.inputs:execIn"),
            ("SubTwist.outputs:linearVelocity",            "BreakLinear.inputs:tuple"),
            ("SubTwist.outputs:angularVelocity",           "BreakAngular.inputs:tuple"),
            ("BreakLinear.outputs:x",                       "DiffController.inputs:linearVelocity"),
            ("BreakAngular.outputs:z",                      "DiffController.inputs:angularVelocity"),
            ("DiffController.outputs:velocityCommand",     "ArtController.inputs:velocityCommand"),
            ("ReadLidarBeams.outputs:execOut",             "PublishScan.inputs:execIn"),
            ("ReadLidarBeams.outputs:azimuthRange",        "PublishScan.inputs:azimuthRange"),
            ("ReadLidarBeams.outputs:depthRange",          "PublishScan.inputs:depthRange"),
            ("ReadLidarBeams.outputs:horizontalFov",       "PublishScan.inputs:horizontalFov"),
            ("ReadLidarBeams.outputs:horizontalResolution","PublishScan.inputs:horizontalResolution"),
            ("ReadLidarBeams.outputs:intensitiesData",     "PublishScan.inputs:intensitiesData"),
            ("ReadLidarBeams.outputs:linearDepthData",     "PublishScan.inputs:linearDepthData"),
            ("ReadLidarBeams.outputs:numCols",             "PublishScan.inputs:numCols"),
            ("ReadLidarBeams.outputs:numRows",             "PublishScan.inputs:numRows"),
            ("ReadLidarBeams.outputs:rotationRate",        "PublishScan.inputs:rotationRate"),
        ],
    }
)
print("Bridge ready — publishing to tf_isaac / scan_raw / odom_raw / joint_states_raw")
print("Run isaac_restamper to forward with wall clock timestamps")
