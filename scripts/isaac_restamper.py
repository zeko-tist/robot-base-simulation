#!/usr/bin/env python3
"""
isaac_restamper.py — Restamp Isaac Sim messages with wall clock time.

Isaac Sim publishes /tf, /scan, /odom, /joint_states with simulation
timestamps (near zero). Nav2 uses wall clock (use_sim_time: false). This
node restamps all four with current wall clock time so Nav2 and
robot_state_publisher process them correctly.

Pipeline:
  Isaac Sim → /tf_isaac          → restamper → /tf            (wall clock stamped)
  Isaac Sim → /scan_raw          → restamper → /scan           (wall clock stamped)
  Isaac Sim → /odom_raw          → restamper → /odom           (wall clock stamped)
  Isaac Sim → /joint_states_raw  → restamper → /joint_states   (wall clock stamped)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                        DurabilityPolicy, HistoryPolicy)
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry

RELIABLE = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
BEST_EFFORT = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


class IsaacRestamper(Node):
    def __init__(self):
        super().__init__('isaac_restamper')

        # TF: /tf_isaac → /tf
        self.tf_sub  = self.create_subscription(
            TFMessage, '/tf_isaac', self.tf_cb, RELIABLE)
        self.tf_pub  = self.create_publisher(TFMessage, '/tf', RELIABLE)

        # Scan: /scan_raw → /scan
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_cb, BEST_EFFORT)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', BEST_EFFORT)

        # Odom: /odom_raw → /odom
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_cb, RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, '/odom', RELIABLE)

        # JointState: /joint_states_raw → /joint_states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states_raw', self.joint_cb, RELIABLE)
        self.joint_pub = self.create_publisher(
            JointState, '/joint_states', RELIABLE)

        self.get_logger().info(
            '[Restamper] Running — /tf_isaac /scan_raw /odom_raw '
            '/joint_states_raw → wall clock stamped /tf /scan /odom '
            '/joint_states')

    def tf_cb(self, msg: TFMessage):
        now = self.get_clock().now().to_msg()
        for t in msg.transforms:
            t.header.stamp = now
        self.tf_pub.publish(msg)

    def scan_cb(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)

    def odom_cb(self, msg: Odometry):
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        self.odom_pub.publish(msg)

    def joint_cb(self, msg: JointState):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
