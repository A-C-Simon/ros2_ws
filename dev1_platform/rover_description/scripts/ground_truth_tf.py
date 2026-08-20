#!/usr/bin/env python3
"""Republish Gazebo ground-truth model poses as TF (and, in GT mode, odometry)
for the whole fleet.

Why this exists: the DiffDrive plugin's odometry (``<ns>/odom →
<ns>/base_link`` TF + ``/<ns>/odom`` topic) is ENCODER dead reckoning from
wheel velocities, planar only. When a rover is teleop'd into a wall the
wheels keep spinning and the odometry walks straight through it; a flip never
shows at all. Gazebo knows the true pose but never exported it.

The URDF's PosePublisher plugin publishes each model's world pose as
ignition.msgs.Pose on gz topic ``/model/<ns>/pose``; the per-rover bridge
forwards it to ROS as ``/model/<ns>/pose`` (geometry_msgs/PoseStamped). This
node consumes that stream and publishes:

  ALWAYS:
    TF  map → <ns>/gt_base_link        (full 6-DOF truth, on global /tf)

  With --publish-odom (ground_truth:=true, the default):
    TF  <ns>/odom → <ns>/base_link     (planar, spawn-relative truth)
    /<ns>/odom  (nav_msgs/Odometry)    (same pose + finite-difference twist)

    In this mode the launch file diverts the DiffDrive's encoder TF to a
    dead-end gz topic (<ns>/encoder_tf) and drops the /<ns>/odom bridge, so
    this node is the ONLY source of odom TF/topic — no interleaving. The
    planar spawn-relative convention matches encoder odometry exactly
    (identity at spawn), so slam_toolbox / Nav2 / cslam see their usual
    interface, just fed with truth.

The gz world frame IS the ROS ``map`` frame: each rover's ``<ns>/map`` is
anchored to ``map`` by a static TF at its gz spawn pose, and gz world
coordinates of a spawn match that pose — so the world pose from Gazebo is
directly a pose in ``map``. (The incoming header.frame_id carries the gz
world name, e.g. "test_station"; it is deliberately replaced with ``map``.)

The GT frame is named ``gt_base_link`` (prefix style) so a second
robot_state_publisher running with ``frame_prefix=<ns>/gt_`` hangs the full
URDF link tree under it — letting RViz render a fully-articulated robot at
the TRUE pose ("RobotModel (Gazebo truth)" display), and letting
gt_sensor_relay.py re-anchor the sensor overlays onto it.

Published on global /tf only, exactly like the DiffDrive TF: RViz (running
namespaced under the first rover) sees it wherever it already sees the
odom TF — i.e. whenever tf_relay.py runs (nav2_rover / rover_swarm
sessions). In a bare simulation.launch.py session RViz's TF feed is
limited to /rover_0/tf either way; that pre-existing gap is unchanged.

Usage:
    ground_truth_tf.py --namespaces rover_0 rover_1 [--publish-odom]
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# BEST_EFFORT subscriber is compatible with whatever QoS the bridge picks
# (a reliable subscriber would silently starve on a best-effort publisher).
_QOS_SUB = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class GroundTruthTF(Node):
    """One subscription per rover; republishes each pose as TF (+ odometry)."""

    def __init__(self, namespaces, publish_odom):
        super().__init__('ground_truth_tf')
        self._broadcaster = TransformBroadcaster(self)
        self._publish_odom = publish_odom
        self._odom_pubs = {}
        self._origin = {}   # ns -> (x0, y0, yaw0) first pose = odom origin
        self._last = {}     # ns -> (x_rel, y_rel, yaw_rel, stamp_sec)
        for ns in namespaces:
            self.create_subscription(
                PoseStamped, f'/model/{ns}/pose',
                lambda msg, _ns=ns: self._on_pose(msg, _ns), _QOS_SUB)
            if publish_odom:
                self._odom_pubs[ns] = self.create_publisher(
                    Odometry, f'/{ns}/odom', 10)
        self.get_logger().info(
            f'ground_truth_tf: map → <ns>/gt_base_link for '
            f'{len(namespaces)} rover(s): {", ".join(namespaces)}'
            + (' (+ <ns>/odom TF+topic from ground truth)'
               if publish_odom else ''))

    def _on_pose(self, msg, ns):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp      # sim time, straight from gz
        t.header.frame_id = 'map'              # gz world frame == ROS map
        t.child_frame_id = f'{ns}/gt_base_link'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self._broadcaster.sendTransform(t)
        if self._publish_odom:
            self._publish_odom_for(ns, msg)

    def _publish_odom_for(self, ns, msg):
        """Planar, spawn-relative truth — drop-in replacement for the
        DiffDrive encoder odometry interface (TF + nav_msgs/Odometry)."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = _yaw_from_quat(msg.pose.orientation)
        if ns not in self._origin:
            self._origin[ns] = (x, y, yaw)
        x0, y0, yaw0 = self._origin[ns]
        c0, s0 = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        xr = c0 * dx + s0 * dy
        yr = -s0 * dx + c0 * dy
        yawr = _wrap(yaw - yaw0)

        stamp = msg.header.stamp
        quat_z = math.sin(yawr / 2.0)
        quat_w = math.cos(yawr / 2.0)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = f'{ns}/odom'
        t.child_frame_id = f'{ns}/base_link'
        t.transform.translation.x = xr
        t.transform.translation.y = yr
        t.transform.rotation.z = quat_z
        t.transform.rotation.w = quat_w
        self._broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = f'{ns}/odom'
        odom.child_frame_id = f'{ns}/base_link'
        odom.pose.pose.position.x = xr
        odom.pose.pose.position.y = yr
        odom.pose.pose.orientation = t.transform.rotation
        last = self._last.get(ns)
        now = _stamp_to_sec(stamp)
        if last is not None:
            dt = now - last[3]
            if dt > 1e-3:
                vx_o = (xr - last[0]) / dt
                vy_o = (yr - last[1]) / dt
                cy, sy = math.cos(yawr), math.sin(yawr)
                # twist is expressed in the child (base_link) frame, matching
                # the DiffDrive plugin's odometry convention
                odom.twist.twist.linear.x = cy * vx_o + sy * vy_o
                odom.twist.twist.linear.y = -sy * vx_o + cy * vy_o
                odom.twist.twist.angular.z = _wrap(yawr - last[2]) / dt
        self._last[ns] = (xr, yr, yawr, now)
        self._odom_pubs[ns].publish(odom)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespaces', nargs='+', required=True,
                        help='Rover namespaces to watch (e.g. rover_0 rover_1)')
    parser.add_argument('--publish-odom', action='store_true',
                        help='Also publish <ns>/odom TF + topic from ground '
                             'truth (use when the launch diverted the '
                             'DiffDrive encoder odometry)')
    parser.add_argument('--ros-args', nargs='*', help='Pass-through ignored args')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GroundTruthTF(args.namespaces, args.publish_odom)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
