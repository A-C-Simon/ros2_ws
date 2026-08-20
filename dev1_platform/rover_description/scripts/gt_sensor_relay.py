#!/usr/bin/env python3
"""Republish rover sensor streams re-anchored to the ground-truth (gt_) frames.

Why: RViz renders PointCloud2 / Range by transforming each message's
header.frame_id through TF. Those frames resolve through the BELIEF chain
(encoder odometry), so when skid-steer slip drags the belief robot away from
reality, every sensor overlay in RViz visually follows the wrong robot.

This relay republishes the same streams on ``/<ns>/gt/...`` topics with the
frame_id swapped to the GT mirror links (``<ns>/gt_lidar_link`` etc.), which
the per-rover gt_state_publisher hangs under ``map → <ns>/gt_base_link``.
Result: RViz draws lidar points and sonar cones where the rover ACTUALLY is
("RobotModel (Gazebo truth)"), while the Odometry trail / SLAM map continue
to show the belief.

RViz-only. The original topics are published untouched — slam_toolbox, Nav2
and cslam keep consuming the belief-anchored streams exactly as before. If
the autonomy inputs ever need GT anchoring too, that's a separate, deliberate
decision (it makes localization 'perfect' and SLAM benchmarks meaningless).

Frame rewrite rules (per rover namespace <ns>):
  <ns>/base_link/<ign_suffix>  → <ns>/gt_<urdf_link>   (gz-scoped frames;
                                 identity-equivalent — see the ALIASES table
                                 in sensor_frame_aliases.py)
  <ns>/<urdf_link>             → <ns>/gt_<urdf_link>   (RSP belief frames;
                                 sonar_to_range.py puts these on Range msgs)

Usage:
    gt_sensor_relay.py --namespaces rover_0 rover_1
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Range

# gz-scoped sensor frame suffix → URDF link.
# Keep in sync with ALIASES in sensor_frame_aliases.py.
IGN_SUFFIX_TO_LINK = {
    'lidar':             'lidar_link',
    'camera':            'camera_link',
    'thermal':           'camera_link',
    'imu':               'base_link',
    'sonar_front':       'sonar_front_link',
    'sonar_front_right': 'sonar_front_right_link',
    'sonar_front_left':  'sonar_front_left_link',
    'sonar_rear':        'sonar_rear_link',
    'sonar_rear_right':  'sonar_rear_right_link',
    'sonar_rear_left':   'sonar_rear_left_link',
}

SONAR_NAMES = ['front', 'front_right', 'front_left',
               'rear', 'rear_right', 'rear_left']

# Subscribing BEST_EFFORT is compatible with any publisher side.
_QOS_SUB = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)
# Points: publish RELIABLE — a RELIABLE publisher feeds both RELIABLE and
# BEST_EFFORT subscribers, and RViz's PointCloud2 display was observed
# requesting RELIABLE regardless of the saved config policy (QoS warning:
# "requesting incompatible QoS ... RELIABILITY" → no messages shown).
_QOS_POINTS_PUB = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
# Ranges: 5 Hz — RELIABLE out feeds both RELIABLE and BEST_EFFORT subscribers.
_QOS_RANGE_PUB = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)


def gt_frame(frame_id, ns):
    """Map a belief-chain frame to its GT mirror, or return it unchanged."""
    prefix = f'{ns}/'
    gz_scoped = f'{prefix}base_link/'
    if frame_id.startswith(gz_scoped):
        link = IGN_SUFFIX_TO_LINK.get(frame_id[len(gz_scoped):])
        return f'{ns}/gt_{link}' if link else frame_id
    if frame_id.startswith(prefix):
        rest = frame_id[len(prefix):]
        if '/' not in rest:      # plain URDF link frame → gt_ counterpart
            return f'{ns}/gt_{rest}'
    return frame_id


class GtSensorRelay(Node):
    """Per rover: lidar points + 6 sonar ranges, re-framed onto the gt_ tree."""

    def __init__(self, namespaces):
        super().__init__('gt_sensor_relay')
        for ns in namespaces:
            self._relay(ns, 'lidar/points', PointCloud2, _QOS_POINTS_PUB)
            for sonar in SONAR_NAMES:
                self._relay(ns, f'sonar/{sonar}/range', Range, _QOS_RANGE_PUB)
        self.get_logger().info(
            f'gt_sensor_relay: re-framing sensor streams onto gt_ frames for '
            f'{len(namespaces)} rover(s): {", ".join(namespaces)}')

    def _relay(self, ns, suffix, msg_type, pub_qos):
        pub = self.create_publisher(msg_type, f'/{ns}/gt/{suffix}', pub_qos)
        self.create_subscription(
            msg_type, f'/{ns}/{suffix}',
            lambda msg, _ns=ns, _pub=pub: self._cb(msg, _ns, _pub),
            _QOS_SUB)

    @staticmethod
    def _cb(msg, ns, pub):
        msg.header.frame_id = gt_frame(msg.header.frame_id, ns)
        pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespaces', nargs='+', required=True,
                        help='Rover namespaces to relay (e.g. rover_0 rover_1)')
    parser.add_argument('--ros-args', nargs='*', help='Pass-through ignored args')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GtSensorRelay(args.namespaces)
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
