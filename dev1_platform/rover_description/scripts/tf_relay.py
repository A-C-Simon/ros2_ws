#!/usr/bin/env python3
"""Namespace-aware bidirectional relay between global /tf and per-rover namespaces.

Gazebo DiffDrive publishes robot TF (odom->base_link, sensor frames) on the
global /tf and /tf_static.  Nav2 stacks that run inside a rover namespace
(use_namespace:=True) subscribe to /<ns>/tf instead.

This relay is namespace-aware:
  - Global → namespace: prefixes frame_id/child_frame_id with <ns>/ if not
    already prefixed, so Nav2 sees rover_0/odom, rover_0/base_link etc.
  - Namespace → global: copies transforms as-is (keeps rover_0/odom etc.)
    so rviz and other global consumers see namespaced frames.

Loop prevention:
  - Tracks outbound frame pairs published to global, keyed by (frame pair,
    stamp). Only the relay's own echo of a message is skipped — a NEW message
    with the same frame pair but a different stamp still relays, so live
    streams (e.g. DiffDrive odom -> base_link on global /tf) keep flowing
    instead of being blacklisted after their first echo.

Usage:
    tf_relay.py --namespaces rover_0 rover_1
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

_QOS_DYN_GLOBAL_SUB = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE)
_QOS_DYN_NS_SUB = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
_QOS_DYN_PUB = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE)
_QOS_STATIC_GLOBAL_SUB = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
_QOS_STATIC_NS_SUB = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
_QOS_STATIC_PUB = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _frame_pair(transform):
    return (transform.header.frame_id, transform.child_frame_id)


def _stamp_key(stamp):
    return (stamp.sec, stamp.nanosec)


class _EchoTracker:
    """Set of (frame pair, stamp) the relay already forwarded to the global
    /tf. Keying on the stamp means only the relay's own echo of a message is
    skipped — a fresh message on the same frame pair still relays (a naive
    pair-only set blacklists the whole stream after the first echo)."""

    def __init__(self, window_sec=60.0, cap=20000):
        self._entries = {}   # (pair, sec, nanosec) -> None
        self._window_sec = window_sec
        self._cap = cap

    def contains(self, pair, stamp):
        key = (pair, stamp.sec, stamp.nanosec)
        return key in self._entries

    def add(self, pair, stamp):
        self._entries[(pair, stamp.sec, stamp.nanosec)] = None
        if len(self._entries) > self._cap:
            newest = max(sec for _, sec, _ in self._entries)
            cutoff = newest - self._window_sec
            self._entries = {
                k: None for k in self._entries if k[1] >= cutoff}


def _prefix_frame(frame, ns):
    """Prefix a frame name with ns/ if it doesn't already start with it."""
    if frame.startswith(f'{ns}/') or frame.startswith(f'/{ns}/'):
        return frame
    return f'{ns}/{frame}'


def _prefix_transform(tf, ns):
    """Return a copy of tf with frame_id and child_frame_id prefixed.

    If the child_frame_id is already namespaced (starts with ns/), the
    transform was deliberately placed at the global level (e.g. the static
    publisher's ``map → rover_0/map``) and must pass through unchanged.
    """
    if tf.child_frame_id.startswith(f'{ns}/') or tf.child_frame_id.startswith(f'/{ns}/'):
        return tf  # already namespaced — pass through
    import copy
    t = copy.deepcopy(tf)
    t.header.frame_id = _prefix_frame(t.header.frame_id, ns)
    t.child_frame_id = _prefix_frame(t.child_frame_id, ns)
    return t


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespaces', nargs='+', required=True,
                        help='Rover namespaces to relay TF into (e.g. rover_0 rover_1)')
    parser.add_argument('--ros-args', nargs='*', help='Pass-through ignored args')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Node('tf_relay')

    # --- publishers ---
    dyn_pubs = {ns: node.create_publisher(TFMessage, f'/{ns}/tf', _QOS_DYN_PUB)
                for ns in args.namespaces}
    static_pubs = {ns: node.create_publisher(TFMessage, f'/{ns}/tf_static', _QOS_STATIC_PUB)
                   for ns in args.namespaces}
    global_dyn_pub = node.create_publisher(TFMessage, '/tf', _QOS_DYN_PUB)
    global_static_pub = node.create_publisher(TFMessage, '/tf_static', _QOS_STATIC_PUB)

    # --- loop prevention: track (frame pair, stamp) echoes we published to global ---
    outbound_global_dyn = _EchoTracker()   # echoes we published to /tf
    outbound_global_static = _EchoTracker()  # echoes we published to /tf_static

    # --- global → namespaces (prefix frame names, skip our own echoes) ---
    def on_global_tf(msg, is_static=False):
        if is_static:
            skip = outbound_global_static
            fresh = [t for t in msg.transforms if not skip.contains(_frame_pair(t), t.header.stamp)]
            if fresh:
                for ns in args.namespaces:
                    prefixed = [_prefix_transform(t, ns) for t in fresh]
                    static_pubs[ns].publish(TFMessage(transforms=prefixed))
        else:
            skip = outbound_global_dyn
            fresh = [t for t in msg.transforms if not skip.contains(_frame_pair(t), t.header.stamp)]
            if fresh:
                for ns in args.namespaces:
                    prefixed = [_prefix_transform(t, ns) for t in fresh]
                    dyn_pubs[ns].publish(TFMessage(transforms=prefixed))

    # --- namespaces → global (copy as-is, skip our own echoes) ---
    def on_ns_tf(msg, ns, is_static=False):
        if is_static:
            skip = outbound_global_static
            fresh = [t for t in msg.transforms if not skip.contains(_frame_pair(t), t.header.stamp)]
            if fresh:
                for t in fresh:
                    skip.add(_frame_pair(t), t.header.stamp)
                global_static_pub.publish(TFMessage(transforms=fresh))
        else:
            skip = outbound_global_dyn
            fresh = [t for t in msg.transforms if not skip.contains(_frame_pair(t), t.header.stamp)]
            if fresh:
                for t in fresh:
                    skip.add(_frame_pair(t), t.header.stamp)
                global_dyn_pub.publish(TFMessage(transforms=fresh))

    node.create_subscription(TFMessage, '/tf',
                             lambda msg: on_global_tf(msg, False), _QOS_DYN_GLOBAL_SUB)
    node.create_subscription(TFMessage, '/tf_static',
                             lambda msg: on_global_tf(msg, True), _QOS_STATIC_GLOBAL_SUB)

    for ns in args.namespaces:
        node.create_subscription(
            TFMessage, f'/{ns}/tf',
            lambda msg, _ns=ns: on_ns_tf(msg, _ns, False), _QOS_DYN_NS_SUB)
        node.create_subscription(
            TFMessage, f'/{ns}/tf_static',
            lambda msg, _ns=ns: on_ns_tf(msg, _ns, True), _QOS_STATIC_NS_SUB)

    node.get_logger().info(
        f'tf_relay: namespace-aware relay for {len(args.namespaces)} '
        f'namespace(s): {", ".join(args.namespaces)}')

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
