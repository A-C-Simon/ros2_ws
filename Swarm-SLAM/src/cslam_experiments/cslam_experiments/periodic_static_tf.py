#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class PeriodicStaticTF(Node):
    """Republishes fixed identity transforms on /tf, stamped to match incoming
    sensor data rather than the node's own clock.

    Unlike tf2_ros static_transform_publisher (one-shot, latched on /tf_static),
    this re-broadcasts on every `stamp_topic` message using THAT message's own
    header.stamp. That sidesteps clock domain entirely: `ros2 bag play --clock`
    doesn't scale /clock by `-r` in this ROS 2 distro (it advances at wall-clock
    pace regardless of playback rate), so at rate != 1.0 a use_sim_time-driven
    broadcaster drifts away from bag-relative message timestamps and TF lookups
    against it fail. Mirroring the data's own stamp can't drift, by definition.

    `mounts` takes one or more "parent_frame:child_frame" pairs, all
    broadcast from a single node/callback - keeping this to one process matters
    on constrained hardware where every extra rclpy process adds contention.

    Republishes on every single stamp_topic message, not throttled: this node
    and icp_odometry's own scan_cloud subscription are two independently
    scheduled callbacks on the *same* topic, and tf2 never extrapolates
    forward - icp_odometry's lookup for message M only succeeds without
    extrapolation once a base_link->velodyne sample stamped at-or-after M's
    own timestamp is in its buffer. Throttling this republish (tried and
    measured: 285 -> 835 extrapolation/abort errors over a comparable run)
    only widens that gap, since it silently skips the exact per-message
    sample icp_odometry needs next.

    The subscription depth is 1 on purpose. icp_odometry reads the same topic
    with rtabmap's topic_queue_size=1, i.e. it always jumps to the *newest*
    cloud; a deeper queue here makes this node walk the stream in arrival
    order instead, so it serves stamps from progressively further back while
    icp_odometry asks about the head. Measured over two full runs, every one
    of 1464 "extrapolation into the future" gaps was an exact integer number
    of scan periods (1-8, mean deviation 0.26% of a period), capped by the
    old depth of 10 - a queue backlog, not jitter. Depth 1 keeps both
    subscribers pinned to the head of the same stream. Skipping intermediate
    clouds is harmless: the transform is constant, so a later sample still
    satisfies a lookup at an earlier stamp.
    """

    def __init__(self):
        super().__init__('periodic_static_tf')
        self.declare_parameter('mounts', ['base_link:velodyne'])
        self.declare_parameter('stamp_topic', '/r0/pointcloud')

        mounts = self.get_parameter('mounts').value
        self.frame_pairs = [tuple(m.split(':')) for m in mounts]
        stamp_topic = self.get_parameter('stamp_topic').value

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(PointCloud2, stamp_topic, self.publish_tf, 1)

    def publish_tf(self, msg):
        transforms = []
        for parent_frame, child_frame in self.frame_pairs:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = PeriodicStaticTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
