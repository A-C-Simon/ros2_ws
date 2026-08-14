#!/usr/bin/env python3
"""One-off migration: copy an existing kitti2rosbag2 bag into a new bag that
also has /tf, and corrected /car/base/odom + /car/base/odom_path.

Image, camera_info and velodyne_points messages are copied through as raw
serialized bytes (no decode/re-encode), so this only pays the cost of the
sqlite copy plus a cheap numpy pass over the poses - not a full re-run of the
kitti_rec node.
"""

import argparse
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_msgs.msg import TFMessage

from kitti2rosbag2.utils.kitti_utils import KITTIOdometryDataset
from kitti2rosbag2.utils.quaternion import Quaternion

R_PERM = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def stamp(timestamp_ns):
    return Time(sec=int(timestamp_ns // 10**9), nanosec=int(timestamp_ns % 10**9))


def make_transform(parent_frame, child_frame, translation, quaternion, timestamp_ns):
    t = TransformStamped()
    t.header.stamp = stamp(timestamp_ns)
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(translation[0])
    t.transform.translation.y = float(translation[1])
    t.transform.translation.z = float(translation[2])
    t.transform.rotation.w = float(quaternion[0])
    t.transform.rotation.x = float(quaternion[1])
    t.transform.rotation.y = float(quaternion[2])
    t.transform.rotation.z = float(quaternion[3])
    return t


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--src-bag', required=True)
    parser.add_argument('--dst-bag', required=True)
    parser.add_argument('--sequence', type=int, required=True)
    parser.add_argument('--data-dir', required=True)
    parser.add_argument('--odom-dir', required=True)
    parser.add_argument('--velodyne-dir', default=None)
    args = parser.parse_args()

    if os.path.exists(args.dst_bag):
        raise SystemExit(f'{args.dst_bag} already exists, refusing to overwrite')

    dataset = KITTIOdometryDataset(args.data_dir, args.sequence, args.odom_dir, args.velodyne_dir)
    ground_truth = dataset.odom_pose()
    times = dataset.times_file()

    Tr = dataset.velo_to_cam_matrix()
    quat_helper = Quaternion()
    velodyne_translation = R_PERM @ Tr[:, 3]
    velodyne_rotation = quat_helper.rotationmtx_to_quaternion(R_PERM @ Tr[:, :3])

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py._storage.StorageOptions(uri=args.src_bag, storage_id='sqlite3'),
        rosbag2_py._storage.ConverterOptions('', ''),
    )
    src_topics = reader.get_all_topics_and_types()

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py._storage.StorageOptions(uri=args.dst_bag, storage_id='sqlite3'),
        rosbag2_py._storage.ConverterOptions('', ''),
    )
    for topic in src_topics:
        writer.create_topic(topic)
    writer.create_topic(rosbag2_py._storage.TopicMetadata(
        name='/tf', type='tf2_msgs/msg/TFMessage', serialization_format='cdr'))

    p_msg = Path()
    counter = 0
    copied = 0
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()

        if topic == '/car/base/odom':
            # regenerate from ground truth with the fixed math instead of
            # copying the old (mis-indexed quaternion) message through
            translation = R_PERM @ ground_truth[counter][:3, 3]
            rotation = R_PERM @ ground_truth[counter][:3, :3] @ R_PERM.T
            quaternion = quat_helper.rotationmtx_to_quaternion(rotation)

            odom_msg = Odometry()
            odom_msg.header.stamp = stamp(timestamp_ns)
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'odom'
            odom_msg.pose.pose.position.x = float(translation[0])
            odom_msg.pose.pose.position.y = float(translation[1])
            odom_msg.pose.pose.position.z = float(translation[2])
            odom_msg.pose.pose.orientation.w = float(quaternion[0])
            odom_msg.pose.pose.orientation.x = float(quaternion[1])
            odom_msg.pose.pose.orientation.y = float(quaternion[2])
            odom_msg.pose.pose.orientation.z = float(quaternion[3])
            writer.write('/car/base/odom', serialize_message(odom_msg), timestamp_ns)

            pose = PoseStamped()
            pose.pose = odom_msg.pose.pose
            pose.header.stamp = stamp(timestamp_ns)
            pose.header.frame_id = 'odom'
            p_msg.poses.append(pose)
            p_msg.header.stamp = stamp(timestamp_ns)
            p_msg.header.frame_id = 'map'
            writer.write('/car/base/odom_path', serialize_message(p_msg), timestamp_ns)

            tf_msg = TFMessage()
            tf_msg.transforms.append(make_transform('map', 'odom', translation, quaternion, timestamp_ns))
            tf_msg.transforms.append(make_transform(
                'odom', 'velodyne', velodyne_translation, velodyne_rotation, timestamp_ns))
            writer.write('/tf', serialize_message(tf_msg), timestamp_ns)

            counter += 1

        elif topic == '/car/base/odom_path':
            # superseded by the recomputed path written above
            continue

        else:
            writer.write(topic, data, timestamp_ns)

        copied += 1
        if copied % 2000 == 0:
            print(f'{copied} messages written...')

    del writer
    print(f'Done. {counter} poses regenerated, {copied} messages written to {args.dst_bag}')


if __name__ == '__main__':
    main()
