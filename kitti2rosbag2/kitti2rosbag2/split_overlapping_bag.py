#!/usr/bin/env python3
"""Splits a kitti2rosbag2-recorded ROS 2 bag into two overlapping bags, for
multi-robot / multi-agent Swarm-SLAM (cslam) testing.

WHY THIS EXISTS
---------------
cslam finds *inter-robot* loop closures by matching keyframe descriptors
across robots - but that only has something to find if the robots' bags
actually cover overlapping physical space. Two different KITTI odometry
sequences might not (this benchmark-only download has no OXTS/GPS ground
truth to check overlap against, and different sequence numbers are
different drives, possibly different parts of town entirely). Rather than
guessing which sequence pair happens to cross paths, this script takes ONE
sequence and splits it into two overlapping halves: both output bags
cover the same middle stretch of the drive, guaranteeing genuine spatial
overlap to test multi-robot loop closure against, e.g. with:

  ros2 launch cslam_experiments kitti2rosbag2_lidar.launch.py \\
      bag_files:=<output_a>,<output_b> max_nb_robots:=2

HOW THE SPLIT WORKS
--------------------
kitti2rosbag2's recorder (kitti_rec.py) writes one message per topic per
KITTI frame, so /velodyne_points, /imu/data and /gps/fix all have the same
message count and advance in lockstep - frame 0 of each topic corresponds
to the same KITTI frame, frame 1 to the next, and so on. This script counts
per-topic messages as it reads the source bag in recorded order to assign
each message a 0-based frame index, then writes:

  - bag A: frames [0, midpoint + overlap/2)
  - bag B: frames [midpoint - overlap/2, N)

so both bags share `overlap` frames (~overlap/10 seconds, at KITTI's 10Hz
lidar rate) of *identical* driving in the middle of the sequence. Point it
at robot 0 and robot 1's bag_files respectively and cslam has real
overlapping keyframes to find a match in.

Only /velodyne_points, /imu/data and /gps/fix are copied - the three topics
cslam_experiments/launch/sensors/bag_kitti2rosbag2.launch.py actually plays
back (see its own `--topics` filter) - not the full recording (camera
images, /tf, ground-truth odometry, etc), which cslam's lidar pipeline never
reads and which would multiply output size for no benefit.

Message timestamps are copied byte-for-byte from the source: both the
rosbag2 recording time (what `ros2 bag play` paces playback against) and
each message's own embedded header.stamp (what periodic_static_tf.py and
icp_odometry key their TF lookups off, per cslam_experiments'
kitti2rosbag2_lidar.launch.py). Neither output bag is re-timed to start at
zero - `ros2 bag play` only cares about the *relative* gaps between a bag's
own messages, so preserving the originals (including bag B's messages
starting mid-recording) keeps every downstream consumer's assumptions
intact without any extra bookkeeping here.

USAGE
-----
  ros2 run kitti2rosbag2 split_overlapping_bag \\
      --input /path/to/source_bag \\
      --output-a /path/to/robot0_bag \\
      --output-b /path/to/robot1_bag \\
      --overlap-frames 300

  --overlap-frames (default 300) is the *total* number of shared frames
  between the two bags, split evenly around the sequence's midpoint - e.g.
  300 frames = ~30s of identical driving shared by both robots at KITTI's
  10Hz lidar rate. Raise it if the loop-closure detector still doesn't find
  a match (more shared keyframes to try); lower it to keep the two robots'
  covered ground more distinct.
"""
import argparse
import os

import rosbag2_py
from rosbag2_py import ConverterOptions, StorageFilter, StorageOptions, TopicMetadata

# The only topics cslam_experiments' bag playback launch file reads
# (bag_kitti2rosbag2.launch.py's own `--topics` filter) - see module
# docstring for why we don't bother copying the rest of the recording.
SPLIT_TOPICS = ('/velodyne_points', '/imu/data', '/gps/fix')

# Used as the frame-index reference when counting total frames; any of
# SPLIT_TOPICS would give the same count since they advance in lockstep.
REFERENCE_TOPIC = SPLIT_TOPICS[0]


def _open_reader(input_bag, topics=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(StorageOptions(uri=input_bag, storage_id='sqlite3'),
                ConverterOptions('', ''))
    if topics is not None:
        reader.set_filter(StorageFilter(topics=list(topics)))
    return reader


def _read_topic_types(input_bag):
    reader = _open_reader(input_bag)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def _count_frames(input_bag):
    reader = _open_reader(input_bag, topics=[REFERENCE_TOPIC])
    n = 0
    while reader.has_next():
        reader.read_next()
        n += 1
    return n


def _open_writer(output_bag, topic_types):
    if os.path.exists(output_bag):
        raise FileExistsError(
            f"'{output_bag}' already exists - remove it first "
            "(rosbag2 refuses to write into an existing bag directory).")
    writer = rosbag2_py.SequentialWriter()
    writer.open(StorageOptions(uri=output_bag, storage_id='sqlite3'),
                ConverterOptions('', ''))
    for topic in SPLIT_TOPICS:
        writer.create_topic(TopicMetadata(
            name=topic, type=topic_types[topic], serialization_format='cdr'))
    return writer


def split_bag(input_bag, output_a, output_b, overlap_frames):
    topic_types = _read_topic_types(input_bag)
    missing = [t for t in SPLIT_TOPICS if t not in topic_types]
    if missing:
        raise ValueError(
            f"Input bag '{input_bag}' is missing expected topic(s): {missing} "
            f"(has: {sorted(topic_types)})")

    n_frames = _count_frames(input_bag)
    midpoint = n_frames // 2
    half_overlap = overlap_frames // 2
    # bag A: frames [0, a_end); bag B: frames [b_start, n_frames)
    a_end = min(n_frames, midpoint + half_overlap)
    b_start = max(0, midpoint - half_overlap)
    shared = max(0, a_end - b_start)

    print(f"Source bag: {n_frames} frames on '{REFERENCE_TOPIC}'")
    print(f"  bag A -> frames [0, {a_end})       ({a_end} frames)      -> {output_a}")
    print(f"  bag B -> frames [{b_start}, {n_frames})  ({n_frames - b_start} frames) -> {output_b}")
    print(f"  shared/overlap: frames [{b_start}, {a_end})  "
          f"({shared} frames, ~{shared / 10:.1f}s at 10Hz)")

    writer_a = _open_writer(output_a, topic_types)
    writer_b = _open_writer(output_b, topic_types)

    reader = _open_reader(input_bag, topics=SPLIT_TOPICS)
    per_topic_frame = {t: -1 for t in SPLIT_TOPICS}
    n_written_a = 0
    n_written_b = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        per_topic_frame[topic] += 1
        frame = per_topic_frame[topic]
        if frame < a_end:
            writer_a.write(topic, data, t)
            n_written_a += 1
        if frame >= b_start:
            writer_b.write(topic, data, t)
            n_written_b += 1

    del writer_a
    del writer_b
    print(f"Wrote {n_written_a} messages to bag A, {n_written_b} messages to bag B.")


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Split a kitti2rosbag2 bag into two overlapping bags for '
                    'multi-robot/multi-agent Swarm-SLAM testing.')
    parser.add_argument('--input', required=True,
                         help='Path to the source kitti2rosbag2 bag directory.')
    parser.add_argument('--output-a', required=True,
                         help="Output path for robot 0's bag "
                              "(frames [0, midpoint + overlap/2)).")
    parser.add_argument('--output-b', required=True,
                         help="Output path for robot 1's bag "
                              "(frames [midpoint - overlap/2, N)).")
    parser.add_argument('--overlap-frames', type=int, default=300,
                         help='Total shared frames between the two bags, split '
                              'evenly around the midpoint (default: 300, '
                              "~30s at KITTI's 10Hz lidar rate).")
    parsed = parser.parse_args(args)
    split_bag(parsed.input, parsed.output_a, parsed.output_b, parsed.overlap_frames)


if __name__ == '__main__':
    main()
