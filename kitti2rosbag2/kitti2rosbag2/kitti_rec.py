#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
import rosbag2_py
import os
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, Imu, NavSatFix, NavSatStatus
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
from kitti2rosbag2.utils.kitti_utils import KITTIOdometryDataset
from kitti2rosbag2.utils.quaternion import Quaternion
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message

class Kitti_Odom(Node):
    def __init__(self):
        super().__init__("kitti_rec")

        # parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sequence', rclpy.Parameter.Type.INTEGER),
                ('data_dir', rclpy.Parameter.Type.STRING),
                ('odom', rclpy.Parameter.Type.BOOL),
                ('odom_dir', rclpy.Parameter.Type.STRING),
                ('bag_dir', rclpy.Parameter.Type.STRING),
            ]
        )
        # optional, so that configs written before velodyne/imu support still load
        self.declare_parameter('velodyne', False)
        self.declare_parameter('velodyne_dir', '')
        self.declare_parameter('imu', False)
        self.declare_parameter('raw_dir', '')

        sequence = self.get_parameter('sequence').value
        data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        odom = self.get_parameter('odom').value
        bag_dir = self.get_parameter('bag_dir').get_parameter_value().string_value
        if odom == True:
            odom_dir = self.get_parameter('odom_dir').get_parameter_value().string_value
        else:
            odom_dir = None

        velodyne = self.get_parameter('velodyne').value
        velodyne_dir = self.get_parameter('velodyne_dir').get_parameter_value().string_value
        if velodyne_dir == '':
            velodyne_dir = None

        imu = self.get_parameter('imu').value
        raw_dir = self.get_parameter('raw_dir').get_parameter_value().string_value

        # set once the bag is complete (or setup failed), so main() can stop spinning
        self.done = False
        self.bag_open = False

        self.kitti_dataset = KITTIOdometryDataset(data_dir, sequence, odom_dir, velodyne_dir)
        self.bridge = CvBridge()
        self.quat_helper = Quaternion()
        # KITTI's camera axes (x-right, y-down, z-forward) permuted into the
        # standard ROS body convention (x-forward, y-left, z-up), applied to
        # both position and orientation so the two stay consistent with each
        # other and "local +X" actually matches the direction of travel.
        self.R_perm = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ])
        self.counter = 0
        self.counter_limit = len(self.kitti_dataset.left_images()) - 1 
        
        self.left_imgs = self.kitti_dataset.left_images()
        self.right_imgs = self.kitti_dataset.right_images()
        self.times_file = self.kitti_dataset.times_file()
        self.odom = odom
        if odom == True:
            try:
                self.ground_truth = self.kitti_dataset.odom_pose()
            except FileNotFoundError as filenotfounderror:
                self.get_logger().error("Error: {}".format(filenotfounderror))
                self.done = True
                return

        self.velodyne = velodyne
        if velodyne == True:
            try:
                self.velodyne_scans = self.kitti_dataset.velodyne_scans()
            except FileNotFoundError as filenotfounderror:
                self.get_logger().error("Error: {}".format(filenotfounderror))
                self.done = True
                return
            # scans and images are index-aligned, stop at whichever runs out first
            self.counter_limit = min(self.counter_limit, len(self.velodyne_scans) - 1)

            # velodyne is rigidly mounted, so its pose relative to the vehicle
            # frame is constant; derive it once from calib.txt's Tr (velo->cam0)
            Tr = self.kitti_dataset.velo_to_cam_matrix()
            static_rotation = self.R_perm @ Tr[:, :3]
            self.velodyne_translation = self.R_perm @ Tr[:, 3]
            self.velodyne_rotation = self.quat_helper.rotationmtx_to_quaternion(static_rotation)

        self.imu = imu
        self.imu_translation = None
        self.imu_rotation = None
        if imu == True:
            try:
                self.oxts_files = self.kitti_dataset.oxts_files(raw_dir)
            except FileNotFoundError as filenotfounderror:
                self.get_logger().error("Error: {}".format(filenotfounderror))
                self.done = True
                return
            # oxts and images are index-aligned, stop at whichever runs out first
            self.counter_limit = min(self.counter_limit, len(self.oxts_files) - 1)

            # imu_link is rigidly mounted too; if the raw dataset's IMU->velodyne
            # calibration is available, hang it off the 'velodyne' tf frame -
            # otherwise still publish IMU/GPS, just without a tf link
            if self.velodyne == True:
                imu_calib = self.kitti_dataset.imu_to_velo_matrix(raw_dir)
                if imu_calib is not None:
                    R_imu_velo, T_imu_velo = imu_calib
                    self.imu_rotation = self.quat_helper.rotationmtx_to_quaternion(R_imu_velo)
                    self.imu_translation = T_imu_velo
                else:
                    self.get_logger().warn(
                        "calib_imu_to_velo.txt not found under raw_dir; publishing IMU/GPS without a tf link.")

        # rosbag writer
        self.writer = rosbag2_py.SequentialWriter()
        if os.path.exists(bag_dir):
            self.get_logger().error(f'The directory {bag_dir} already exists. Shutting down...')
            self.done = True
            return
        storage_options = rosbag2_py._storage.StorageOptions(uri=bag_dir, storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        self.bag_open = True

        left_img_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera2/left/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr')
        right_img_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera3/right/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr')
        odom_topic_info = rosbag2_py._storage.TopicMetadata(name='/car/base/odom', type='nav_msgs/msg/Odometry', serialization_format='cdr')
        path_topic_info = rosbag2_py._storage.TopicMetadata(name='/car/base/odom_path', type='nav_msgs/msg/Path', serialization_format='cdr')
        left_cam_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera2/left/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr')
        right_cam_topic_info = rosbag2_py._storage.TopicMetadata(name='/camera3/right/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr')

        self.writer.create_topic(left_img_topic_info)
        self.writer.create_topic(right_img_topic_info)
        self.writer.create_topic(odom_topic_info)
        self.writer.create_topic(path_topic_info)
        self.writer.create_topic(left_cam_topic_info)
        self.writer.create_topic(right_cam_topic_info)

        if self.velodyne == True:
            velodyne_topic_info = rosbag2_py._storage.TopicMetadata(name='/velodyne_points', type='sensor_msgs/msg/PointCloud2', serialization_format='cdr')
            self.writer.create_topic(velodyne_topic_info)

        if self.imu == True:
            imu_topic_info = rosbag2_py._storage.TopicMetadata(name='/imu/data', type='sensor_msgs/msg/Imu', serialization_format='cdr')
            gps_topic_info = rosbag2_py._storage.TopicMetadata(name='/gps/fix', type='sensor_msgs/msg/NavSatFix', serialization_format='cdr')
            self.writer.create_topic(imu_topic_info)
            self.writer.create_topic(gps_topic_info)

        if self.velodyne == True or self.odom == True:
            tf_topic_info = rosbag2_py._storage.TopicMetadata(name='/tf', type='tf2_msgs/msg/TFMessage', serialization_format='cdr')
            self.writer.create_topic(tf_topic_info)

        # path msg
        self.p_msg = Path()

        # initialization
        self.timer = self.create_timer(0.05, self.rec_callback)


    def rec_callback(self):
        time = self.times_file[self.counter]
        timestamp_ns = int(time * 1e9) # nanoseconds

        # retrieving images and writing to bag
        left_image = cv2.imread(self.left_imgs[self.counter])
        right_image = cv2.imread(self.right_imgs[self.counter])
        left_img_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='passthrough')
        left_img_msg.header.stamp = self.stamp(timestamp_ns)
        self.writer.write('/camera2/left/image_raw', serialize_message(left_img_msg), timestamp_ns)
        right_img_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='passthrough')
        right_img_msg.header.stamp = self.stamp(timestamp_ns)
        self.writer.write('/camera3/right/image_raw', serialize_message(right_img_msg), timestamp_ns)

        # retrieving project mtx and writing to bag
        p_mtx2 = self.kitti_dataset.projection_matrix(1)
        self.rec_camera_info(p_mtx2, '/camera2/left/camera_info', timestamp_ns)
        p_mtx3 = self.kitti_dataset.projection_matrix(2)
        self.rec_camera_info(p_mtx3, '/camera3/right/camera_info', timestamp_ns)

        # retrieving velodyne scan and writing to bag
        if self.velodyne == True:
            points = self.kitti_dataset.read_velodyne_scan(self.velodyne_scans[self.counter])
            self.rec_pointcloud_msg(points, timestamp_ns)

        # tf: velodyne's mount point on the vehicle, plus the vehicle's pose in
        # map if ground truth is available. Both are needed for RViz to place
        # the point cloud - KITTI itself ships neither as tf.
        tf_msg = TFMessage()
        if self.velodyne == True:
            tf_msg.transforms.append(self.make_transform(
                'odom', 'velodyne', self.velodyne_translation, self.velodyne_rotation, timestamp_ns))

        # retrieving oxts entry and writing imu/gps to bag
        if self.imu == True:
            oxts = self.kitti_dataset.read_oxts(self.oxts_files[self.counter])
            self.rec_imu_msg(oxts, timestamp_ns)
            self.rec_gps_msg(oxts, timestamp_ns)
            if self.imu_rotation is not None:
                tf_msg.transforms.append(self.make_transform(
                    'velodyne', 'imu_link', self.imu_translation, self.imu_rotation, timestamp_ns))

        if self.odom == True:
            translation = self.R_perm @ self.ground_truth[self.counter][:3, 3]
            rotation = self.R_perm @ self.ground_truth[self.counter][:3, :3] @ self.R_perm.T
            quaternion = self.quat_helper.rotationmtx_to_quaternion(rotation)
            self.rec_odom_msg(translation, quaternion, timestamp_ns)
            tf_msg.transforms.append(self.make_transform('map', 'odom', translation, quaternion, timestamp_ns))

        if tf_msg.transforms:
            self.writer.write('/tf', serialize_message(tf_msg), timestamp_ns)

        self.get_logger().info(f'{self.counter}-Images Processed')
        if self.counter >= self.counter_limit:
            self.timer.cancel()
            self.get_logger().info('All images and poses published. Stopping...')
            self.close_bag()
            self.done = True

        self.counter += 1
        return

    def stamp(self, timestamp):
        # KITTI times.txt is relative to the start of the sequence, so the stamps
        # match the timestamps the messages are written under
        return Time(sec=int(timestamp // 10**9), nanosec=int(timestamp % 10**9))

    def close_bag(self):
        # flushes the sqlite file and writes metadata.yaml
        if self.bag_open:
            self.writer.close()
            self.bag_open = False
        return
    
    def rec_camera_info(self, mtx, topic, timestamp):
        camera_info_msg_2 = CameraInfo()
        camera_info_msg_2.header.stamp = self.stamp(timestamp)
        camera_info_msg_2.p = mtx.flatten()
        self.writer.write(topic, serialize_message(camera_info_msg_2), timestamp)   
        return
    
    def rec_pointcloud_msg(self, points, timestamp):
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = "velodyne"
        cloud_msg.header.stamp = self.stamp(timestamp)
        cloud_msg.height = 1
        cloud_msg.width = points.shape[0]
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * points.shape[0]
        cloud_msg.is_dense = True
        # KITTI's on-disk layout already matches the field layout above
        cloud_msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        self.writer.write('/velodyne_points', serialize_message(cloud_msg), timestamp)
        return

    def rec_imu_msg(self, oxts, timestamp):
        imu_msg = Imu()
        imu_msg.header.stamp = self.stamp(timestamp)
        imu_msg.header.frame_id = "imu_link"
        # roll/pitch/yaw are an absolute orientation with yaw=0 at east,
        # positive ccw - the same axis convention as ROS ENU, so a plain
        # Rz(yaw) @ Ry(pitch) @ Rx(roll) carries over directly
        roll, pitch, yaw = oxts['roll'], oxts['pitch'], oxts['yaw']
        Rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, np.cos(roll), -np.sin(roll)],
            [0.0, np.sin(roll), np.cos(roll)],
        ])
        Ry = np.array([
            [np.cos(pitch), 0.0, np.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-np.sin(pitch), 0.0, np.cos(pitch)],
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ])
        quaternion = self.quat_helper.rotationmtx_to_quaternion(Rz @ Ry @ Rx)
        imu_msg.orientation.w = quaternion[0]
        imu_msg.orientation.x = quaternion[1]
        imu_msg.orientation.y = quaternion[2]
        imu_msg.orientation.z = quaternion[3]
        # ax/ay/az and wx/wy/wz are already given in the vehicle body frame
        # (x-forward, y-left, z-up), matching ROS's body convention directly
        imu_msg.linear_acceleration.x = oxts['ax']
        imu_msg.linear_acceleration.y = oxts['ay']
        imu_msg.linear_acceleration.z = oxts['az']
        imu_msg.angular_velocity.x = oxts['wx']
        imu_msg.angular_velocity.y = oxts['wy']
        imu_msg.angular_velocity.z = oxts['wz']
        # oxts reports no per-axis accuracy for orientation/imu (only
        # pos_accuracy/vel_accuracy for GPS), so covariances are left unknown
        self.writer.write('/imu/data', serialize_message(imu_msg), timestamp)
        return

    def rec_gps_msg(self, oxts, timestamp):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.stamp(timestamp)
        gps_msg.header.frame_id = "imu_link"
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        gps_msg.latitude = oxts['lat']
        gps_msg.longitude = oxts['lon']
        gps_msg.altitude = oxts['alt']
        self.writer.write('/gps/fix', serialize_message(gps_msg), timestamp)
        return

    def rec_odom_msg(self, translation, quaternion, timestamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.stamp(timestamp)
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "odom"
        odom_msg.pose.pose.position.x = translation[0]
        odom_msg.pose.pose.position.y = translation[1]
        odom_msg.pose.pose.position.z = translation[2]
        # quaternion is [w, x, y, z]
        odom_msg.pose.pose.orientation.w = quaternion[0]
        odom_msg.pose.pose.orientation.x = quaternion[1]
        odom_msg.pose.pose.orientation.y = quaternion[2]
        odom_msg.pose.pose.orientation.z = quaternion[3]
        self.writer.write('/car/base/odom', serialize_message(odom_msg), timestamp)
        self.rec_path_msg(odom_msg, timestamp)
        return

    def make_transform(self, parent_frame, child_frame, translation, quaternion, timestamp):
        # quaternion is [w, x, y, z]
        t = TransformStamped()
        t.header.stamp = self.stamp(timestamp)
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
    
    def rec_path_msg(self, odom_msg, timestamp):
        pose= PoseStamped()
        pose.pose = odom_msg.pose.pose
        pose.header.stamp = self.stamp(timestamp)
        pose.header.frame_id = "odom"
        self.p_msg.poses.append(pose)
        # the path accumulates, so its own stamp tracks the latest pose
        self.p_msg.header.stamp = self.stamp(timestamp)
        self.p_msg.header.frame_id = "map"
        self.writer.write('/car/base/odom_path', serialize_message(self.p_msg), timestamp)
        return

def main(args=None):
    rclpy.init(args=args)
    node = Kitti_Odom()
    try:
        # spin_once instead of spin so the node can stop itself once the bag is
        # complete; calling rclpy.shutdown() from inside the timer callback hangs
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, closing the bag...')
    finally:
        # a partial bag is still readable as long as metadata.yaml gets written
        node.close_bag()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            pass

if __name__ == '__main__':
    main()