<div align="center">
  <h1>kitti2rosbag2</h1>
  <a href="https://github.com/bharadwajsirigadi/kitti2rosbag2/tree/main"><img src="https://img.shields.io/badge/ROS-humble-blue" /></a>
</div>

`kitti2rosbag2` is designed to convert the KITTI Odometry dataset to ROS2 bag format, emphasizing manual control over message publishing and bag recording.

## KITTI car setup

<div align="center">
  <img width="800" alt="Screenshot 2023-12-08 at 12 47 56 PM" src="https://github.com/bharadwajsirigadi/kitti2rosbag2/assets/105838762/42cd202f-2a14-418a-b576-bf5c55ea9d26">
</div>
Image source: (https://www.cvlibs.net/datasets/kitti/)

## Usage

#### 1. Clone Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/bharadwajsirigadi/kitti2rosbag2.git
```

#### 2. Parameters Input

Open  [params file](./config/params.yaml) </br>

Update following tags.

```yaml
kitti_pub:
  ros__parameters:
    sequence: <sequence_no>  #Integer
    data_dir: '<dataset_dir>'
    odom_dir: '<data_odometry_poses_dir>'
    bag_dir : '<bag_dir>/<bag_name>'
    odom : [True/False] 
    velodyne : [True/False]
    velodyne_dir : '<data_odometry_velodyne_dir>'  # leave '' if velodyne/ sits under data_dir
    imu : [True/False]  # publishes /imu/data + /gps/fix from the raw dataset's OXTS files
    raw_dir : '<kitti_raw_dir>'  # only needed if imu is True; see note below
```

Example: Converts kitty dataset to rosbag2.

Download data_odometry_poses from [here](https://github.com/bharadwajsirigadi/kitti2rosbag2/files/14692772/data_odometry_poses.zip)

```yaml
kitti_rec:
  ros__parameters:
    sequence: 0
    data_dir: '/home/user_name/Download/data_odometry_gray/dataset/'
    odom_dir: '/home/user_name/Download/data_odometry_poses/dataset/' 
    bag_dir : '/home/user_name/Download/00_bag'
    odom : True
    velodyne : True
    velodyne_dir : '/home/user_name/Download/data_odometry_velodyne/dataset/'
    imu : True
    raw_dir : '/home/user_name/Download/kitti_raw/'
```

#### 3. Building Package

```python
cd ~/ros2_ws
colcon build --packages-select kitti2rosbag2 --symlink-install
```

#### 4. Converting to bag

`
ros2 launch kitti2rosbag2 kitti2rosbag2.launch
`

[KITTI Odometry Dataset Folder Structure](https://github.com/bharadwajsirigadi/kitti2rosbag2/wiki)

## ROS Topics Info

* `car/base/odom`--> Odometry of Car.
* `car/base/odom_path`--> Ground Truth path of Car.
* `camera2/left/image_raw`--> Color Images from Left Camera.
* `camera3/right/image_raw`--> Color Images from Right Camera.
* `camera2/left/camera_info`--> Left Camera Information.
* `camera3/right/camera_info`--> Right Camera Information.
* `velodyne_points`--> Velodyne HDL-64E point clouds (`sensor_msgs/PointCloud2`, fields `x, y, z, intensity`, frame `velodyne`).
* `imu/data`--> IMU orientation/angular velocity/linear acceleration (`sensor_msgs/Imu`, frame `imu_link`).
* `gps/fix`--> GPS fix (`sensor_msgs/NavSatFix`, frame `imu_link`).

## Requirements

* Python 3.x
* ROS2 installed(tested on ROS2 humble)

### Note

Ground Truth-Odometry data is available for sequences(1-10) in KITTI dataset.

The KITTI odometry benchmark ships no IMU/GPS - that data only exists in KITTI's separate **raw (synced+rectified)** dataset, as per-frame OXTS files. Setting `imu: True` reads those files (via the devkit's odometry-sequence-to-raw-drive mapping, which only covers sequences 00-10) and publishes them as `/imu/data` and `/gps/fix`. Download the matching raw drive from the [raw data page](https://www.cvlibs.net/datasets/kitti/raw_data.php) and point `raw_dir` at the folder containing its date directory (e.g. `raw_dir/2011_10_03/2011_10_03_drive_0027_sync/...`). If `velodyne` is also enabled and `raw_dir/<date>/calib_imu_to_velo.txt` is present (from the raw calibration archive), `imu_link` is additionally linked into `/tf` off of `velodyne`.

## Contributors

* [Sai Bharadwaj Sirigadi](https://github.com/bharadwajsirigadi/kitti2rosbag2/graphs/contributors)
