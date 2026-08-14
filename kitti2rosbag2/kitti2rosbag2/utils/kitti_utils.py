import time
import os
import numpy as np
from pathlib import Path
import cv2

DATA_EXTENSION = ".png"
DATASET_DIR = "/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset" # Fix this line
ODOM_DIR = '/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset_2'  # Fix this line
SEQUENCE = 0
LEFT_IMG_FOLDER = "image_0"
RIGHT_IMG_FOLDER = "image_1"
VELODYNE_FOLDER = "velodyne"
# DISTANCE = 0.54meters

# The odometry benchmark ships no IMU/GPS - that only exists in KITTI's raw
# (synced+rectified) drives. The devkit's readme.txt maps each odometry
# sequence with published ground truth (00-10) to the raw drive and frame
# range it was cut from; sequences 11-21 are the withheld test set and have
# no such mapping.
RAW_SEQUENCE_MAP = {
    0: ('2011_10_03', '0027', 0, 4540),
    1: ('2011_10_03', '0042', 0, 1100),
    2: ('2011_10_03', '0034', 0, 4660),
    3: ('2011_09_26', '0067', 0, 800),
    4: ('2011_09_30', '0016', 0, 270),
    5: ('2011_09_30', '0018', 0, 2760),
    6: ('2011_09_30', '0020', 0, 1100),
    7: ('2011_09_30', '0027', 0, 1100),
    8: ('2011_09_30', '0028', 1100, 5170),
    9: ('2011_09_30', '0033', 0, 1590),
    10: ('2011_09_30', '0034', 0, 1200),
}

# oxts/data/*.txt: one line per frame, defined by the raw devkit's
# oxts/dataformat.txt
OXTS_FIELDS = [
    'lat', 'lon', 'alt', 'roll', 'pitch', 'yaw',
    'vn', 've', 'vf', 'vl', 'vu',
    'ax', 'ay', 'az', 'af', 'al', 'au',
    'wx', 'wy', 'wz', 'wf', 'wl', 'wu',
    'pos_accuracy', 'vel_accuracy',
    'navstat', 'numsats', 'posmode', 'velmode', 'orimode',
]

class KITTIOdometryDataset():
    def __init__(self, data_dir, sequence: int, odom_dir = None, velodyne_dir = None,  *_, **__) -> None:
        self.sequence = sequence
        self.kitti_sequence_dir = os.path.join(data_dir, "sequences", f'{sequence:02d}')
        if odom_dir is not None:
            self.odom_dir = os.path.join(odom_dir, 'poses', f'{sequence:02d}.txt')
        # velodyne scans may live under data_dir, or under their own root if the
        # data_odometry_velodyne archive was extracted separately
        velodyne_root = data_dir if velodyne_dir is None else velodyne_dir
        self.velodyne_sequence_dir = os.path.join(velodyne_root, "sequences", f'{sequence:02d}', VELODYNE_FOLDER)
        self.left_cam_sequence_dir = os.path.join(self.kitti_sequence_dir, LEFT_IMG_FOLDER)
        self.right_cam_sequence_dir = os.path.join(self.kitti_sequence_dir, RIGHT_IMG_FOLDER)
        self.calib_file = os.path.join(self.kitti_sequence_dir,"calib.txt")
        self.time_file = os.path.join(self.kitti_sequence_dir,"times.txt")

    def get_files(self, extension, dir:Path):
        files = os.listdir(dir)
        filtered_files = []
        for file in files:
            if file.endswith(extension):
                filtered_files.append(file)
        filtered_files.sort()
        return filtered_files
    
    def write_text(self, files_list, file_name):
        file_name = f"file_{file_name}.txt"
        file_path = os.path.join(self.sequence_dir, file_name)
        try:
            with open(file_path, 'w') as file:
                for item in files_list:
                    file.write(f"{item}\n")
            print(f"File '{file_name}' has been created and written to '{file_path}'.")
        except Exception as e:
            print(f"An error occurred: {e}")
        return
    
    def left_images(self):
        image_extensions = [".jpg", ".jpeg", ".png", ".bmp", ".gif"]
        image_files = []
        for filename in os.listdir(self.left_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                image_files.append(os.path.join(self.left_cam_sequence_dir, filename))
        image_files = sorted(image_files)
        return image_files
    
    def right_images(self):
        image_extensions = [".jpg", ".jpeg", ".png", ".bmp", ".gif"]
        image_files = []
        for filename in os.listdir(self.right_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                image_files.append(os.path.join(self.right_cam_sequence_dir, filename))
        image_files = sorted(image_files)
        return image_files
    
    def stereo_images(self):
        image_extensions = [".jpg", ".jpeg", ".png", ".bmp", ".gif"]
        left_image_files = []
        for filename in os.listdir(self.left_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                left_image_files.append(os.path.join(self.left_cam_sequence_dir, filename))
        left_image_files = sorted(left_image_files)
        right_image_files = []
        for filename in os.listdir(self.right_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                right_image_files.append(os.path.join(self.right_cam_sequence_dir, filename))
        right_image_files = sorted(right_image_files)
        print(len(right_image_files))
        return left_image_files, right_image_files
    
    def velodyne_scans(self):
        if not os.path.isdir(self.velodyne_sequence_dir):
            raise FileNotFoundError(f"Velodyne directory not found: {self.velodyne_sequence_dir}, "
                                    "extract data_odometry_velodyne.zip or disable the 'velodyne' parameter. "
                                    "Stopping the process.")
        scan_files = []
        for filename in os.listdir(self.velodyne_sequence_dir):
            if filename.lower().endswith(".bin"):
                scan_files.append(os.path.join(self.velodyne_sequence_dir, filename))
        scan_files = sorted(scan_files)
        return scan_files

    def read_velodyne_scan(self, scan_file):
        # KITTI stores each scan as a flat float32 buffer of [x, y, z, intensity]
        scan = np.fromfile(scan_file, dtype=np.float32)
        return scan.reshape(-1, 4)

    def continuous_image_reader(self, images):
        if images == "right_images":
            image_files = self.right_images()
        elif images == "left_images":
            image_files = self.left_images()
        else:
            left_image_files, right_img_files = self.stereo_images()
        while True:
            if images == "right_images" or images == "left_images":
                for image_file in image_files:
                    image = cv2.imread(image_file)
                    window_name = "image display"
                    if image is not None:
                        print(f"Reading image: {image_file}")
                        cv2.imshow(window_name, image)
                        cv2.waitKey(55)
                time.sleep(5)
            else:
                for left_img_file, right_img_file in zip(left_image_files, right_img_files):
                    left_image = cv2.imread(left_img_file)
                    right_image = cv2.imread(right_img_file)
                    window_name = "Stereo display"
                    if left_image is not None and right_image is not None:
                        stereo_image = cv2.hconcat([left_image, right_image])
                        cv2.imshow("Stereo Display", stereo_image)
                        cv2.waitKey(55)
                    else:
                        print("One or both images are invalid.")
                time.sleep(5)
                cv2.destroyAllWindows()
                
    def projection_matrix(self, cam):
        projection_matrices = []
        with open(self.calib_file, 'r') as file:
            for line in file:
                if line.startswith('P'):
                    values = [float(x) for x in line.split(':')[1].strip().split()]
                    matrix = np.array(values).reshape(3, 4)
                    projection_matrices.append(matrix)
        return projection_matrices[cam]
    
    def velo_to_cam_matrix(self):
        # Tr: 3x4 extrinsic transforming a point from the velodyne frame into
        # the (rectified) cam0 frame: p_cam0 = Tr[:, :3] @ p_velo + Tr[:, 3]
        with open(self.calib_file, 'r') as file:
            for line in file:
                if line.startswith('Tr'):
                    values = [float(x) for x in line.split(':')[1].strip().split()]
                    return np.array(values).reshape(3, 4)
        raise ValueError(f"Tr (velodyne-to-camera) calibration not found in {self.calib_file}")

    def oxts_files(self, raw_dir):
        if self.sequence not in RAW_SEQUENCE_MAP:
            raise FileNotFoundError(
                f"No raw-drive mapping for sequence {self.sequence:02d}: OXTS (IMU/GPS) is only "
                "available for sequences 00-10, the ones the devkit maps back to a raw KITTI drive.")
        date, drive, start, end = RAW_SEQUENCE_MAP[self.sequence]
        oxts_dir = os.path.join(raw_dir, date, f'{date}_drive_{drive}_sync', 'oxts', 'data')
        if not os.path.isdir(oxts_dir):
            raise FileNotFoundError(
                f"OXTS directory not found: {oxts_dir}. Download the raw (synced+rectified) drive "
                f"'{date}_drive_{drive}_sync' from https://www.cvlibs.net/datasets/kitti/raw_data.php "
                "and point the 'raw_dir' parameter at the folder containing its date directory.")
        files = [os.path.join(oxts_dir, f) for f in self.get_files(".txt", oxts_dir)]
        # odometry frame i of this sequence is raw drive frame (start + i)
        return files[start:end + 1]

    def read_oxts(self, file_path):
        with open(file_path, 'r') as file:
            values = [float(v) for v in file.readline().split()]
        return dict(zip(OXTS_FIELDS, values))

    def imu_to_velo_matrix(self, raw_dir):
        # calib_imu_to_velo.txt is per-date, not per-drive, and only ships in
        # the raw dataset's calibration archive - treat it as optional
        if self.sequence not in RAW_SEQUENCE_MAP:
            return None
        date, _, _, _ = RAW_SEQUENCE_MAP[self.sequence]
        calib_file = os.path.join(raw_dir, date, 'calib_imu_to_velo.txt')
        if not os.path.isfile(calib_file):
            return None
        R, T = None, None
        with open(calib_file, 'r') as file:
            for line in file:
                if line.startswith('R:'):
                    R = np.array([float(x) for x in line.split(':')[1].strip().split()]).reshape(3, 3)
                elif line.startswith('T:'):
                    T = np.array([float(x) for x in line.split(':')[1].strip().split()])
        if R is None or T is None:
            return None
        return R, T

    def times_file(self):
        matrix = []
        with open(self.time_file, 'r') as file:
            for line in file:
                matrix.append(float(line))
        return np.array(matrix)
    
    def odom_pose(self):
        if not os.path.exists(self.odom_dir):
            raise FileNotFoundError(f"Odom directory not found: {self.odom_dir}, Ground truth(Odometry) is available for only 10 sequences in KITTI. Stopping the process.")
        with open(self.odom_dir, 'r') as file:
            lines = file.readlines()
        transformation_data = [[float(val) for val in line.split()] for line in lines]
        homogenous_matrix_arr = []
        for i in range(len(transformation_data)):
            homogenous_matrix = np.identity(4)
            homogenous_matrix[0, :] = transformation_data[i][0:4]
            homogenous_matrix[1:2, :] = transformation_data[i][4:8]
            homogenous_matrix[2:3, :] = transformation_data[i][8:12]
            homogenous_matrix_arr.append(homogenous_matrix)
        return np.array(homogenous_matrix_arr)
    
    def __getitem__(self, idx):

        return

    def __len__(self):
        
        return

def main():
    kitti = KITTIOdometryDataset(DATASET_DIR, ODOM_DIR, SEQUENCE)
    matrix = kitti.projection_matrix(3)
    right, left = kitti.stereo_images()
    # kitti.times_file()
    # arr = kitti.odom_pose()
    return

if __name__=="__main__":
    main()
    
