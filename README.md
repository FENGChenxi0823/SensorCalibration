# Imu-lidar extrinsic parameter calibration

This project aims to implement imu-lidar extrinsic parameter calibration algorithms.

There are two main packages(methods) implemented: "lidar_align" and "imu-lidar". All packges here are tested with real data and run on ROS (except "displyply").

This README file will briefly introduce the usge of these packages. Some packages are dependencies for others, so please read these introduction thoroughly for a proper usage. For detailed information of each package, please refer to the document in their folder.

# Packages

## 1. Lidar_align
A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor
To use this package, the input data should be a rosbag containing two msg types: 
sensor_msgs/PointCloud2 & geometry_msgs/TransformStamped
The output will be a calibration file and an aligned pointcloud map(ply format).

## 2. rr_localization 
subscribe /nax/fix && /imu/enu topics, output the poses as geometry_msgs/TransformStamped.

## 3. displayply
Read pointcloud in .ply format and display it.

## 4. imu-lidar
A modified version of hand eye calibration algorithm. It contains two modules: lidar-odometry front-end && dual-quaternion solver back-end. These two modules are designed as two nodes and can be used separately.
The first module 

## 5. ndt_tku_mapping 
Another lidar odometry method by ndt-tku. Can serve as the front-end in imu-lidar package.

## 6. ndt_tku 
ndt_tku library. Installed before using ndt_tku_mapping.

