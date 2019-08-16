# imu_lidar

This project aims to implement sensor calibration algorithms.

First: a modified version of hand eye calibration algorithms from github is implemented. It is based on this paper http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.18.366&rep=rep1&type=pdf


# Prerequisites
1. Eigen3
2. ceres
3. OpenCV 3


# Modules

## 1. Data exporter
Data exporter subscribes to sensor topics (such as lidar, imu, gnss), and exports the data for hand-eye calibration. It has the following functions:  
1. subscribe to sensor topics in a synchronized way
2. extracts poses from the imu and gnss
3. use ndt and icp to do point cloud tracking and get the transformation between two frames
4. save the corresponding transformation pairs of imu and gnss, and the lidar

#### Usage
First, play the recorded bag `rosbag play -l .bag`, then `roslaunch imu-lidar handeye_exporter.launch` will launch the node, subscribe to the specific topics, and save the data under folder "sensor_calibration/data". The transformation pairs and .pcd will be stored.


## 2. Hand-Eye calibration
This module reads the transformation pairs from the file, calibrates the (R,t) between the camera and the lidar. 
#### Usage
It does the followings: 

1. perform hand-eye and the refine the initial pose with nonlinear optimization method.
`roslaunch imu-lidar handeye_example.launch` starts the calibration node, read data from the file and write the calibrated result to the file.

# Parameters
## handeye_exporter.launch

	<arg name="mode" default="ndt" /> The pointcloud matching method including "ndt", "gicp", "icp"
    <arg name="ndt_data_path" default="$(find imu-lidar)/data/ndt" /> the directory to save the output result: hand eye tranformation pairs

    <arg name="odometryTopic" default="/gps_odometry" /> not used
    <arg name="gnssTopic" default="/nav/fix" /> not used
    <arg name="imuTopic" default="/imu/enu" /> not used
    <arg name="poseTopic" default="/pose" /> used
    <arg name="useOdoemetryTopic" default="false" />

    <arg name="lidarTopic1" default="/qml/sensor/vlp16/left/PointCloud2" /> 
    <arg name="lidarTopic2" default="/qml/sensor/vlp16/right/PointCloud2" />
    <arg name="lidarCount" default="1" /> if count=1, only the LidarTopic1 will be used
   
    <arg name="min_scan_range"  default="10.0" /> minimum confined scan range
    <arg name="max_scan_range"  default="80.0" /> maximum confined scan range
    <arg name="min_add_scan_shift" default="1.5" /> a distance threshold for adding the scan to the map
    <arg name="voxel_leaf_size" default="2.0" /> pcl downsample parameter

    <arg name="ndt_resolution" default="3.5" /> the size of ndt map voxels
    <arg name="fitness_tresh" default="100.0" /> a threshold for matching fitness score

    <arg name="max_corresp_distance" default="1.0" /> the maximum coresponding distance for icp or gicp matching  
    <arg name="trans_epsilon" default="1e-4" />
    <arg name="euc_fitness_epsilon" default="0.001" />
    <arg name="max_iter" default="30" />

## handeye_example
	<arg name="roundCount" default="1" /> The number of round for the entire process

    <arg name="hypothesesCount" default="256" /> The number of  
    <arg name="hypothesesSampleCount" default="10" /> The number of poses used for generating hypo
    <arg name="hypothesesMaxIters" default="256" /> The maximum iterations for computing a hypothese satisfying the threshold 
    <arg name="hypothesesPercentage" default="0.6" /> the percentage of poses satisfying the treshold

    <arg name="ransacSampleCount" default="60" /> the number of poses used for ransac refinement process
    <arg name="ransacPrunedCount" default="64" /> the number of hypothese kept for ransac refinement

    <arg name="rotationThreshold" default="5.0" /> the treshold of angle discrepancy between the hand poses and transformed eye poses
    <arg name="translationThreshold" default="0.5" />the treshold of distance discrepancy between the hand poses and transformed eye poses
