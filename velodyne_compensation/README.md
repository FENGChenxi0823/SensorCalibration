#Velodyne_compensation

This package is originated from the apollo velodyne drivers with some changes.
Since the gnss device is not avaliable for the velodyne lidar, it cannot listen to the up-to-date poses. Alternatively, it wraps the current pointcloud2_msg and use the gnss+imu information to compensate the distortion.

Specifically, it subscribes 'Geometry_msgs/TransformStamped' topic and synchronize with pointcloud2_msgs. By linearly interpolate the 'TranformStamped', it compensate the distortion due to motion.

"Note: Inside the lidar driver, each lidar point has its inherent timestamp, but it is not known to us right now. As we can see, the linear interpolation here is just an unsupported way of process. This flaw makes this package a temporary, even invalid, solution for motion compensation " 

## Source 
https://github.com/ApolloAuto/apollo/tree/r2.5.0/modules/drivers/velodyne/velodyne_pointcloud/src