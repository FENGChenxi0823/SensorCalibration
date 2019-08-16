# Localization 
This package takes in gnss and imu data, and output relative transformations.
It serves as a preprocess of data for other projects.

Subscribe /nax/fix && /imu/enu topics, and advertise the computed poses as geometry_msgs/poseStamped and geometry_msgs/TransformStamped.

## Usgae 
'rosrun localization localization_node' 