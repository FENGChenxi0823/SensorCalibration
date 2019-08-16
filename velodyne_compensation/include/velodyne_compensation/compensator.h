/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef VELODYNE_POINTCLOUD_COMPENSATOR_H_
#define VELODYNE_POINTCLOUD_COMPENSATOR_H_

// #include "velodyne_pointcloud/const_variables.h"

#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf2_ros/transform_listener.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "velodyne_compensation/transform.h"

// #define PCL_NO_PRECOMPILE

// typedef long long int Timestamp;

// struct PointXYZIT {
//   PCL_ADD_POINT4D
//   uint8_t intensity;
//   double timestamp;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
// };
// typedef pcl::PointCloud<PointXYZIT> LoaderPointcloud;


class Compensator {
 public:
  Compensator(ros::NodeHandle& node, ros::NodeHandle& private_nh);
  virtual ~Compensator() = default;

 private:
  /**
  * @brief get pointcloud2 msg, compensate it,publish pointcloud2 after
  * compensator
  */
  void pointcloudCallback(sensor_msgs::PointCloud2ConstPtr msg);
  /**
  * @brief get tform msg, save it as a vector for compensation
  */
  void transformCallback(const geometry_msgs::TransformStampedConstPtr& msg);
  /**
  * @brief get pose affine from tf by gps timestamp
  *  rr_localization-preprocess broadcast the tf transfrom.
  */
  // bool queryPoseAffineFromTf(const double timestamp,
                                  // Eigen::Affine3d& pose);
  // @brief get pose affine from gps transformation by pc timestamp
  Eigen::Affine3d getTransform(const double timestamp);

  // * @brief check if message is valid, check width, height, timestamp.
  // *   set timestamp_offset and point data type
  
  inline bool checkMessage(sensor_msgs::PointCloud2ConstPtr msg);
  // * @brief motion compensation for point cloud
  // template <typename Scalar>
  void motionCompensation(sensor_msgs::PointCloud2::Ptr& msg,
									        const double timestamp_min,
                          const double timestamp_max,
                          const Eigen::Affine3d& pose_min_time,
                          const Eigen::Affine3d& pose_max_time);
  // * @brief get min timestamp and max timestamp from points in pointcloud2
  
  inline void getTimestampInterval(
      sensor_msgs::PointCloud2ConstPtr msg, double& timestamp_min,
      double& timestamp_max);
  /**
  * @brief get point field size by sensor_msgs::datatype
  */
  inline uint getFieldSize(const int data_type);

  // subscribe velodyne pointcloud2 msg.
  ros::Subscriber pointcloud_sub_;
  // subscribe transform msg.
  ros::Subscriber transform_sub_;
  // publish point cloud2 after motion compensation
  ros::Publisher compensation_pub_;

  // struct trans{
  //   double timestamp_;
  //   Transform tform_;
  // }
  //variables store the transformation
  std::vector<std::pair <double, Transform>> trans_data_;

  // Transform T_ltoi_;

  // variables for point fields value, we get point x,y,z by these offset
  int x_offset_;
  int y_offset_;
  int z_offset_;
  int timestamp_offset_;
  uint timestamp_data_size_;

  // topic names
  std::string topic_compensated_pointcloud_;
  std::string topic_pointcloud_;
  std::string topic_transform_;
  // ros queue size for publisher and subscriber
  int queue_size_;
};

// POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_align::PointXYZIT,
//                                   (float, x, x)(float, y, y)(float, z, z)(
//                                       uint8_t, intensity,
//                                       intensity)(double, timestamp, timestamp))

#endif  // VELODYNE_POINTCLOUD_COMPENSATOR_H_