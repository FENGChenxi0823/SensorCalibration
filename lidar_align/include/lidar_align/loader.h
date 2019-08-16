#ifndef LIDAR_ALIGN_LOADER_H_
#define LIDAR_ALIGN_LOADER_H_

#include <pcl/point_types.h>
#include <ros/ros.h>

#include "lidar_align/sensors.h"

#define PCL_NO_PRECOMPILE

namespace lidar_align {

class Loader {
 public:
  struct Config {
    int use_n_scans = std::numeric_limits<int>::max();
    float leaf_size;
  };

  Loader(const Config& config);

  void parsePointcloudMsg(const sensor_msgs::PointCloud2 msg,
                          LoaderPointcloud* pointcloud);

  // void parseImuMsg(const sensor_msgs::Imu imu,
  //                   Transform* T);

  bool loadPointcloudFromROSBag(const std::string& bag_path,
                                const Scan::Config& scan_config, Lidar* lidar);

  // bool loadImuFromROSBag(const std::string& bag_path, Odom* odom);

  bool loadTformFromROSBag(const std::string& bag_path, Odom* odom);

  bool loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom);

  static Config getConfig(ros::NodeHandle* nh);

 private:
  static bool getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                  Transform* T);

  Config config_;
};
}  // namespace lidar_align

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_align::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      uint8_t, intensity,
                                      intensity)(double, timestamp, timestamp))

#endif  // LIDAR_ALIGN_ALIGNER_H_
