#ifndef LIDAR_ALIGN_SENSORS_H_
#define LIDAR_ALIGN_SENSORS_H_
#define PCL_NO_PRECOMPILE
#include <random>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <pcl/features/normal_3d.h>
#include "lidar_align/transform.h"

namespace lidar_align {

typedef std::string LidarId;
// this must be at least 64 bit and signed or things will break
typedef long long int Timestamp;

// struct EIGEN_ALIGN16 PointAllFields {
//   PCL_ADD_POINT4D;
//   int32_t time_offset_us;
//   uint16_t reflectivity;
//   uint16_t intensity;
//   uint8_t ring;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };
struct PointXYZIT {
  PCL_ADD_POINT4D
  uint8_t intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
};

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Pointcloud;
typedef pcl::PointCloud<PointXYZIT> LoaderPointcloud;

class OdomTformData {
 public:
  OdomTformData(Timestamp timestamp_us, Transform T_o0_ot);

  const Transform& getTransform() const;
  const Timestamp& getTimestamp() const;

 private:
  Transform T_o0_ot_;
  Timestamp timestamp_us_;
};

class Odom {
 public:
  void addTransformData(const Timestamp& timestamp_us,
                        const Transform& transform);

  Transform getOdomTransform(const Timestamp timestamp_us,
                             const size_t start_idx = 0,
                             size_t* match_idx = nullptr) const;

  bool empty() { return data_.empty(); }

 private:
  std::vector<OdomTformData> data_;
};

class Scan {
 public:
  struct Config {
    float min_point_distance = 0.0;
    float max_point_distance = 100.0;
    float keep_points_ratio = 0.1;
    float min_return_intensity = -1.0;

    bool estimate_point_times = false;
    bool clockwise_lidar = false;
    bool motion_compensation = true;
    float lidar_rpm = 600.0;
  };

  Scan(const LoaderPointcloud& pointcloud, const Config& config);

  static Config getConfig(ros::NodeHandle* nh);

  void setOdomTransform(const Odom& odom, const double time_offset,
                        const size_t start_idx, size_t* match_idx, int* scan_num);

  const Transform& getOdomTransform() const;

  const Pointcloud& getRawPointcloud() const;

  void getTimeAlignedPointcloud(const Transform& T_o_l,
                                Pointcloud* pointcloud,
                                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
  void computeNormals();

  pcl::PointCloud<pcl::Normal> getCloudNormals();

 private:
  Timestamp timestamp_us_;  // signed to allow simpler comparisons
  Pointcloud raw_points_;
  std::vector<Transform>
      T_o0_ot_;  // absolute odom transform to each point in pointcloud
  pcl::PointCloud<pcl::Normal> cloud_normals;
  bool odom_transform_set_;
};

class Lidar {
 public:
  Lidar(const LidarId& lidar_id = "Lidar");

  const size_t getNumberOfScans() const;

  const size_t getTotalPoints() const;

  // note points are appended so any points in *pointcloud are preserved;
  void getCombinedPointcloud(Pointcloud* pointcloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) ;

  const LidarId& getId() const;

  void addPointcloud(const LoaderPointcloud& pointcloud,
                     const Scan::Config& config = Scan::Config());

  void setOdomOdomTransforms(const Odom& odom, const double time_offset = 0.0);

  void setOdomLidarTransform(const Transform& T_o_l);

  // used for debugging frames
  void saveCombinedPointcloud(const std::string& file_path);

  const Transform& getOdomLidarTransform() const;

 private:
  LidarId lidar_id_;
  Transform T_o_l_;  // transform from lidar to odometry

  std::vector<Scan> scans_;
};

}  // namespace lidar_align

#endif
