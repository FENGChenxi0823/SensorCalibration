#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <ros/ros.h>
#include <future>
#include <limits>
#include <nlopt.hpp>
#include <pcl/features/normal_3d.h>
#include "lidar_align/sensors.h"

namespace lidar_align {

class Aligner {
 public:
  struct Config {
    bool local;
    std::vector<double> inital_guess={};
    double max_time_offset;// = 0.1;
    double angular_range;// = 0.05;
    double translation_range;// = 0.1;

    double max_evals;// = 200;
    double xtol;// = 0.0001;

    int knn_batch_size;// = 1000;
    int knn_k;// = 15;
    float local_knn_max_dist;// = 1;
    float global_knn_max_dist;// = 1.0;
    bool time_cal;// = true;

    std::string mode = "";
    std::string output_pointcloud_path = "";
    std::string output_calibration_path = "";
  };

  struct OptData {
    Lidar* lidar;
    Odom* odom;
    Aligner* aligner;
    bool time_cal;
  };

  Aligner(const Config& config);

  static Config getConfig(ros::NodeHandle* nh);

  void lidarOdomTransform(Lidar* lidar, Odom* odom);

 private:
  void optimize(const std::vector<double>& lb, const std::vector<double>& ub,
                OptData* opt_data, std::vector<double>* x);

  std::string generateCalibrationString(const Transform& T,
                                        const double time_offset);

  static float kNNError(
      const std::string KNNMode,
      const pcl::KdTreeFLANN<Point>& kdtree, const Pointcloud& pointcloud,
      const pcl::PointCloud<pcl::Normal>::Ptr clouds_normals,
      const size_t k, const float max_dist, const size_t start_idx = 0,
      const size_t end_idx = std::numeric_limits<size_t>::max());

  float lidarOdomKNNError(const Pointcloud& base_pointcloud,
                          const Pointcloud& combined_pointcloud,
                          const pcl::PointCloud<pcl::Normal>::Ptr clouds_normals);

  float lidarOdomKNNError(Lidar& lidar);

  static double LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data);

  Config config_;
};

}  // namespace lidar_align

#endif  // LIDAR_ALIGN_ALIGNER_H_
