#include "lidar_align/sensors.h"

namespace lidar_align {

OdomTformData::OdomTformData(Timestamp timestamp_us, Transform T_o0_ot)
    : timestamp_us_(timestamp_us), T_o0_ot_(T_o0_ot) {}

const Transform& OdomTformData::getTransform() const { return T_o0_ot_; }

const Timestamp& OdomTformData::getTimestamp() const { return timestamp_us_; }

void Odom::addTransformData(const Timestamp& timestamp_us, const Transform& T) {
  data_.emplace_back(timestamp_us, T);
}

Transform Odom::getOdomTransform(const Timestamp timestamp_us,
                                 const size_t start_idx,
                                 size_t* match_idx) const {
  size_t idx = start_idx;

  while ((idx < (data_.size() - 1)) &&
         (timestamp_us > data_[idx].getTimestamp())) {
    ++idx;
  }
  if (idx > 0) {
    --idx;
  }

  if (match_idx != nullptr) {
    *match_idx = idx;
  }
  // std::cout << "scan timestamp" << timestamp_us << std::endl;
  // std::cout << "odom timestamp" << data_[idx].getTimestamp() << std::endl;
  // interpolate
  double t_diff_ratio =
      static_cast<double>(timestamp_us - data_[idx].getTimestamp()) /
      static_cast<double>(data_[idx + 1].getTimestamp() -
                          data_[idx].getTimestamp());
  // std:: cout <<"t_diff_ratio:  "<< t_diff_ratio << std::endl;
  Transform::Vector6 diff_vector =
      (data_[idx].getTransform().inverse() * data_[idx + 1].getTransform())
          .log();
  Transform out =
      data_[idx].getTransform() * Transform::exp(t_diff_ratio * diff_vector);

  // std::cout << "previous tform： " << data_[idx].getTransform().translation() <<std::endl;
  // std::cout <<" next tform： " << data_[idx+1].getTransform().translation()<<std::endl;
  // std::cout << " output transform： " << out.translation() << std::endl;

  return out;
}

Scan::Scan(const LoaderPointcloud& in, const Config& config)
    : timestamp_us_(in.header.stamp), odom_transform_set_(false) {
  std::default_random_engine generator(in.header.stamp);
  std::uniform_real_distribution<float> distribution(0, 1);

  //compute normals
  pcl::NormalEstimation<PointXYZIT, pcl::Normal> ne;
  LoaderPointcloud::Ptr cloud (new LoaderPointcloud);
  *cloud = in;
  // std:: cout << "cloud size: " << (*cloud).size() <<std::endl;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<PointXYZIT>::Ptr tree (new pcl::search::KdTree<PointXYZIT> ());
  ne.setSearchMethod (tree);
  ne.setViewPoint (0.0, 0.0, 1.0);
  pcl::PointCloud<pcl::Normal>::Ptr temp_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch (5);
  ne.compute (*temp_normals);

  // filtering and downsample
  for (size_t i = 0; i < in.size(); i++) {
    PointXYZIT point = in[i];
    if ((point.intensity > config.min_return_intensity) &&
        distribution(generator) < config.keep_points_ratio) {
      float sq_dist = point.x * point.x + point.y * point.y + point.z * point.z;
      if (std::isfinite(sq_dist) &&
          (sq_dist > (config.min_point_distance * config.min_point_distance)) &&
          (sq_dist < (config.max_point_distance * config.max_point_distance))) {
        Point store_point;
        store_point.x = point.x;
        store_point.y = point.y;
        store_point.z = point.z;
        store_point.intensity = point.intensity;

        // if (config.estimate_point_times) {
        //   // 100000 * 600 / pi
        //   const double timing_factor = 19098593.171 / config.lidar_rpm;
        //   const double angle = std::atan2(point.x, point.y);

        //   // cut out wrap zone
        //   if (std::abs(angle) > 3.0) {
        //     continue;
        //   }
        //   store_point.intensity = angle * timing_factor;
        //   if (!config.clockwise_lidar) {
        //     store_point.intensity *= -1.0;
        //   }
        // }
        raw_points_.push_back(store_point);
        cloud_normals.push_back((*temp_normals)[i]);
      }
    }
  }
  raw_points_.header = in.header;
}

Scan::Config Scan::getConfig(ros::NodeHandle* nh) {
  Scan::Config config;
  nh->param("min_point_distance", config.min_point_distance,
            config.min_point_distance);
  nh->param("max_point_distance", config.max_point_distance,
            config.max_point_distance);
  nh->param("keep_points_ratio", config.keep_points_ratio,
            config.keep_points_ratio);
  nh->param("min_return_intensity", config.min_return_intensity,
            config.min_return_intensity);

  nh->param("estimate_point_times", config.estimate_point_times,
            config.estimate_point_times);
  nh->param("clockwise_lidar", config.clockwise_lidar, config.clockwise_lidar);
  nh->param("motion_compensation", config.motion_compensation,
            config.motion_compensation);
  nh->param("lidar_rpm", config.lidar_rpm, config.lidar_rpm);

  return config;
}

void Scan::setOdomTransform(const Odom& odom, const double time_offset,
                            const size_t start_idx, size_t* match_idx, int* scan_num) {
  T_o0_ot_.clear();

  size_t i = 0;
  for (Point point : raw_points_) {
    // NOTE: currently the timestamp of a specific point is not propoerly set in the velodyne parser.
    // Here, we simply use the timesstamp_us_, which is a timestamp of one scan, to roughly estimate 
    // time of each point. This estimation is definelty not accurate enough. We can change it back to point timestamp
    // after the parser is corrected. -------by fengchenxi

    Timestamp point_ts_us = timestamp_us_;
    //Timestamp point_ts_us = point.timestamp;

    T_o0_ot_.push_back(
        odom.getOdomTransform(point_ts_us, start_idx, match_idx));
  }
  odom_transform_set_ = true;
}

const Transform& Scan::getOdomTransform() const {
  if (!odom_transform_set_) {
    throw std::runtime_error(
        "Attempted to get odom transform before it was set");
  }
  return T_o0_ot_.front();
}

void Scan::getTimeAlignedPointcloud(const Transform& T_o_l,
                                    Pointcloud* pointcloud,
                                    pcl::PointCloud<pcl::Normal>::Ptr clouds_normals) {
  pcl::PointCloud<pcl::Normal> normals = getCloudNormals();
  for (size_t i = 0; i < raw_points_.size(); ++i) {
    Transform T_o_lt = T_o0_ot_[i] * T_o_l;

    Eigen::Affine3f pcl_transform;

    pcl_transform.matrix() = T_o_lt.matrix();
    pointcloud->push_back(pcl::transformPoint(raw_points_[i], pcl_transform));
    //transform normal of each point
    Point tempP;
    tempP.x = normals.points[i].normal[0];
    tempP.y = normals.points[i].normal[1];
    tempP.z = normals.points[i].normal[2];
    Point transP = pcl::transformPoint(tempP, pcl_transform);
    pcl::Normal tempN;
    tempN.normal[0] = transP.x;
    tempN.normal[1] = transP.y;
    tempN.normal[2] = transP.z;
    clouds_normals->push_back(tempN);
  }
}

const Pointcloud& Scan::getRawPointcloud() const {return raw_points_; }
pcl::PointCloud<pcl::Normal> Scan::getCloudNormals(){return cloud_normals;}

void Scan::computeNormals(){
  pcl::NormalEstimation<Point, pcl::Normal> ne;
  Pointcloud::Ptr cloud (new Pointcloud);
  *cloud = getRawPointcloud();
  std:: cout << "cloud size: " << (*cloud).size() <<std::endl;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
  ne.setSearchMethod (tree);
  ne.setViewPoint (0.0, 0.0, 1.0);
  // pcl::PointCloud<pcl::Normal>::Ptr temp_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch (5);
  ne.compute (cloud_normals);
  // std::cout << "normal : " << cloud_normals.points[0].normal[0] << cloud_normals.points[0].normal[1]
  // << cloud_normals.points[0].normal[2] << std::endl;
}

Lidar::Lidar(const LidarId& lidar_id) : lidar_id_(lidar_id){};

const size_t Lidar::getNumberOfScans() const { return scans_.size(); }

const size_t Lidar::getTotalPoints() const {
  size_t num_points = 0;
  for (const Scan& scan : scans_) {
    num_points += scan.getRawPointcloud().size();
  }
  return num_points;
}

const LidarId& Lidar::getId() const { return lidar_id_; }

void Lidar::addPointcloud(const LoaderPointcloud& pointcloud,
                          const Scan::Config& config) {
  Scan scan(pointcloud,config);
  // scan.computeNormals();
  scans_.push_back(scan);
}

void Lidar::getCombinedPointcloud(Pointcloud* pointcloud, pcl::PointCloud<pcl::Normal>::Ptr clouds_normals) {
  //put all pointclouds into one pointcloud 
  for (Scan& scan : scans_) {
    scan.getTimeAlignedPointcloud(getOdomLidarTransform(), pointcloud, clouds_normals);
  }
}

void Lidar::saveCombinedPointcloud(const std::string& file_path) {
  Pointcloud combined;
  pcl::PointCloud<pcl::Normal>::Ptr clouds_normals (new pcl::PointCloud<pcl::Normal>);
  getCombinedPointcloud(&combined, clouds_normals);
  pcl::PLYWriter writer;
  writer.write(file_path, combined, true, false);
}

void Lidar::setOdomOdomTransforms(const Odom& odom, const double time_offset) {
  size_t idx = 0;
  int scan_num = 0;
  // std::cout <<"start set odom transform" << std::endl;
  // std:: cout << "scans size" << scans_.size() << std::endl;
  for (Scan& scan : scans_) {
    scan_num++;
    scan.setOdomTransform(odom, time_offset, idx, &idx, &scan_num);
  }
}

void Lidar::setOdomLidarTransform(const Transform& T_o_l) { T_o_l_ = T_o_l; }

const Transform& Lidar::getOdomLidarTransform() const { return T_o_l_; }

}  // namespace lidar_align
