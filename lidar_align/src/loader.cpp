#define PCL_NO_PRECOMPILE
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include "lidar_align/loader.h"
#include "lidar_align/transform.h"
#include "lidar_align/sensors.h"

#define foreach BOOST_FOREACH
namespace lidar_align {

Loader::Loader(const Config& config) : config_(config) {}

Loader::Config Loader::getConfig(ros::NodeHandle* nh) {
  Loader::Config config;
  nh->param("use_n_scans", config.use_n_scans, config.use_n_scans);
  nh->param("leaf_size", config.leaf_size, config.leaf_size);
  //std::cout << "config.leaf_size" << config.leaf_size <<std::endl;
  return config;
}

void Loader::parsePointcloudMsg(const sensor_msgs::PointCloud2 msg,
                                LoaderPointcloud* pointcloud) {

  // LoaderPointcloud* pointcloud_tmp;
  
  pcl::fromROSMsg(msg, *pointcloud);

  //confine scan, delete points less than the min range
  //std::cout << "original point cloud size:  "<< pointcloud->size() << std:: endl;
  double r;
  PointXYZIT p;
  LoaderPointcloud::Ptr confined_scan(new LoaderPointcloud);
  LoaderPointcloud::Ptr filtered_scan(new LoaderPointcloud);

  for(LoaderPointcloud::const_iterator item = pointcloud->begin(); item!= pointcloud->end(); item++){
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));

    if (r > 12.0 && r < 60.0)
      // (*pointcloud).push_back(p);
      confined_scan->push_back(p);
  }
  
  //std::cout << "confined scan size:  "<< confined_scan->size() << std:: endl;
  //filter the point coud using voxel
  //std::cout << "leaf size: " << config_.leaf_size << std::endl;
  pcl::VoxelGrid<PointXYZIT> voxel;
  voxel.setInputCloud(confined_scan);
  voxel.setLeafSize(config_.leaf_size, config_.leaf_size, config_.leaf_size);
  voxel.filter(*filtered_scan);

  //std::cout << "filtered pointcloud size:  "<< filtered_scan->size() << std:: endl;
  (*pointcloud).points = (*filtered_scan).points;
  // std::cout << "final pointcloud size:  "<< pointcloud->size() << std:: endl;

  //
  // bool has_timing = false;
  // bool has_intensity = false;
  // for (const sensor_msgs::PointField& field : msg.fields) {
  //   if (field.name == "time_offset_us") {
  //     has_timing = true;
  //   } else if (field.name == "intensity") {
  //     has_intensity = true;
  //   }
  // }

  // if (has_timing) {
  //   pcl::fromROSMsg(msg, *pointcloud);
  //   return;
  // } else if (has_intensity) {
  //   Pointcloud raw_pointcloud;
  //   pcl::fromROSMsg(msg, raw_pointcloud);

  //   for (const Point& raw_point : raw_pointcloud) {
  //     PointAllFields point;
  //     point.x = raw_point.x;
  //     point.y = raw_point.y;
  //     point.z = raw_point.z;
  //     point.intensity = raw_point.intensity;
  //     if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
  //         !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
  //       continue;
  //     }

  //     pointcloud->push_back(point);
  //   }
  //   pointcloud->header = raw_pointcloud.header;
  // } else {
  //   pcl::PointCloud<pcl::PointXYZ> raw_pointcloud;
  //   pcl::fromROSMsg(msg, raw_pointcloud);

  //   for (const pcl::PointXYZ& raw_point : raw_pointcloud) {
  //     PointAllFields point;
  //     point.x = raw_point.x;
  //     point.y = raw_point.y;
  //     point.z = raw_point.z;

  //     if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
  //         !std::isfinite(point.z)) {
  //       continue;
  //     }

  //     pointcloud->push_back(point);
  //   }
  //   pointcloud->header = raw_pointcloud.header;
  // }
}

bool Loader::loadPointcloudFromROSBag(const std::string& bag_path,
                                      const Scan::Config& scan_config,
                                      Lidar* lidar) {
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return false;
  }

  std::vector<std::string> types;
  types.push_back(std::string("sensor_msgs/PointCloud2"));
  rosbag::View view(bag, rosbag::TypeQuery(types));

  size_t scan_num = 0;
  for (const rosbag::MessageInstance& m : view) {
    std::cout << " Loading scan: \e[1m" << scan_num++ << "\e[0m from ros bag"
              << '\r' << std::flush;

    if (scan_num % 20 == 1){
        LoaderPointcloud pointcloud;
        parsePointcloudMsg(*(m.instantiate<sensor_msgs::PointCloud2>()),
                       &pointcloud);
        lidar->addPointcloud(pointcloud, scan_config);
    }


    if (lidar->getNumberOfScans() >= config_.use_n_scans) {
      break;
    }
  }
  std::cout << "lidar->getNumberOfScans():  " << lidar->getNumberOfScans() << std::endl;
  if (lidar->getTotalPoints() == 0) {
    ROS_ERROR_STREAM(
        "No points were loaded, verify that the bag contains populated "
        "messages of type sensor_msgs/PointCloud2");
    return false;
  }
  //std::cout << "number of scans:  " << scan_num++ << std::endl;
  return true;
}

bool Loader::loadTformFromROSBag(const std::string& bag_path, Odom* odom) {
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return false;
  }

  std::vector<std::string> types;
  types.push_back(std::string("geometry_msgs/TransformStamped"));
  rosbag::View view(bag, rosbag::TypeQuery(types));

  size_t tform_num = 0;
  for (const rosbag::MessageInstance& m : view) {
    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from ros bag" << '\r' << std::flush;

    geometry_msgs::TransformStamped transform_msg =
        *(m.instantiate<geometry_msgs::TransformStamped>());

    Timestamp stamp = transform_msg.header.stamp.sec * 1000000ll +
                      transform_msg.header.stamp.nsec / 1000ll;

    Transform T(Transform::Translation(transform_msg.transform.translation.x,
                                       transform_msg.transform.translation.y,
                                       transform_msg.transform.translation.z),
                Transform::Rotation(transform_msg.transform.rotation.w,
                                    transform_msg.transform.rotation.x,
                                    transform_msg.transform.rotation.y,
                                    transform_msg.transform.rotation.z));
    odom->addTransformData(stamp, T);
  }

  if (odom->empty()) {
    ROS_ERROR_STREAM("No odom messages found!");
    return false;
  }

  return true;
}

// bool Loader::loadTformFromROSBag(const std::string& bag_path, Odom* odom) {
//   rosbag::Bag bag;
//   try {
//     bag.open(bag_path, rosbag::bagmode::Read);
//   } catch (rosbag::BagException e) {
//     ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
//     return false;
//   }
//   std::cout << "Start loading!" << std::endl;
//   // std::vector<std::string> types;
//   // types.push_back(std::string("nav_msgs/Odometry"));
//   // types.push_back(std::string("sensor_msgs/Imu"));
//   // rosbag::View view(bag, rosbag::TypeQuery(types));

//   std::vector<std::string> topics;
//   topics.push_back(std::string("imu/enu"));
//   topics.push_back(std::string("odom"));

//   rosbag::View view(bag, rosbag::TopicQuery(topics));

//   std::cout << "Establish view!" << std::endl;
//   size_t tform_num = 0;
//   foreach (const rosbag::MessageInstance& m , view) {
//     // std::cout << " Loading transform: \e[1m" << tform_num++
//     //           << "\e[0m from ros bag" << '\r' << std::flush << std::endl;

//     sensor_msgs::Imu imu_msg =
//         *(m.instantiate<sensor_msgs::Imu>());
//     std::cout << "Loading imu msgs!:" << *(m.instantiate<sensor_msgs::Imu>()) << std::endl;

//     nav_msgs::Odometry odom_msg =
//         *(m.instantiate<nav_msgs::Odometry>());
//     std::cout << "Loading odom msgs!" << std::endl; 

   

//     Timestamp stamp = imu_msg.header.stamp.sec * 1000000ll +
//                       imu_msg.header.stamp.nsec / 1000ll;

//     Transform T(Transform::Translation(odom_msg.pose.pose.position.x,
//                                        odom_msg.pose.pose.position.y,
//                                        odom_msg.pose.pose.position.z),
//                 Transform::Rotation(imu_msg.orientation.w,
//                                     imu_msg.orientation.x,
//                                     imu_msg.orientation.y,
//                                     imu_msg.orientation.z));
//     odom->addTransformData(stamp, T);
//   }

//   if (odom->empty()) {
//     ROS_ERROR_STREAM("No odom messages found!");
//     return false;
//   }

//   return true;
// }

bool Loader::loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom) {
  std::ifstream file(csv_path, std::ifstream::in);

  size_t tform_num = 0;
  while (file.peek() != EOF) {
    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from csv file" << '\r' << std::flush;

    Timestamp stamp;
    Transform T;

    if (getNextCSVTransform(file, &stamp, &T)) {
      odom->addTransformData(stamp, T);
    }
  }

  return true;
}

// lots of potential failure cases not checked
bool Loader::getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                 Transform* T) {
  std::string line;
  std::getline(str, line);

  // ignore comment lines
  if (line[0] == '#') {
    return false;
  }

  std::stringstream line_stream(line);
  std::string cell;

  std::vector<std::string> data;
  while (std::getline(line_stream, cell, ',')) {
    data.push_back(cell);
  }

  if (data.size() < 9) {
    return false;
  }

  constexpr size_t TIME = 0;
  constexpr size_t X = 2;
  constexpr size_t Y = 3;
  constexpr size_t Z = 4;
  constexpr size_t RW = 5;
  constexpr size_t RX = 6;
  constexpr size_t RY = 7;
  constexpr size_t RZ = 8;

  *stamp = std::stoll(data[TIME]) / 1000ll;
  *T = Transform(Transform::Translation(std::stod(data[X]), std::stod(data[Y]),
                                        std::stod(data[Z])),
                 Transform::Rotation(std::stod(data[RW]), std::stod(data[RX]),
                                     std::stod(data[RY]), std::stod(data[RZ])));

  return true;
}

}  // namespace lidar_align
