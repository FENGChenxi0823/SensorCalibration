#include <ros/ros.h>
#include <localization/localization.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include "nav_msgs/Odometry.h"
#include <list>
#include <mutex>

ros::Publisher odom_pub, tform_pub,curr_pose_pub,curr_path_pub;
tf::Vector3 utm_base;
std::string frame_id, child_frame_id;
nav_msgs::Path current_path;

// void NavFixCallback(const sensor_msgs::NavSatFix::ConstPtr& in_gps) {
//   //std::cout<< "I heared it!!" << std::endl;
//   double utm[3], lat, lon, alt;
//   lat = in_gps->latitude;
//   lon = in_gps->longitude;
//   alt = in_gps->altitude;
//   std::string utm_zone = UTMZoneDesignator(lat, lon);
//   std::string utm_zone_str = utm_zone;
//   utm_zone_str.erase(utm_zone_str.end() - 1);
//   int utm_zone_num = std::stoi(utm_zone_str);
//   std::tuple<bool, tf::Vector3> frame_and_result;
//   frame_and_result = LatLongAltToUtm(lat, lon, alt, utm_zone_num);

//   tf::Vector3 utm_vec = std::get<1>(frame_and_result) - utm_base;
//   if (odom_pub) {
//     nav_msgs::Odometry odom;
//     odom.header.stamp = in_gps->header.stamp;
//     odom.header.frame_id = in_gps->header.frame_id;

//     odom.child_frame_id = child_frame_id;

//     odom.pose.pose.position.x = utm_vec[0];
//     odom.pose.pose.position.y = utm_vec[1];
//     odom.pose.pose.position.z = in_gps->altitude;
    
//     odom.pose.pose.orientation.x = 0;
//     odom.pose.pose.orientation.y = 0;
//     odom.pose.pose.orientation.z = 0;
//     odom.pose.pose.orientation.w = 1;
    
//     // Use ENU covariance to build XYZRPY covariance
//     boost::array<double, 36> covariance = {{
//       in_gps->position_covariance[0],
//       in_gps->position_covariance[1],
//       in_gps->position_covariance[2],
//       0, 0, 0,
//       in_gps->position_covariance[3],
//       in_gps->position_covariance[4],
//       in_gps->position_covariance[5],
//       0, 0, 0,
//       in_gps->position_covariance[6],
//       in_gps->position_covariance[7],
//       in_gps->position_covariance[8],
//       0, 0, 0,
//       0, 0, 0, rot_cov, 0, 0,
//       0, 0, 0, 0, rot_cov, 0,
//       0, 0, 0, 0, 0, rot_cov
//     }};

//     odom.pose.covariance = covariance;
//     //std::cout << "odom.pose.pose.position.x = " << odom.pose.pose.position.x << std::endl;
//     odom_pub.publish(odom);
//   }
//   
// }


void ImuAndOdomCallback(const sensor_msgs::Imu::ConstPtr& in_imu,const sensor_msgs::NavSatFix::ConstPtr& in_gps) {
  //std::cout<< "I heared it!!" << std::endl;
  double utm[3], lat, lon, alt;
  lat = in_gps->latitude;
  lon = in_gps->longitude;
  alt = in_gps->altitude;
  std::string utm_zone = UTMZoneDesignator(lat, lon);
  std::string utm_zone_str = utm_zone;
  utm_zone_str.erase(utm_zone_str.end() - 1);
  int utm_zone_num = std::stoi(utm_zone_str);
  std::tuple<bool, tf::Vector3> frame_and_result;
  frame_and_result = LatLongAltToUtm(lat, lon, alt, utm_zone_num);

  tf::Vector3 utm_vec = std::get<1>(frame_and_result) - utm_base;
  if (tform_pub) {
    geometry_msgs::TransformStamped tform;
    tform.header.stamp = in_gps->header.stamp;
    tform.header.frame_id = in_gps->header.frame_id;

    tform.child_frame_id = child_frame_id;

    tform.transform.translation.x = utm_vec[0];
    tform.transform.translation.y = utm_vec[1];
    tform.transform.translation.z = in_gps->altitude;
    
    tform.transform.rotation.x = in_imu->orientation.x;
    tform.transform.rotation.y = in_imu->orientation.y;
    tform.transform.rotation.z = in_imu->orientation.z;
    tform.transform.rotation.w = in_imu->orientation.w;
    //std::cout << "odom.pose.pose.position.x = " << odom.pose.pose.position.x << std::endl;
    tform_pub.publish(tform);
  }

  geometry_msgs::PoseStamped current_pose_msg;
  current_pose_msg.header.frame_id = in_gps->header.frame_id;
  current_pose_msg.header.stamp = in_gps->header.stamp;
  current_pose_msg.pose.position.x = utm_vec[0];
  current_pose_msg.pose.position.y = utm_vec[1];
  current_pose_msg.pose.position.z = in_gps->altitude;
  current_pose_msg.pose.orientation.x = in_imu->orientation.x;
  current_pose_msg.pose.orientation.y = in_imu->orientation.y;
  current_pose_msg.pose.orientation.z = in_imu->orientation.z;
  current_pose_msg.pose.orientation.w = in_imu->orientation.w;
  curr_pose_pub.publish(current_pose_msg);
  current_path.header.frame_id = in_gps->header.frame_id;
  current_path.header.stamp = in_gps->header.stamp;
  current_path.poses.push_back(current_pose_msg);
  curr_path_pub.publish(current_path);

}
int main (int argc, char **argv) {
  ros::init(argc, argv, "localization_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  std::string str_utm_base_;
  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param("utm_base", str_utm_base_, std::string("[216632.655397396 2497312.778132404 2.06519764]"));

  std::vector<std::string> vec_str = split(str_utm_base_, ", []");
  double x = std::atof(vec_str[0].c_str());
  double y = std::atof(vec_str[1].c_str());
  double z = std::atof(vec_str[2].c_str());
  utm_base.setX(x);
  utm_base.setY(y);
  utm_base.setZ(z);


  tform_pub = node.advertise<geometry_msgs::TransformStamped>("tform", 10);
  curr_path_pub =node.advertise<nav_msgs::Path>("path", 10);
  curr_pose_pub =node.advertise<geometry_msgs::PoseStamped>("pose", 10);

  #if 1
  message_filters::Subscriber<sensor_msgs::Imu> imu_sync_sub_(node, "/imu/enu", 1000);
  message_filters::Subscriber<sensor_msgs::NavSatFix> nav_fix_sub_(node, "/nav/fix", 1000);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::NavSatFix> MySyncPolicy_new;
  message_filters::Synchronizer<MySyncPolicy_new> sync_new_(MySyncPolicy_new(10), imu_sync_sub_, nav_fix_sub_);

  sync_new_.registerCallback(boost::bind(ImuAndOdomCallback, _1, _2));
  
  #endif
  ros::spin();
}