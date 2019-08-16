#include "velodyne_compensation/compensator.h"

#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ROS_INFO("Point cloud node init");
  ros::init(argc, argv, "velodyne_compensation_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  Compensator compensator(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  ros::shutdown();

  return 0;
}