/*
  Normal Distributions Transform test program.

  2005/4/24 tku
*/

// number of cells
#define G_MAP_X 200
#define G_MAP_Y 200
#define G_MAP_Z 20
#define G_MAP_CELLSIZE 1.0

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <chrono>
#include <string>
#include <Eigen/Dense>
// #include <opencv2/core/persistence.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include "ceres/rotation.h"
#include "algebra.h"
#include "ndt.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <sstream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/rawdata.h"

/*grobal variables*/
NDMapPtr NDmap;
NDPtr NDs;
int NDs_num;

Point scan_points[130000];
double scan_points_i[130000];
int scan_points_num;

int is_first_time = 1;
int is_map_exist = 0;

double scan_points_weight[130000];
double scan_points_totalweight;

Point map_points[130000];
double map_points_i[130000];

int layer_select = LAYER_NUM - 1;

Posture prev_pose, prev_pose2;

// params
double g_map_center_x, g_map_center_y, g_map_center_z;
double g_map_rotation;
char g_ndmap_name[500];
int g_use_gnss;
int g_map_update = 1;
double g_ini_x, g_ini_y, g_ini_z, g_ini_roll, g_ini_pitch, g_ini_yaw;
std::string topic_compensated_pointcloud, data_path;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;  // tf between base_link and localizer
static tf::Quaternion q_local_to_global;
static Eigen::Matrix4f tf_local_to_global;
std::vector<Eigen::Matrix4f> transVecsGnss;
std::vector<Eigen::Matrix4f> transVecsLidar;

Posture sync_pose;
double sync_t = 0.1;

void save_nd_map(char *name);
void publish_nd_map();

static pcl::PointCloud<pcl::PointXYZ> map;

static ros::Publisher ndmap_pub;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;
static double exe_time = 0.0;

static ros::Publisher localizer_pose_pub, ndt_pose_pub, localizer_path_pub, ndt_path_pub;
static geometry_msgs::PoseStamped localizer_pose_msg, ndt_pose_msg;
static nav_msgs::Path localizer_path_msg, ndt_path_msg;

// double pose_mod(Posture *pose){
void pose_mod(Posture *pose)
{
  while (pose->theta < -M_PI)
    pose->theta += 2 * M_PI;
  while (pose->theta > M_PI)
    pose->theta -= 2 * M_PI;
  while (pose->theta2 < -M_PI)
    pose->theta2 += 2 * M_PI;
  while (pose->theta2 > M_PI)
    pose->theta2 -= 2 * M_PI;
  while (pose->theta3 < -M_PI)
    pose->theta3 += 2 * M_PI;
  while (pose->theta3 > M_PI)
    pose->theta3 -= 2 * M_PI;
}

double nrand(double n)
{
  double r;
  r = n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX);
  return r;
}

template <typename T>
Eigen::Matrix<T, 3, 3> QuaternionToRotation(const T* const q) {
    T R[9];
    ceres::QuaternionToRotation(q, R);

    Eigen::Matrix<T, 3, 3> rmat;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rmat(i, j) = R[i * 3 + j];
        }
    }

    return rmat;
}


void points_callback(const geometry_msgs::PoseStampedConstPtr& poseMsg, const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  const geometry_msgs::Quaternion& orien = poseMsg->pose.orientation;
  double orienTemp[4] = {orien.w, orien.x, orien.y, orien.z}; // scalar, i, j ,k

  Eigen::Matrix3d rot = QuaternionToRotation(orienTemp);
  // std::cout << "rot : \n" << rot <<std::endl;
  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  trans.block<3,3>(0,0) = rot.cast<float>();
  trans.block<3,1>(0,3) << poseMsg->pose.position.x,poseMsg->pose.position.y, poseMsg->pose.position.z;

  transVecsGnss.push_back(trans);

  matching_start = std::chrono::system_clock::now();
  static int count = 0;
  static tf::TransformBroadcaster br;
  static FILE *log_fp;
  ros::Time time;
  static tf::TransformListener listener;
  static ros::Time current_scan_time, current_pose_time;
  double diff_time;
  current_scan_time = msg->header.stamp;
  current_pose_time = poseMsg->header.stamp;
  diff_time = current_pose_time.toSec() - current_scan_time.toSec();
  std::cout << "current_scan_time:  " << current_scan_time << " current_pose_time: " << current_pose_time <<std::endl;
  
  static int iteration;
  int j = 0;

  Posture pose, bpose, initial_pose;
  static Posture key_pose;
  double e = 0;
  double x_offset, y_offset, z_offset, theta_offset;
  double distance;

  tf::Quaternion ndt_q, localizer_q;

  if (!log_fp)
    log_fp = fopen("/tmp/ndt_log", "w");

  count++;
  scan_points_totalweight = 0;

  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(*msg, scan);

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(0.5, 0.5, 0.5);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  j = 0;
  for (int i = 0; i < (int)filtered_scan_ptr->points.size(); i++)
  {
    scan_points[j].x = filtered_scan_ptr->points[i].x + nrand(0.01);
    scan_points[j].y = filtered_scan_ptr->points[i].y + nrand(0.01);
    scan_points[j].z = filtered_scan_ptr->points[i].z + nrand(0.01);
    scan_points_i[j] = 100;  // filterd_scan_ptr->points[i].intensity;

    double dist =
        scan_points[j].x * scan_points[j].x + scan_points[j].y * scan_points[j].y + scan_points[j].z * scan_points[j].z;
    if (dist < 3 * 3)
      continue;

    scan_points_weight[j] = 1;
    scan_points_totalweight += scan_points_weight[j];
    j++;
    if (j > 130000)
      break;
  }
  scan_points_num = j;

  /*--matching---*/
  // calc offset
  x_offset = prev_pose.x - prev_pose2.x;
  y_offset = prev_pose.y - prev_pose2.y;
  z_offset = prev_pose.z - prev_pose2.z;
  theta_offset = prev_pose.theta3 - prev_pose2.theta3;

  if (theta_offset < -M_PI)
    theta_offset += 2 * M_PI;
  if (theta_offset > M_PI)
    theta_offset -= 2 * M_PI;

  // calc estimated initial position
  pose.x = prev_pose.x + x_offset;
  pose.y = prev_pose.y + y_offset;
  pose.z = prev_pose.z + z_offset;
  pose.theta = prev_pose.theta;
  pose.theta2 = prev_pose.theta2;
  pose.theta3 = prev_pose.theta3 + theta_offset;

  initial_pose = pose;

  // matching
  for (layer_select = 1; layer_select >= 1; layer_select -= 1)
  {
    for (j = 0; j < 100; j++)
    {
      if (layer_select != 1 && j > 2)
      {
        break;
      }
      bpose = pose;

      e = adjust3d(scan_points, scan_points_num, &pose, layer_select);

      pose_mod(&pose);

      if ((bpose.x - pose.x) * (bpose.x - pose.x) + (bpose.y - pose.y) * (bpose.y - pose.y) +
              (bpose.z - pose.z) * (bpose.z - pose.z) + 3 * (bpose.theta - pose.theta) * (bpose.theta - pose.theta) +
              3 * (bpose.theta2 - pose.theta2) * (bpose.theta2 - pose.theta2) +
              3 * (bpose.theta3 - pose.theta3) * (bpose.theta3 - pose.theta3) <
          0.00001)
      {
        break;
      }
    }
    iteration = j;

    /*gps resetting*/
    if (g_use_gnss)
    {
      static FILE *e_fp;
      if (!e_fp)
      {
        e_fp = fopen("/tmp/e_log", "w");
      }
      fprintf(e_fp, "%f\n", e);
      if (layer_select == 1 && e < 1000)
      {
        printf("reset\n");
        tf::StampedTransform gps_tf_on_world;
        try
        {
          listener.lookupTransform("world", "gps", ros::Time(0), gps_tf_on_world);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
          return;
        }
        printf("set initial position by gps posiotion\n");
        pose.x = gps_tf_on_world.getOrigin().x();
        pose.y = gps_tf_on_world.getOrigin().y();
        pose.z = gps_tf_on_world.getOrigin().z() + nrand(5);
        tf::Quaternion q(gps_tf_on_world.getRotation().x(), gps_tf_on_world.getRotation().y(),
                         gps_tf_on_world.getRotation().z(), gps_tf_on_world.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        pose.theta = roll;
        pose.theta2 = pitch + 0.22;
        pose.theta3 = yaw + nrand(0.7);
        prev_pose2 = prev_pose = pose;
        printf("reset %f %f %f %f %f %f\n", pose.x, pose.y, pose.z, pose.theta, pose.theta2, pose.theta3);

        return;
      }
    }

    // unti-distotion
    if (layer_select == 2)
    {
      double rate, xrate, yrate, dx, dy, dtheta;
      double tempx, tempy;
      int i;

      tempx = (pose.x - prev_pose.x);
      tempy = (pose.y - prev_pose.y);
      dx = tempx * cos(-prev_pose.theta3) - tempy * sin(-prev_pose.theta3);
      dy = tempx * sin(-prev_pose.theta3) + tempy * cos(-prev_pose.theta3);
      dtheta = pose.theta3 - prev_pose.theta3;
      if (dtheta < -M_PI)
      {
        dtheta += 2 * M_PI;
      }
      if (dtheta > M_PI)
      {
        dtheta -= 2 * M_PI;
      }

      rate = dtheta / (double)scan_points_num;
      xrate = dx / (double)scan_points_num;
      yrate = dy / (double)scan_points_num;

      printf("untidist x %f y %f yaw %f\n", dx, dy, dtheta);

      dx = -dx;
      dy = -dy;
      dtheta = -dtheta;
      for (i = 0; i < scan_points_num; i++)
      {
        tempx = scan_points[i].x * cos(dtheta) - scan_points[i].y * sin(dtheta) + dx;
        tempy = scan_points[i].x * sin(dtheta) + scan_points[i].y * cos(dtheta) + dy;

        scan_points[i].x = tempx;
        scan_points[i].y = tempy;

        dtheta += rate;
        dx += xrate;
        dy += yrate;
      }
    }
  }

  // localizer
  Eigen::Translation3f translation(pose.x, pose.y, pose.z);
  Eigen::AngleAxisf rotation_x(pose.theta, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(pose.theta2, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(pose.theta3, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f local_t = (translation * rotation_z * rotation_y * rotation_x).matrix();
  Eigen::Matrix4f global_t = tf_local_to_global * local_t;
  // transVecsLidar.push_back(global_t);
  // std::cout << "global t : \n" << global_t <<std::endl;
  // //sync_localizer
  sync_pose.x = pose.x + diff_time/sync_t * (pose.x -prev_pose.x);
  sync_pose.y = pose.y + diff_time/sync_t * (pose.y -prev_pose.y);
  sync_pose.z = pose.z + diff_time/sync_t * (pose.z -prev_pose.z);
  sync_pose.theta = pose.theta;
  sync_pose.theta2 = pose.theta2;
  sync_pose.theta3 = pose.theta3 + diff_time/sync_t * (pose.theta3 -prev_pose.theta3);
  Eigen::Translation3f sync_translation(sync_pose.x, sync_pose.y, sync_pose.z);
  Eigen::AngleAxisf sync_rotation_x(sync_pose.theta, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf sync_rotation_y(sync_pose.theta2, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf sync_rotation_z(sync_pose.theta3, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f sync_local_t = (sync_translation * sync_rotation_z * sync_rotation_y * sync_rotation_x).matrix();
  Eigen::Matrix4f sync_global_t = tf_local_to_global * sync_local_t;
  transVecsLidar.push_back(sync_global_t);
  // std::cout << "sync_global_t : \n" << sync_global_t <<std::endl;

  tf::Matrix3x3 mat_l;
  mat_l.setValue(
      static_cast<double>(global_t(0, 0)), static_cast<double>(global_t(0, 1)), static_cast<double>(global_t(0, 2)),
      static_cast<double>(global_t(1, 0)), static_cast<double>(global_t(1, 1)), static_cast<double>(global_t(1, 2)),
      static_cast<double>(global_t(2, 0)), static_cast<double>(global_t(2, 1)), static_cast<double>(global_t(2, 2)));

  mat_l.getRotation(localizer_q);
  localizer_pose_msg.header.frame_id = "/imu_link";
  localizer_path_msg.header.frame_id = "/imu_link";
  localizer_pose_msg.header.stamp = current_scan_time;
  localizer_pose_msg.pose.position.x = global_t(0, 3);
  localizer_pose_msg.pose.position.y = global_t(1, 3);
  localizer_pose_msg.pose.position.z = global_t(2, 3);
  localizer_pose_msg.pose.orientation.x = localizer_q.x();
  localizer_pose_msg.pose.orientation.y = localizer_q.y();
  localizer_pose_msg.pose.orientation.z = localizer_q.z();
  localizer_pose_msg.pose.orientation.w = localizer_q.w();
  localizer_path_msg.poses.push_back(localizer_pose_msg);
  // base_link
  Eigen::Matrix4f global_t2 = global_t * tf_ltob;
  tf::Matrix3x3 mat_b;  // base_link
  mat_b.setValue(
      static_cast<double>(global_t2(0, 0)), static_cast<double>(global_t2(0, 1)), static_cast<double>(global_t2(0, 2)),
      static_cast<double>(global_t2(1, 0)), static_cast<double>(global_t2(1, 1)), static_cast<double>(global_t2(1, 2)),
      static_cast<double>(global_t2(2, 0)), static_cast<double>(global_t2(2, 1)), static_cast<double>(global_t2(2, 2)));
  mat_b.getRotation(ndt_q);

  ndt_pose_msg.header.frame_id = "/imu_link";
  ndt_path_msg.header.frame_id = "/imu_link";
  ndt_pose_msg.header.stamp = current_scan_time;
  ndt_pose_msg.pose.position.x = global_t2(0, 3);
  ndt_pose_msg.pose.position.y = global_t2(1, 3);
  ndt_pose_msg.pose.position.z = global_t2(2, 3);
  ndt_pose_msg.pose.orientation.x = ndt_q.x();
  ndt_pose_msg.pose.orientation.y = ndt_q.y();
  ndt_pose_msg.pose.orientation.z = ndt_q.z();
  ndt_pose_msg.pose.orientation.w = ndt_q.w();
  ndt_path_msg.poses.push_back(ndt_pose_msg);

  localizer_pose_pub.publish(localizer_pose_msg);
  ndt_pose_pub.publish(ndt_pose_msg);
  localizer_path_pub.publish(localizer_path_msg);
  ndt_path_pub.publish(ndt_path_msg);
  scan_transrate(scan_points, map_points, &pose, scan_points_num);

  for (int i = 0; i < scan_points_num; i++)
  {
    map_points_i[i] = scan_points_i[i];
  }

  // update ND map
  distance = (key_pose.x - pose.x) * (key_pose.x - pose.x) + (key_pose.y - pose.y) * (key_pose.y - pose.y) +
             (key_pose.z - pose.z) * (key_pose.z - pose.z);

  if (g_map_update && (!is_map_exist || (distance > 0.1 * 0.1 && scan_points_num > 100)))
  {
    int i;
    for (i = 0; i < scan_points_num; i++)
    {
      add_point_map(NDmap, &map_points[i]);
    }
    key_pose = pose;
    is_map_exist = 1;
    publish_nd_map();
  }

  prev_pose2 = prev_pose;
  prev_pose = pose;

  if (is_first_time)
  {
    prev_pose2 = prev_pose;
    is_first_time = 0;
  }

  fprintf(log_fp, "%f %f %f %f %f %f %f\n", current_scan_time.toSec(),
          pose.x * cos(g_map_rotation) - pose.y * sin(g_map_rotation) + g_map_center_x,
          pose.x * sin(g_map_rotation) + pose.y * cos(g_map_rotation) + g_map_center_y, pose.z + g_map_center_z,
          pose.theta, pose.theta2, pose.theta3 + g_map_rotation);

  fflush(log_fp);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(global_t2(0, 3), global_t2(1, 3), global_t2(2, 3)));
  transform.setRotation(ndt_q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "imu_link", "base_link"));

  matching_end = std::chrono::system_clock::now();
  exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << msg->header.seq << std::endl;
  std::cout << "Number of filtered scan points: " << scan_points_num << " points." << std::endl;
  std::cout << "Number of iteration: " << iteration << std::endl;
  std::cout << "Execution time: " << exe_time << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.theta << ", " << pose.theta2 << ", "
            << pose.theta3 << ")" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void save_nd_map(char *name)
{
  int i, j, k, layer;
  NDData nddat;
  NDMapPtr ndmap;
  NDPtr *ndp;
  FILE *ofp;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ p;

  ndmap = NDmap;
  ofp = fopen(name, "w");

  for (layer = 0; layer < 2; layer++)
  {
    ndp = ndmap->nd;
    for (i = 0; i < ndmap->x; i++)
    {
      for (j = 0; j < ndmap->y; j++)
      {
        for (k = 0; k < ndmap->z; k++)
        {
          if (*ndp)
          {
            update_covariance(*ndp);
            nddat.nd = **ndp;
            nddat.x = i;
            nddat.y = j;
            nddat.z = k;
            nddat.layer = layer;

            fwrite(&nddat, sizeof(NDData), 1, ofp);

            // regist the point to pcd data;
            p.x = (*ndp)->mean.x;
            p.y = (*ndp)->mean.y;
            p.z = (*ndp)->mean.z;
            cloud.points.push_back(p);
          }
          ndp++;
        }
      }
    }
    ndmap = ndmap->next;
  }
  fclose(ofp);

  // save pcd

  cloud.header.frame_id = "/imu_link";
  cloud.width = cloud.points.size();
  cloud.height = 1;
  pcl::io::savePCDFileASCII("/tmp/ndmap.pcd", cloud);
  printf("NDMap points num: %d points.\n", (int)cloud.points.size());

}

void publish_nd_map()
{
  std::cout << "publish map \n"; 
  int i, j, k, layer;
  NDData nddat;
  NDMapPtr ndmap;
  NDPtr *ndp;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ p;

  ndmap = NDmap;

  for (layer = 0; layer < 2; layer++)
  {
    ndp = ndmap->nd;
    for (i = 0; i < ndmap->x; i++)
    {
      for (j = 0; j < ndmap->y; j++)
      {
        for (k = 0; k < ndmap->z; k++)
        {
          if (*ndp)
          {
            update_covariance(*ndp);
            nddat.nd = **ndp;
            nddat.x = i;
            nddat.y = j;
            nddat.z = k;
            nddat.layer = layer;
            // regist the point to pcd data;
            p.x = (*ndp)->mean.x;
            p.y = (*ndp)->mean.y;
            p.z = (*ndp)->mean.z;
            cloud.points.push_back(p);
          }
          ndp++;
        }
      }
    }
    ndmap = ndmap->next;
  }

  cloud.header.frame_id = "/imu_link";
  cloud.width = cloud.points.size();
  cloud.height = 1;
  std::cout << "ndt map size ： " << cloud.points.size() <<std::endl;

  sensor_msgs::PointCloud2::Ptr ndmap_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(cloud, *ndmap_ptr);
  ndmap_pub.publish(*ndmap_ptr);
}

void serializeTransformationPairs(const std::vector<Eigen::Matrix4f>& transHand, const std::vector<Eigen::Matrix4f>& transEye, std::string& filePath)
{
    boost::filesystem::path dir(filePath);
    if(!boost::filesystem::is_directory(dir))
    {
        boost::filesystem::create_directory(dir);
        if(!boost::filesystem::is_directory(dir))
        {
            std::cout << "Failed to create directory: " << filePath << std::endl;
            return;
        }
    }

    cv::FileStorage fs(filePath+"/transformationParis.yaml", cv::FileStorage::WRITE);

    if(fs.isOpened())
    {
        fs << "HandOdometry" << "[";
        for(unsigned int i=0; i< transHand.size(); i++)
        {
            cv::Mat cvMat(4,4,CV_64F);
            cv::eigen2cv(transHand[i], cvMat);
            fs << cvMat;
        }
        fs << "]";

        fs << "EyeOdometry" << "[";
        for(unsigned int i=0; i< transEye.size(); i++)
        {
            cv::Mat cvMat(4,4,CV_64F);
            cv::eigen2cv(transEye[i], cvMat);
            fs << cvMat;
        }
        fs << "]";

        fs.release();
    }
}


int main(int argc, char *argv[])
{
  std::cout << " 3D NDT scan mapping" << std::endl;

  ros::init(argc, argv, "ndt_mapping_tku");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("init_x", g_ini_x);
  private_nh.getParam("init_y", g_ini_y);
  private_nh.getParam("init_z", g_ini_z);
  private_nh.getParam("init_roll", g_ini_roll);
  private_nh.getParam("init_pitch", g_ini_pitch);
  private_nh.getParam("init_yaw", g_ini_yaw);
  private_nh.getParam("topic_compensated_pointcloud", topic_compensated_pointcloud);
  private_nh.getParam("datapath", data_path);


  if (!private_nh.getParam("tf_x", _tf_x))
  {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (!private_nh.getParam("tf_y", _tf_y))
  {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (!private_nh.getParam("tf_z", _tf_z))
  {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (!private_nh.getParam("tf_roll", _tf_roll))
  {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (!private_nh.getParam("tf_pitch", _tf_pitch))
  {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (!private_nh.getParam("tf_yaw", _tf_yaw))
  {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  std::cout << "tf_btol: \n" << tf_btol << std::endl;

  Eigen::Translation3f tl_ltob((-1.0) * _tf_x, (-1.0) * _tf_y, (-1.0) * _tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * _tf_roll, Eigen::Vector3f::UnitX());     // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * _tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * _tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();
  std::cout << "tf_ltob: \n" << tf_ltob << std::endl;

  // map path
  sprintf(g_ndmap_name, "%s", "ndmap");

  // map size
  g_map_x = G_MAP_X;
  g_map_y = G_MAP_Y;
  g_map_z = G_MAP_Z;
  g_map_cellsize = G_MAP_CELLSIZE;
  // map center
  g_map_center_x = g_ini_x;
  g_map_center_y = g_ini_y;
  g_map_center_z = g_ini_z;
  g_map_rotation = 0.0;
  // use gnss
  g_use_gnss = 0;

  Eigen::Translation3f tl_local_to_global(g_map_center_x, g_map_center_y, g_map_center_z);  // tl: translation
  Eigen::AngleAxisf rot_x_local_to_global(0.0, Eigen::Vector3f::UnitX());                   // rot: rotation
  Eigen::AngleAxisf rot_y_local_to_global(0.0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_local_to_global(g_map_rotation, Eigen::Vector3f::UnitZ());
  q_local_to_global.setRPY(0.0, 0.0, g_map_rotation);
  tf_local_to_global =
      (tl_local_to_global * rot_z_local_to_global * rot_y_local_to_global * rot_x_local_to_global).matrix();
  std::cout << "tf_local_to_global: \n" << tf_local_to_global << std::endl;

  /*initialize(clear) NDmap data*/
  NDmap = initialize_NDmap();

  // load map
  prev_pose.x = (g_ini_x - g_map_center_x) * cos(-g_map_rotation) - (g_ini_y - g_map_center_y) * sin(-g_map_rotation);
  prev_pose.y = (g_ini_x - g_map_center_x) * sin(-g_map_rotation) + (g_ini_y - g_map_center_y) * cos(-g_map_rotation);
  prev_pose.z = g_ini_z - g_map_center_z;
  prev_pose.theta = g_ini_roll;
  prev_pose.theta2 = g_ini_pitch;
  prev_pose.theta3 = g_ini_yaw - g_map_rotation;

  prev_pose2 = prev_pose;
  is_first_time = 1;

  ndmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndmap", 1000);
  ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1000);
  localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 1000);
  ndt_path_pub =nh.advertise<nav_msgs::Path>("/ndt_path", 1000);
  localizer_path_pub = nh.advertise<nav_msgs::Path>("localizer_path",1000);

  // ros::Subscriber points_sub = nh.subscribe(topic_compensated_pointcloud, 1000, points_callback);
  message_filters::Subscriber<geometry_msgs::PoseStamped> poseSub(nh, "/pose", 1000);
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidarSub1(nh, topic_compensated_pointcloud, 1000);
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> syncPolicy;
  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(1000), poseSub, lidarSub1);
  sync.registerCallback(boost::bind(&points_callback, _1, _2));

  ros::spin();

  if(ros::isShuttingDown()){

    std::cout << "----------------------Saving files----------------------" << std::endl;
    serializeTransformationPairs(transVecsGnss, transVecsLidar, data_path);
    save_nd_map((char *)"/tmp/ndmap");
    std::cout << "----------------------Files saved----------------------" << std::endl;
  }
  
  return 1;
}
