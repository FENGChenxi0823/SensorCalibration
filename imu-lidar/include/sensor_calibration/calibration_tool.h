/*Created by Chen Xia, on Sep 1, 2017 */

#ifndef CALIBRATION_TOOL_H_
#define CALIBRATION_TOOL_H_

#include <fstream>
#include <string>
#include <sys/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

struct Pose
{
  double x, y, z, roll, pitch, yaw;
  Pose(): x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {}
};

void copyPose(Pose &new_pose, Pose &old_pose, std::string mode)
{
  if (mode == "xyz")
  {
    new_pose.x = old_pose.x;
    new_pose.y = old_pose.y;
    new_pose.z = old_pose.z;
  } else if (mode == "rpy"){
    new_pose.roll = old_pose.roll;
    new_pose.pitch = old_pose.pitch;
    new_pose.yaw = old_pose.yaw;
  }
}

void copyPose(Pose &new_pose, Pose &old_pose)
{
  copyPose(new_pose, old_pose, "xyz");
  copyPose(new_pose, old_pose, "rpy");
}


Pose copyPose(double *pos)
{
  Pose new_pose;
  new_pose.x = pos[0];
  new_pose.y = pos[1];
  new_pose.z = pos[2];
  new_pose.roll = pos[3];
  new_pose.pitch = pos[4];
  new_pose.yaw = pos[5];
  return new_pose;
}

Pose copyPose(double x, double y, double z, double roll, double pitch, double yaw)
{
  Pose new_pose;
  new_pose.x = x;
  new_pose.y = y;
  new_pose.z = z;
  new_pose.roll = roll;
  new_pose.pitch = pitch;
  new_pose.yaw = yaw;
  return new_pose;
}

Pose addPose(Pose pose1, Pose pose2, std::string mode)
{
  Pose new_pose;
  new_pose.x = pose1.x + pose2.x;
  new_pose.y = pose1.y + pose2.y;
  new_pose.z = pose1.z + pose2.z;
  new_pose.yaw = pose1.yaw + pose2.yaw;

  if (mode == "rpy"){
    new_pose.roll = pose1.roll + pose2.roll;
    new_pose.pitch = pose1.pitch + pose2.pitch;
  } else if (mode == "y"){
    new_pose.roll = pose1.roll;
    new_pose.pitch = pose1.pitch;
  } else if (mode == "00y"){
    new_pose.roll = 0;
    new_pose.pitch = 0;
  }
  return new_pose;
}

void pointcloudToPCD(pcl::PointCloud<pcl::PointXYZI> pcl_map, double downsample_pcd_voxel_leaf_size, int sequence)
{
  time_t tt = time(NULL);
  tm* t= localtime(&tt);
  std::stringstream name;
  name << t->tm_year + 1900 << "-" << t->tm_mon + 1 << "-" << t->tm_mday << "_" << sequence << ".pcd";
  std::string filename = name.str(); 
  std::string homedir = getenv("HOME");
  std::string filepath = homedir + "/Desktop/" + filename;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_map_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_map));
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  if (downsample_pcd_voxel_leaf_size != 0){
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(downsample_pcd_voxel_leaf_size, downsample_pcd_voxel_leaf_size, downsample_pcd_voxel_leaf_size);
    voxel_grid_filter.setInputCloud(pcl_map_ptr);
    voxel_grid_filter.filter(*pcl_map_filtered);    
  }
  else{   // downsample_pcd_voxel_leaf_size = 0
    pcl_map_filtered = pcl_map_ptr;
  }
  // pcl::io::savePCDFileASCII(filepath, *pcl_map_filtered);
  pcl::io::savePCDFileBinaryCompressed(filepath, *pcl_map_filtered);
  std::cout << "Saved " << pcl_map_filtered->points.size() << " data points to " << filename << "." << std::endl;
}


std::vector<std::string> split(std::string str, std::string delims)
{
  // str.erase(str.begin()); str.erase(str.end()-1);
  std::vector<std::string> ret;
  if (delims.empty()) return ret;
  size_t start = 0, index = str.find_first_of(delims, 0);
  while (index != str.npos){
    if (start != index)
      ret.push_back(str.substr(start, index - start));
    start = index + 1;
    index = str.find_first_of(delims, start);
  }
  if (!str.substr(start).empty())
    ret.push_back(str.substr(start));
  return ret;
}


std::vector<double> ned2enu(geometry_msgs::Quaternion q_ned)
{
  double roll_ned, pitch_ned, yaw_ned;
  tf::Quaternion q(q_ned.x, q_ned.y, q_ned.z, q_ned.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_ned, pitch_ned, yaw_ned); 
  double roll_enu, pitch_enu, yaw_enu;
  std::vector<double> rpy_enu;
  // roll_enu = pitch_ned - M_PI/2;  ///////////////////////////
  // if (roll_enu < -M_PI)
  //   roll_enu += M_PI*2;
  roll_enu = roll_ned;

  // pitch_enu = roll_ned + M_PI/2; /////////////////////////////
  // if (pitch_enu > M_PI)
  //   pitch_enu -= M_PI*2;
  pitch_enu = -pitch_ned;

  rpy_enu.push_back(roll_enu);
  rpy_enu.push_back(pitch_enu);

  yaw_enu = M_PI/2 - yaw_ned;
  if (yaw_enu > M_PI)
    yaw_enu = yaw_enu - 2 * M_PI;

  rpy_enu.push_back(yaw_enu);
  return rpy_enu;
}

double get_average_height(double x, double y, std::vector<double> x_list, std::vector<double> y_list, std::vector<double> z_list, double range)
{
  int t = 0;
  double average_z = 0;
  if (x_list.size() > 0)
  {
    for (unsigned int i = 0; i < x_list.size(); i++)
    {
      if (std::fabs(x - x_list.at(i)) <= range && std::fabs(y - y_list.at(i)) <= range){
        average_z = average_z + z_list.at(i); 
        t = t+1;
      }
    }
    average_z = average_z / t;
  }
  return average_z;
}

void record_path_file(std::ofstream &filename, double x, double y, double z, double roll, double pitch, double yaw)
{
  if (filename)
  {
    roll*= 180/M_PI;
    pitch *= 180/M_PI;
    yaw *= 180/M_PI;
    filename << std::setprecision(11) << x << " " << std::setprecision(11) << y << " " << std::setprecision(11) << z << " "
              << roll << " " << pitch << " " << yaw << std::endl;
  }   
}

void record_path_file(std::ofstream &filename, double x, double y, double z)
{
  if (filename){ 
    filename << std::setprecision(12) << x << " " << std::setprecision(12) << y << " " << std::setprecision(12) << z << std::endl;
  }
}


Eigen::Matrix4f computeTransformMatrix(double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());   
  Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f tl(x, y, z);     
  Eigen::Matrix4f transform_matrix = (tl * rot_z * rot_y * rot_x).matrix(); 
  return transform_matrix;
}


double* compute6DOFfromTransformEigen(Eigen::Matrix4f transform)
{
  static double pos[6];
  tf::Matrix3x3 mat_tf;
  mat_tf.setValue(static_cast<double>(transform(0, 0)), static_cast<double>(transform(0, 1)), static_cast<double>(transform(0, 2)), 
                  static_cast<double>(transform(1, 0)), static_cast<double>(transform(1, 1)), static_cast<double>(transform(1, 2)),
                  static_cast<double>(transform(2, 0)), static_cast<double>(transform(2, 1)), static_cast<double>(transform(2, 2)));
  pos[0] = transform(0, 3);
  pos[1] = transform(1, 3);
  pos[2] = transform(2, 3);
  mat_tf.getRPY(pos[3], pos[4], pos[5], 1);
  return pos;
}


double* computeCovariance(int n, double mu_old, double mu2_old, double new_ele)
{
  static double result[3];  // result[0] = new_cov, result[1] = new_mu, result[2] = new_mu2;
  double mu_new = (mu_old * (n - 1) + new_ele) / n;
  double mu2_new = (mu2_old * (n - 1) + pow(new_ele, 2)) / n;
  double cov_new = mu2_new - pow(mu_new, 2);
  result[0] = cov_new;
  result[1] = mu_new;
  result[2] = mu2_new;
  return result;
}

pcl::PointCloud<pcl::PointXYZI> confinePointCloudByDimension(pcl::PointCloud<pcl::PointXYZI> original_scan, double pos_x, double pos_y, double min_scan_range, double max_scan_range)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> confined_scan;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = original_scan.begin(); item != original_scan.end(); item++){
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;
    r = sqrt(pow(p.x - pos_x, 2.0) + pow(p.y - pos_y, 2.0));
    
    if (r > min_scan_range && r < max_scan_range)
      confined_scan.push_back(p);
  }
  return confined_scan;
}

pcl::PointCloud<pcl::PointXYZI> confinePointCloudByDimension(pcl::PointCloud<pcl::PointXYZI> original_scan, double* pos, double* d_ran, double* x_ran, double* y_ran, double* z_ran)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI>  confined_scan;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = original_scan.begin(); item != original_scan.end(); item++){
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;
    r = sqrt(pow(p.x - pos[0], 2.0) + pow(p.y - pos[1], 2.0));
    double abs_x = std::fabs(p.x);
    double abs_y = std::fabs(p.y);
    double abs_z = std::fabs(p.z);

    if (r >= d_ran[0] && r <= d_ran[1]){
      if ((abs_x >= x_ran[0] && abs_x <= x_ran[1]) || (abs_y >= y_ran[0] && abs_y <= y_ran[1])){
        if (abs_z >= z_ran[0] && abs_z <= z_ran[1]){
          confined_scan.push_back(p);
        }
      }
    }
  }
  return confined_scan;
}

pcl::PointCloud<pcl::PointXYZI> confinePointCloudByDimension(pcl::PointCloud<pcl::PointXYZI> original_scan, double min_scan_range)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI>  confined_scan;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = original_scan.begin(); item != original_scan.end(); item++){
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    
    if (r > min_scan_range)
      confined_scan.push_back(p);
  }
  return confined_scan;
}

pcl::PointCloud<pcl::PointXYZI> downsamplePointCloud(pcl::PointCloud<pcl::PointXYZI> original, double voxel_leaf_size)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(original));
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());  
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  return *filtered_scan_ptr;
}


void printMatrix4f(Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


Eigen::Matrix4f computeTransformaMatrixFromPose(double x, double y, double z, double qx, double qy, double qz, double qw)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3,1>(0,3) << x, y, z;
    Eigen::Quaterniond q(qw, qx, qy, qz);
    transform.block<3,3>(0,0) = q.toRotationMatrix();

    return transform.cast<float>();
}

#endif // #ifndef CALIBRATION_TOOL_H_
