/*
 * Descriptiong: Data exporter of odometry info and point cloud
 * Author: Ran Tang
 * Date: May 10, 2018
*/

#include <iostream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <limits>
#include <algorithm>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
// #include <pcl/surface/mls.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/gicp.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>

#include "sensor_calibration/EigenUtils.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "wgs_conversions/wgs_conversions.h"
#include "sensor_calibration/calibration_tool.h"


struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

std::string dataPath;
std::string mode;

std::string odometryTopic, gnssTopic, imuTopic, poseTopic;
bool useOdoemetryTopic;

std::string lidarTopic1, lidarTopic2;
int lidarCount;

Eigen::Matrix4f referenceOdometry;
Eigen::Matrix4d initialTransformGuess = Eigen::Matrix4d::Identity();
std::vector<double> initialTransGuess ={};
std::vector<Eigen::Matrix4f> transVecsOdometry;
std::vector<Eigen::Matrix4f> transVecsLidar1;
std::vector<Eigen::Matrix4f> transVecsLidar2;
std::vector<pcl::PointCloud<pcl::PointXYZI>> pclPointLidar1;
std::vector<pcl::PointCloud<pcl::PointXYZI>> pclPointLidar2;

geometry_msgs::PoseStamped currentGpsPoseStampedMsg;
geometry_msgs::PoseStamped currentPclPoseStampedMsg;
nav_msgs::Path currentGpsNavPathMsg;
nav_msgs::Path currentPclNavPathMsg;

ros::Publisher gpsPoseStampedPublisher;
ros::Publisher pclPoseStampedPublisher;
ros::Publisher gpsNavPathPublisher;
ros::Publisher pclNavPathPublisher;
ros::Publisher targetPcPublisher;
ros::Publisher sourcePcPublisher;


float min_add_scan_shift, min_scan_range,max_scan_range, voxel_leaf_size, fitness_tresh;
float max_corresp_distance, trans_epsilon, euc_fitness_epsilon, max_iter;
float ndt_resolution;
int testNum = 0;

bool isFirstMessage = true;
Eigen::Matrix4f  initialGuess = Eigen::Matrix4f::Identity();
Eigen::Matrix4d  absolutePose = Eigen::Matrix4d::Identity();
pcl::PointCloud<pcl::PointXYZI> previousPointCloud;

static pose added_pose, current_pose, previous_pose, guess_pose, sync_pose;
float diff_x, diff_y, diff_z, diff_yaw;

double fitnessScore = std::numeric_limits<double>::max();
bool isConverged = false;
double sync_t =0.1; //fused data output 

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}


Eigen::Matrix4f ndtTracker(pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>& ndt, pcl::PointCloud<pcl::PointXYZI>::Ptr previousCloudPtr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloudPtr,
                           const Eigen::Matrix4f& initialGuess);
Eigen::Matrix4f icpTracker(pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>& icp, pcl::PointCloud<pcl::PointXYZI>::Ptr previousCloudPtr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloudPtr,
                           const Eigen::Matrix4f& initialGuess);
Eigen::Matrix4f gicpTracker(pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>& gicp, pcl::PointCloud<pcl::PointXYZI>::Ptr previousCloudPtr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloudPtr,
                           const Eigen::Matrix4f& initialGuess);
//pcl::PointCloud<pcl::PointXYZI> downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZI>& original, double voxel_leaf_size);

void pointCloudTrackerSingle(const pcl::PointCloud<pcl::PointXYZI>::Ptr previousCloudPtr, const pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudPtr,
                       Eigen::Matrix4f& transformation, const Eigen::Matrix4f& initialGuess,
                       const std::string& mode)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());

    if(mode == "icp")
    {
        std::cout << "----------------------Use Icp tracker----------------------" << std::endl;
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

        transformation = icpTracker(icp, previousCloudPtr, currentCloudPtr, outputCloudPtr, initialGuess);
    }
    else if(mode == "ndt")
    {
        std::cout << "----------------------Use ndt tracker----------------------" << std::endl;
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

//        Eigen::Matrix4f currentTransform = ndtTracker(ndt, previousCloudPtr, currentCloudPtr, outputCloudPtr, initialGuess);
          transformation = ndtTracker(ndt, previousCloudPtr, currentCloudPtr, outputCloudPtr, initialGuess);

//        if(currentTransform.isZero())
//            currentTransform = Eigen::Matrix4f::Identity();

//        transformation = icpTracker(icp, previousCloudPtr, currentCloudPtr, outputCloudPtr, currentTransform);
    }
    else if(mode == "gicp"){
        std::cout << "----------------------Use GIcp tracker----------------------" << std::endl;
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;

        transformation = gicpTracker(gicp, previousCloudPtr, currentCloudPtr, outputCloudPtr, initialGuess);
    }
    else 
        return;
}

geometry_msgs::PoseStamped constructPoseStampedFromEigenMatrix(const Eigen::Matrix4d &transform)
{
  geometry_msgs::PoseStamped poseStamped;
  tf::Matrix3x3 mat_tf;
  mat_tf.setValue(transform(0, 0), transform(0, 1), transform(0, 2),
                  transform(1, 0), transform(1, 1), transform(1, 2),
                  transform(2, 0), transform(2, 1), transform(2, 2));
  poseStamped.pose.position.x = transform(0, 3);
  poseStamped.pose.position.y = transform(1, 3);
  poseStamped.pose.position.z = transform(2, 3);

  tf::Quaternion q;
  mat_tf.getRotation(q);
  poseStamped.pose.orientation.x = q.x();
  poseStamped.pose.orientation.y = q.y();
  poseStamped.pose.orientation.z = q.z();
  poseStamped.pose.orientation.w = q.w();
  return poseStamped;
}


void poseLidarCallback(const geometry_msgs::PoseStampedConstPtr& poseMsg, const sensor_msgs::PointCloud2ConstPtr &lidarMsg)
{
    std::cout << "Pose lidar callback " << std::endl;
    ros::Time current_pose_time = poseMsg->header.stamp;
    ros::Time current_scan_time = lidarMsg->header.stamp;
    double diff_time = current_pose_time.toSec() - current_scan_time.toSec();

    const geometry_msgs::Quaternion& orien = poseMsg->pose.orientation;
    double orienTemp[4] = {orien.w, orien.x, orien.y, orien.z}; // scalar, i, j ,k

    Eigen::Matrix3d rot = SensorCalibration::QuaternionToRotation(orienTemp);
    // std::cout << "rot : \n" << rot <<std::endl;
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    // trans.setZero();
    trans.block<3,3>(0,0) = rot.cast<float>();
    trans.block<3,1>(0,3) << poseMsg->pose.position.x,poseMsg->pose.position.y, poseMsg->pose.position.z;
    
    ros::Time loop_begin = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZI> pclMessage;
    pcl::PointCloud<pcl::PointXYZI> transformed_pclMessage;
    pcl::fromROSMsg(*lidarMsg, pclMessage);

    ros::Time bp1 = ros::Time::now();
    if(isFirstMessage)
    {
        previousPointCloud = pclMessage;
        //initialGuess = Eigen::Matrix4f::Identity();
        transVecsLidar1.push_back(Eigen::Matrix4f::Identity());
        transVecsOdometry.push_back(Eigen::Matrix4f::Identity());
        referenceOdometry = trans;
        isFirstMessage = false;
        return;
    }

    Eigen::Matrix4f transNewOrigin = referenceOdometry.inverse()*trans;
    transVecsOdometry.push_back(transNewOrigin);
    Eigen::Matrix4d transPose = initialTransformGuess.inverse()* transNewOrigin.cast<double>() * initialTransformGuess;
    ros::Time bp2 = ros::Time::now();

    guess_pose.x = previous_pose.x + diff_x;
    guess_pose.y = previous_pose.y + diff_y;
    guess_pose.z = previous_pose.z + diff_z; //previous.z = 0, diff_z = 0
    guess_pose.roll = previous_pose.roll;
    guess_pose.pitch = previous_pose.pitch;
    guess_pose.yaw = previous_pose.yaw + diff_yaw;

    Eigen::AngleAxisf init_rotation_x(guess_pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pose.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(guess_pose.x, guess_pose.y, guess_pose.z);

    initialGuess = 
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    // ros::Time initial_end = ros::Time::now();
    // std::cout << "initialGuess time: " << initial_end-initial_start << std::endl;
    //initialGuess = trans;
    Eigen::Matrix4f  transformation;

    // current pc -> downsampled pc -> ndt
    // TODO
    pcl::PointCloud<pcl::PointXYZI> confinedCloud;
    pcl::PointXYZI p;
    for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = pclMessage.begin(); item!= pclMessage.end(); item++){
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = (double)item->intensity;
        float r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));

        if (min_scan_range < r && r < max_scan_range)
          // (*pointcloud).push_back(p);
          confinedCloud.push_back(p);
  }
    //downsample
    pcl::PointCloud<pcl::PointXYZI> currentCloudDownSampled = downsamplePointCloud(confinedCloud, voxel_leaf_size);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudDownSampledPtr(new pcl::PointCloud<pcl::PointXYZI>(currentCloudDownSampled));
    
    // //smoothing
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    // // Output has the PointNormal type in order to store the normals calculated by MLS
    // pcl::PointCloud<pcl::PointXYZI> smoothenedCloud;
    // // Init object (second point type is for the normals, even if unused)
    // pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls;
    // mls.setComputeNormals (false);
    // // Set parameters
    // mls.setInputCloud (currentCloudDownSampledPtr);
    // mls.setPolynomialOrder (2);
    // mls.setSearchMethod (tree);
    // mls.setSearchRadius (0.03);
    // // Reconstruct
    // mls.process (smoothenedCloud);
    std::cout << "---------source point cloud size: "<< currentCloudDownSampled.size() << "-------------" << std::endl;
    std::cout << "---------target point cloud size: "<< previousPointCloud.size() << "-------------" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr previousCloudPtr(new pcl::PointCloud<pcl::PointXYZI>(previousPointCloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudPtr(new pcl::PointCloud<pcl::PointXYZI>(currentCloudDownSampled));

    pointCloudTrackerSingle(previousCloudPtr, currentCloudPtr, transformation, initialGuess, mode);
    //assume planar motion, set z value always 0
    transformation(2,3) = 0.0;
    std::cout << "transformation: \n " <<transformation <<std::endl;
    pcl::transformPointCloud(*currentCloudPtr, transformed_pclMessage, transformation);

    tf::Matrix3x3 mat_l;

    mat_l.setValue(static_cast<double>(transformation(0, 0)), static_cast<double>(transformation(0, 1)),
                 static_cast<double>(transformation(0, 2)), static_cast<double>(transformation(1, 0)),
                 static_cast<double>(transformation(1, 1)), static_cast<double>(transformation(1, 2)),
                 static_cast<double>(transformation(2, 0)), static_cast<double>(transformation(2, 1)),
                 static_cast<double>(transformation(2, 2)));
  // Update localizer_pose.
    current_pose.x = transformation(0, 3);
    current_pose.y = transformation(1, 3);
    current_pose.z = transformation(2, 3);
    mat_l.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw, 1);

    // std::cout<< "initialGuess:  \n" << guess_pose.yaw << std::endl;
    // std::cout<< "transformation: \n" << current_pose.yaw << std::endl;

    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);

    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    float shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
    std::cout << "shift: " << shift << "  fitnessScore: " << fitnessScore << " isconverged " <<isConverged <<std::endl;
    if (shift >= min_add_scan_shift && fitnessScore <fitness_tresh && isConverged){
        added_pose.x = current_pose.x;
        added_pose.y = current_pose.y;
        added_pose.z = current_pose.z;
        added_pose.roll = current_pose.roll;
        added_pose.pitch = current_pose.pitch;
        added_pose.yaw = current_pose.yaw;
        if(mode == "ndt"){
            previousPointCloud += transformed_pclMessage;
            absolutePose = transformation.cast<double>();
        }
        else if(mode == "gicp" or mode =="icp"){
            previousPointCloud = currentCloudDownSampled;
            absolutePose = absolutePose * transformation.cast<double>();
        }

    }
    ros::Time loop_end = ros::Time::now();
    std::cout << "loop time: " << ((bp1- loop_begin) + (loop_end - bp2))*1000  << std::endl;

    // ros::Time pub_start =ros::Time::now();
    //visualize the pointcloud 
    sensor_msgs::PointCloud2 targetPclMsg;
    sensor_msgs::PointCloud2 sourcePclMsg;
    pcl::toROSMsg(pclMessage,sourcePclMsg);
    pcl::toROSMsg(previousPointCloud,targetPclMsg);
    targetPclMsg.header.frame_id = "/imu_link";
    sourcePclMsg.header.frame_id = "/imu_link";
    targetPclMsg.header.stamp = ros::Time::now();
    sourcePclMsg.header.stamp = ros::Time::now();
    targetPcPublisher.publish(targetPclMsg);
    sourcePcPublisher.publish(sourcePclMsg);

    //sync_lidar poses 
    sync_pose.x = current_pose.x + diff_time/sync_t * diff_x;
    sync_pose.y = current_pose.y + diff_time/sync_t * diff_y;
    sync_pose.z = current_pose.z + diff_time/sync_t * diff_z;
    sync_pose.roll = current_pose.roll;
    sync_pose.pitch = current_pose.pitch;
    sync_pose.yaw = current_pose.yaw + diff_time/sync_t * diff_yaw;

    Eigen::Translation3f sync_translation(sync_pose.x, sync_pose.y, sync_pose.z);
    Eigen::AngleAxisf sync_rotation_x(sync_pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf sync_rotation_y(sync_pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf sync_rotation_z(sync_pose.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f sync_local_t = (sync_translation * sync_rotation_z * sync_rotation_y * sync_rotation_x).matrix();
    Eigen::Matrix4f sync_global_t = sync_local_t;
    transVecsLidar1.push_back(sync_global_t);

    // if(mode == "ndt"){
    //     absolutePose = transformation.cast<double>();
    // }
    // else{
    //     absolutePose = absolutePose *transformation.cast<double>();
    // }
    //publish
    currentGpsPoseStampedMsg = constructPoseStampedFromEigenMatrix(transPose);
    currentPclPoseStampedMsg = constructPoseStampedFromEigenMatrix(absolutePose);
    currentGpsPoseStampedMsg.header.frame_id = "/imu_link";
    currentPclPoseStampedMsg.header.frame_id = "/imu_link";
    currentGpsPoseStampedMsg.header.stamp = current_pose_time;
    currentPclPoseStampedMsg.header.stamp = current_scan_time;

    currentGpsNavPathMsg.header.frame_id = "/imu_link";
    currentPclNavPathMsg.header.frame_id = "/imu_link";
    currentGpsNavPathMsg.header.stamp = current_pose_time;
    currentPclNavPathMsg.header.stamp = current_scan_time;
    currentGpsNavPathMsg.poses.push_back(currentGpsPoseStampedMsg);
    currentPclNavPathMsg.poses.push_back(currentPclPoseStampedMsg);
    gpsPoseStampedPublisher.publish(currentGpsPoseStampedMsg);
    gpsNavPathPublisher.publish(currentGpsNavPathMsg);
    pclPoseStampedPublisher.publish(currentPclPoseStampedMsg);
    pclNavPathPublisher.publish(currentPclNavPathMsg);

    // ros::Time pub_end = ros::Time::now();
    // std::cout << "publish time: " << pub_end-pub_start << std::endl;
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


void serializePclPoints(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& pclPointLidar, const std::string& filePath)
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

    for(unsigned int i=0; i<pclPointLidar.size(); i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(5) << i;
        std::string filename = filePath + "/" + ss.str() + ".pcd";
        pcl::io::savePCDFileASCII (filename, pclPointLidar[i]);
    }
}

Eigen::Matrix4f ndtTracker(pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>& ndt, pcl::PointCloud<pcl::PointXYZI>::Ptr previousCloudPtr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloudPtr,
                           const Eigen::Matrix4f& initialGuess)
{
    // ros::Time t1_start = ros::Time::now();
    Eigen::Matrix4f transform;
    transform.setZero();

    ndt.setTransformationEpsilon(trans_epsilon);
    ndt.setStepSize(0.1);
//    ndt.setResolution(align_res_);
    ndt.setResolution(ndt_resolution);
    ndt.setMaximumIterations(max_iter);
    ndt.setInputSource(currentCloudPtr);
    ndt.setInputTarget(previousCloudPtr);
    // ndt.setInputSource(previousCloudPtr);
    // ndt.setInputTarget(currentCloudPtr);
    std::cout << "----------------------ndt align----------------------" << std::endl;
    ndt.align(*outputCloudPtr, initialGuess);
    // ros::Time t1_end = ros::Time::now();
    // std::cout << "ndt align time: " << t1_end-t1_start << std::endl;
    std::cout << "----------------------ndt align exit----------------------" << std::endl;
    fitnessScore = ndt.getFitnessScore();
    isConverged = ndt.hasConverged();

    std::cout << "fitness score:" << fitnessScore << std::endl;
    std::cout << "converged: " << std::boolalpha << isConverged<< std::endl;
//    if(fitnessScore < 0.3 && isConverged)
    transform = ndt.getFinalTransformation();

    return transform;
}

Eigen::Matrix4f icpTracker(pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>& icp, pcl::PointCloud<pcl::PointXYZI>::Ptr previousCloudPtr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloudPtr, 
                           const Eigen::Matrix4f& initialGuess)
{
    Eigen::Matrix4f transform;
    transform.setZero();
    // double fitnessScore = std::numeric_limits<double>::max();
    // bool isConverged = false;

    icp.setMaxCorrespondenceDistance(max_corresp_distance);
    icp.setTransformationEpsilon(trans_epsilon);
    icp.setEuclideanFitnessEpsilon(euc_fitness_epsilon);
    icp.setMaximumIterations (max_iter);
//    icp.setInputSource(currentCloudPtr);
//    icp.setInputTarget(previousCloudPtr);
    icp.setInputSource(currentCloudPtr);
    icp.setInputTarget(previousCloudPtr);
    icp.align(*outputCloudPtr, initialGuess);
    fitnessScore= icp.getFitnessScore();
    isConverged = icp.hasConverged();

    std::cout << "fitness score:" << fitnessScore << std::endl;
    std::cout << "converged: " << std::boolalpha << isConverged<< std::endl;
//    if(fitnessScore < 0.3 && isConverged)
    transform = icp.getFinalTransformation();

    return transform;
}

Eigen::Matrix4f gicpTracker(pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>& gicp, pcl::PointCloud<pcl::PointXYZI>::Ptr previousCloudPtr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloudPtr, pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloudPtr, 
                           const Eigen::Matrix4f& initialGuess)
{
    Eigen::Matrix4f transform;
    transform.setZero();
    // double fitnessScore = std::numeric_limits<double>::max();
    // bool isConverged = false;

    gicp.setMaxCorrespondenceDistance(max_corresp_distance);
    gicp.setTransformationEpsilon(trans_epsilon);
    gicp.setEuclideanFitnessEpsilon(euc_fitness_epsilon);
    gicp.setMaximumIterations (max_iter);
//    icp.setInputSource(currentCloudPtr);
//    icp.setInputTarget(previousCloudPtr);
    gicp.setInputSource(currentCloudPtr);
    gicp.setInputTarget(previousCloudPtr);
    gicp.align(*outputCloudPtr, initialGuess);
    fitnessScore= gicp.getFitnessScore();
    isConverged = gicp.hasConverged();

    std::cout << "fitness score:" << fitnessScore << std::endl;
    std::cout << "converged: " << std::boolalpha << isConverged<< std::endl;
//    if(fitnessScore < 0.3 && isConverged)
    transform = gicp.getFinalTransformation();

    return transform;
}

std::vector<Eigen::Matrix4f> computeRelativeTransform(const std::vector<Eigen::Matrix4f>& absoluteTransforms)
{
    std::vector<Eigen::Matrix4f> relativeTransforms;
    Eigen::Matrix4f referenceFrame;
    Eigen::Matrix4f currentTransform;
    for(unsigned int i=0; i< absoluteTransforms.size(); i++)
    {
        if(i==0)
        {
            referenceFrame = absoluteTransforms[i];
            continue;
        }
        currentTransform = referenceFrame.inverse()*absoluteTransforms[i];
        relativeTransforms.push_back(currentTransform);
        referenceFrame = absoluteTransforms[i];
    }

    return relativeTransforms;

}


pcl::PointCloud<pcl::PointXYZI> pclRegistration(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& points, const std::vector<Eigen::Matrix4f>& relativeTransform)
{
    assert(points.size() == (relativeTransform.size() +1) && "point cloud registration failed");

    if(points.size() == 0)
        return pcl::PointCloud<pcl::PointXYZI>();

    int n = points.size();
    pcl::PointCloud<pcl::PointXYZI> registeredPcl(points[n-1]);
    pcl::PointCloud<pcl::PointXYZI> transformedPcl;
    Eigen::Affine3f affineTransform;

    for(int i=n-2; i >= 0; i--)
    {
        affineTransform = relativeTransform[i];
        pcl::transformPointCloud(registeredPcl, transformedPcl, affineTransform);
        registeredPcl+=points[i];
    }

    return registeredPcl;
}


void organizePoseArrayMessages(const std::vector<Eigen::Matrix4f>& gpsPoses, const std::vector<Eigen::Matrix4f>& lidarPoses, geometry_msgs::PoseArray &gpsMsg, geometry_msgs::PoseArray &lidarMsg)
{
    assert(gpsPoses.size() == lidarPoses.size() && "Transformation pair size does not match!");

    Eigen::Matrix4f gpsReferencePose(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f lidarReferencePose(Eigen::Matrix4f::Identity());

    for(unsigned int i=0; i< gpsPoses.size(); i++)
    {
        Eigen::Matrix4f gpsCurrentPose = gpsPoses[i]*gpsReferencePose;
//        Eigen::Matrix4f gpsCurrentPose = gpsPoses[i];
        gpsReferencePose = gpsCurrentPose;

        geometry_msgs::Pose currGpsPoseMsg;
        currGpsPoseMsg.position.x = gpsCurrentPose(0,3);
        currGpsPoseMsg.position.y = gpsCurrentPose(1,3);
        currGpsPoseMsg.position.z = gpsCurrentPose(2,3);

        Eigen::Quaternionf qGps(gpsCurrentPose.block<3,3>(0,0));
        currGpsPoseMsg.orientation.x = qGps.x();
        currGpsPoseMsg.orientation.y = qGps.y();
        currGpsPoseMsg.orientation.z = qGps.z();
        currGpsPoseMsg.orientation.w = qGps.w();

        gpsMsg.poses.push_back(currGpsPoseMsg);

        Eigen::Matrix4f lidarCurrentPose = lidarPoses[i]*lidarReferencePose;
//        Eigen::Matrix4f lidarCurrentPose = lidarPoses[i];
        lidarReferencePose = lidarCurrentPose;

        geometry_msgs::Pose currLidarPoseMsg;
        currLidarPoseMsg.position.x = lidarCurrentPose(0,3);
        currLidarPoseMsg.position.y = lidarCurrentPose(1,3);
        currLidarPoseMsg.position.z = lidarCurrentPose(2,3);

        Eigen::Quaternionf qLidar(lidarCurrentPose.block<3,3>(0,0));
        currLidarPoseMsg.orientation.x = qLidar.x();
        currLidarPoseMsg.orientation.y = qLidar.y();
        currLidarPoseMsg.orientation.z = qLidar.z();
        currLidarPoseMsg.orientation.w = qLidar.w();

        lidarMsg.poses.push_back(currLidarPoseMsg);
    }
}


void pubNavMessage(ros::Publisher& pub, const Eigen::Matrix4f& pose,  Eigen::Matrix4f& referencePose, nav_msgs::Path &message)
{
    Eigen::Matrix4f currentPose = pose*referencePose;
//        Eigen::Matrix4f currentPose = pose;
    referencePose = currentPose;

    geometry_msgs::PoseStamped  poseMsg;
    poseMsg.header.frame_id = "/map";
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.pose.position.x = pose(0,3);
    poseMsg.pose.position.y = pose(1,3);
    poseMsg.pose.position.z = pose(2,3);

    Eigen::Quaternionf quternion(pose.block<3,3>(0,0));
    poseMsg.pose.orientation.x = quternion.x();
    poseMsg.pose.orientation.y = quternion.y();
    poseMsg.pose.orientation.z = quternion.z();
    poseMsg.pose.orientation.w = quternion.w();

    message.poses.push_back(poseMsg);

    message.header.frame_id = "/map";
    pub.publish(message);
}


bool ref_lla_set_;
double ref_lla[3];
Pose imu_pose_new_;
Eigen::Matrix4f imu_mat_new_;


int main(int argc, char** argv)
{
    previous_pose.x = 0.0;
    previous_pose.y = 0.0;
    previous_pose.z = 0.0;
    previous_pose.roll = 0.0;
    previous_pose.pitch = 0.0;
    previous_pose.yaw = 0.0;

    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.z = 0.0;
    current_pose.roll = 0.0;
    current_pose.pitch = 0.0;
    current_pose.yaw = 0.0;

    guess_pose.x = 0.0;
    guess_pose.y = 0.0;
    guess_pose.z = 0.0;
    guess_pose.roll = 0.0;
    guess_pose.pitch = 0.0;
    guess_pose.yaw = 0.0;

    added_pose.x = 0.0;
    added_pose.y = 0.0;
    added_pose.z = 0.0;
    added_pose.roll = 0.0;
    added_pose.pitch = 0.0;
    added_pose.yaw = 0.0;

    diff_x = 0.0;
    diff_y = 0.0;
    diff_z = 0.0;
    diff_yaw = 0.0;


    ros::init(argc, argv, "handeye_exporter_node");
    ros::NodeHandle nh("~");

//    nh.param("loadFromFile", loadFromFile, false);
//    nh.param("transformationPariFile",  transformationPariFile, std::string(""));
    nh.param("dataPath", dataPath, std::string(""));
//    nh.param("timeSpan", timeSpan, 120.0f);
    nh.param("mode", mode, std::string("ndt"));
    nh.param("odometryTopic", odometryTopic, std::string("/gps_odometry"));
    nh.param("poseTopic", poseTopic, std::string("/pose"));
    nh.param("gnssTopic", gnssTopic, std::string("/nav/fix"));
    nh.param("imuTopic", imuTopic, std::string("/imu/enu"));
    nh.param("useOdoemetryTopic", useOdoemetryTopic, false);

    nh.param("lidarTopic1", lidarTopic1, std::string("/rr/front_left/velodyne_points"));
    nh.param("lidarTopic2", lidarTopic2, std::string("/rr/front_right/velodyne_points"));
    nh.param("lidarCount", lidarCount, 1);
    nh.param<float>("min_scan_range", min_scan_range, min_scan_range);
    nh.param<float>("max_scan_range", max_scan_range, max_scan_range);
    nh.param<float>("min_add_scan_shift", min_add_scan_shift, min_add_scan_shift);
    nh.param<float>("voxel_leaf_size", voxel_leaf_size, voxel_leaf_size);
    nh.param<float>("ndt_resolution", ndt_resolution, ndt_resolution);
    nh.param<float>("fitness_tresh", fitness_tresh, fitness_tresh);
    
    nh.param<float>("max_corresp_distance", max_corresp_distance, max_corresp_distance);
    nh.param<float>("trans_epsilon", trans_epsilon, trans_epsilon);
    nh.param<float>("euc_fitness_epsilon", euc_fitness_epsilon, euc_fitness_epsilon);
    nh.param<float>("max_iter", max_iter, max_iter);

    nh.param("initialTransGuess", initialTransGuess, initialTransGuess);

    if(lidarCount<1 )
    {
        std::cout << "Please provide at lease 1 lidar topic" << std::endl;
        return -1;
    }

    std::cout << "----------------------Configuration----------------------" << std::endl;
    std::cout << "dataPath: " << dataPath << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "useOdoemetryTopic: " << std::boolalpha << useOdoemetryTopic << std::endl;
    std::cout << "odometryTopic: " << odometryTopic << std::endl;
    std::cout << "gnssTopic: " << gnssTopic << std::endl;
    std::cout << "imuTopic: " << imuTopic << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "lidarCount: " << lidarCount << std::endl;
    std::cout << "lidarTopic1: " << lidarTopic1 << std::endl;
    std::cout << "lidarTopic2: " << lidarTopic2 << std::endl;
    std::cout << "----------------------Configuration----------------------" << std::endl;

    Eigen::AngleAxisd rollAngle(initialTransGuess[3], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(initialTransGuess[5], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(initialTransGuess[4], Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();

    initialTransformGuess.block<3,3>(0,0) = rotationMatrix;
    initialTransformGuess.block<3,1>(0,3) << initialTransGuess[0],initialTransGuess[1],initialTransGuess[2];


    ros::Subscriber odoSubTest;
    ros::Subscriber pclSubTest;

    message_filters::Subscriber<geometry_msgs::PoseStamped> poseSub(nh, poseTopic, 1000);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidarSub1(nh, lidarTopic1, 1000);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(1000), poseSub, lidarSub1);
    sync.registerCallback(boost::bind(&poseLidarCallback, _1, _2));

    gpsNavPathPublisher = nh.advertise<nav_msgs::Path>("/SensorCalibration/HandEye/gpsNavPath", 10);
    pclNavPathPublisher = nh.advertise<nav_msgs::Path>("/SensorCalibration/HandEye/pclNavPath", 10);
    gpsPoseStampedPublisher = nh.advertise<geometry_msgs::PoseStamped>("/SensorCalibration/HandEye/gpsPoseStamped", 10);
    pclPoseStampedPublisher = nh.advertise<geometry_msgs::PoseStamped>("/SensorCalibration/HandEye/pclPoseStamed", 10);
    targetPcPublisher =nh.advertise<sensor_msgs::PointCloud2>("/SensorCalibration/HandEye/targetPc",10);
    sourcePcPublisher =nh.advertise<sensor_msgs::PointCloud2>("/SensorCalibration/HandEye/sourcePc",10);

    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "----------------------Subscribing to topics----------------------" << std::endl;
    std::cout << "----------------------Press Ctrl + c to exit----------------------" << std::endl;

    ros::Rate r(10);

    ros::spin();

if(ros::isShuttingDown())
{

    std::cout << "----------------------Saving files----------------------" << std::endl;
    // std::vector<Eigen::Matrix4f> releativeTransforms = computeRelativeTransform(transVecsOdometry);
    // std::cout << "Obtained number of transforms: " << releativeTransforms.size() << std::endl;
    serializeTransformationPairs(transVecsOdometry, transVecsLidar1, dataPath);
    std::cout << "----------------------Files saved----------------------" << std::endl;
}

    return 0;
}
