/*
 * Descriptiong: Hand eye calibration demo
 * Author: Ran Tang
 * Date: May 10, 2018
*/

#include "sensor_calibration/HandEyeCalibration.h"
#include "sensor_calibration/DualQuaternion.h"
#include "sensor_calibration/EigenUtils.h"

#include <string>
#include <iostream>
#include <assert.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <numeric>
#include <random>
#include <algorithm>

// #define _USE_DATA_SELECTION_ true

std::string  transformationPariFile;
std::string resultPath;

bool useRefinement;
bool useInitialGuess;
int solverType;

std::vector<Eigen::Matrix4d> handPoses;
std::vector<Eigen::Matrix4d> eyePoses;

Eigen::Matrix4d calibratedTransformation;
Eigen::Matrix4d initialTransformGuess = Eigen::Matrix4d::Identity();
//geometry_msgs::PoseArray transformedPosesMsg;
//geometry_msgs::PoseArray originalPosesMsg;

template<typename Input>
Eigen::Vector3d eigenRotToEigenVector3dAngleAxis(Input eigenQuat)
{
    Eigen::AngleAxisd ax3d(eigenQuat);
    return ax3d.angle()*ax3d.axis();
}

struct EigenHypothesis
{
    Eigen::Matrix4d mHypothesis;
    double mScore;
    bool operator <(const EigenHypothesis& rhs) const
    {
        return mScore < rhs.mScore;// descending order
    }
};

void readTransformPairsFromFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    assert(fs.isOpened());

//    if(!fs.isOpened())
//    {
//        std::cout << "Failed to open file: " << filename << std::endl;
//        return;
//    }

    cv::Mat poseTemp (4, 4, CV_32F);
    cv::FileNode handOdometry = fs["HandOdometry"];
    for( cv::FileNodeIterator it = handOdometry.begin() ; it != handOdometry.end(); it++)
    {
        Eigen::Matrix4d eigenPose;
        *it >> poseTemp;
        cv::cv2eigen(poseTemp, eigenPose);
        Eigen::Matrix4d eigenPoseInv = eigenPose;
//        std::cout << "hand odoemtry eigenPose: " << std::setprecision(10) << eigenPose << std::endl;
        handPoses.push_back(eigenPose);
    }

//    Eigen::AngleAxisd rot_x(-0.5*M_PI, Eigen::Vector3d::UnitX());
//    Eigen::AngleAxisd rot_y(0.5*M_PI, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd rot_z(0.0, Eigen::Vector3d::UnitZ());
//    Eigen::Translation3d tl(0.0, 0.0, 0.0);
//    Eigen::Matrix4d eigenRotation = (tl *  rot_y * rot_z * rot_x).matrix();

//    std::cout << "eigen rotation matrix:" << eigenRotation << std::endl;

    cv::FileNode eyeOdometry = fs["EyeOdometry"];

    for( cv::FileNodeIterator it = eyeOdometry.begin() ; it != eyeOdometry.end(); it++)
    {
        Eigen::Matrix4d eigenPose;
        *it >> poseTemp;

        cv::cv2eigen(poseTemp, eigenPose);
        Eigen::Matrix4d eigenPoseInv = eigenPose;

        eyePoses.push_back(eigenPoseInv);
    }


    fs.release();
    // for test
//    std::cout << "Size of handPoses: " << handPoses.size() <<std::endl;
//    for(int i=0; i<handPoses.size(); i++)
//    {
//        std::cout << "handPoses" << i << ": " <<handPoses[i] <<std::endl;
//    }

//    std::cout << "Size of eyePoses: " << eyePoses.size() <<std::endl;
//    for(int i=0; i<eyePoses.size(); i++)
//    {
//        std::cout << "eyePoses" << i << ": " <<eyePoses[i] <<std::endl;
//    }
}

// Solve Ax = XB and then refine the pose again
// handPoses: A
// eyePoses: B
// X: A_to_B
Eigen::Matrix4d handEyeCalibrationWithRefinement(const std::vector<Eigen::Matrix4d>& handPoses, const std::vector<Eigen::Matrix4d>& eyePoses, const Eigen::Matrix4d& initialGuess, bool useInitialGuess)
{
    Eigen::Matrix4d calibrationMatrix;
    ceres::Solver::Summary summary;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> handRotVecs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> handTransVecs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eyeRotVecs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eyeTransVecs;

    assert(handPoses.size() == eyePoses.size() && "Transformation pair size not equal!");
    for(int i=0; i< handPoses.size(); i++)
    {
        Eigen::Matrix4d handCurrentPose = handPoses[i];
        handRotVecs.push_back(eigenRotToEigenVector3dAngleAxis(handCurrentPose.block<3,3>(0,0)));
        handTransVecs.push_back(handCurrentPose.block<3,1>(0,3));

        Eigen::Matrix4d eyeCurrentPose = eyePoses[i];
        eyeRotVecs.push_back(eigenRotToEigenVector3dAngleAxis(eyeCurrentPose.block<3,3>(0,0)));
        eyeTransVecs.push_back(eyeCurrentPose.block<3,1>(0,3));
    }

    if(useInitialGuess)
        calibrationMatrix = initialGuess;

    SensorCalibration::HandEye::HandEyeCalibration::estimateHandEyeScrew(handRotVecs, handTransVecs, eyeRotVecs, eyeTransVecs, calibrationMatrix, summary,false, useInitialGuess);

    return calibrationMatrix;
}

Eigen::Matrix4d handEyeCalibrationStandalone(const std::vector<Eigen::Matrix4d>& handPoses, const std::vector<Eigen::Matrix4d>& eyePoses)
{
    Eigen::Matrix4d calibrationMatrix = initialTransformGuess;
    ceres::Solver::Summary summary;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> handRotVecs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> handTransVecs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eyeRotVecs;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eyeTransVecs;

    assert(handPoses.size() == eyePoses.size() && "Transformation pair size not equal!");
    for(int i=0; i< handPoses.size(); i++)
    {
        Eigen::Matrix4d handCurrentPose = handPoses[i];
        handRotVecs.push_back(eigenRotToEigenVector3dAngleAxis(handCurrentPose.block<3,3>(0,0)));
        handTransVecs.push_back(handCurrentPose.block<3,1>(0,3));

        Eigen::Matrix4d eyeCurrentPose = eyePoses[i];
        eyeRotVecs.push_back(eigenRotToEigenVector3dAngleAxis(eyeCurrentPose.block<3,3>(0,0)));
        eyeTransVecs.push_back(eyeCurrentPose.block<3,1>(0,3));
    }
    // std::cout << "handrotvecs: " <<handRotVecs.size() << "handtransvecs: " <<handTransVecs.size()
              // << "eyeRotVecs: " <<eyeRotVecs.size() << "eyeTransVecs: " << eyeTransVecs.size() <<std::endl;
    SensorCalibration::HandEye::HandEyeCalibration::estimateHandEyeStandalone(handRotVecs, handTransVecs, eyeRotVecs, eyeTransVecs, calibrationMatrix, summary,true, false);

    return calibrationMatrix;
}

void writeCalibrationToFile(const std::string& resultPath)
{
    std::string filename = resultPath + "/lidarImuCalibration.yaml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    cv::Mat cvTransform;
    cv::eigen2cv(calibratedTransformation, cvTransform);
    fs << "Transformation" << cvTransform;
    std::cout << cvTransform << std::endl;
}

void comparePoses(const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2, double & rotError, double & transError)
{
    Eigen::Matrix4d temp = pose1.inverse() * pose2;

    transError = temp.block<3,1>(0,3).norm();
    Eigen::AngleAxisd aa( temp.block<3,3>(0,0));
    rotError = aa.angle() * 180 / M_PI;
}

void comparePoses2(const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2, double & rotError, double & transError)
{
    const Eigen::Matrix3d &getR = pose1.block<3,3>(0,0);
    const Eigen::Matrix3d &testR = pose2.block<3,3>(0,0);

    const Eigen::Vector3d &getT = pose1.block<3,1>(0,3);
    const Eigen::Vector3d &testT = pose2.block<3,1>(0,3);

//    transError = (getT-testT).norm();
    Eigen::Vector3d diff = getT-testT;
    transError = std::sqrt(diff[0]*diff[0] + diff[1]*diff[1]);

    Eigen::Matrix3d dr = getR*testR.transpose();

    Eigen::AngleAxisd aa(dr);
    rotError = aa.angle() * 180 / M_PI;
}

std::vector<Eigen::Matrix4d> transformPoses(const std::vector<Eigen::Matrix4d>& eyePoses,
                                        const Eigen::Matrix4d& calibratedTransformation)
{

    std::vector<Eigen::Matrix4d> transformedPoses;
    for(int i=0; i< eyePoses.size(); i++)
    {

        Eigen::Matrix4d eyeCurrentPose = eyePoses[i];
        eyeCurrentPose = calibratedTransformation * eyeCurrentPose * calibratedTransformation.inverse();
//        eyeCurrentPose = calibratedTransformation * eyeCurrentPose;
        transformedPoses.push_back(eyeCurrentPose);
    }

    return transformedPoses;
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

nav_msgs::Odometry computeOdometryFromEigenMatrix(const Eigen::Matrix4d &transform)
{
  nav_msgs::Odometry odom;
  odom.header.frame_id = "/imu_link";

  tf::Matrix3x3 mat_tf;
  mat_tf.setValue(transform(0, 0), transform(0, 1), transform(0, 2),
                  transform(1, 0), transform(1, 1), transform(1, 2),
                  transform(2, 0), transform(2, 1), transform(2, 2));
  odom.pose.pose.position.x = transform(0, 3);
  odom.pose.pose.position.y = transform(1, 3);
  odom.pose.pose.position.z = transform(2, 3);

  tf::Quaternion q;
  mat_tf.getRotation(q);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  return odom;
}

void publishNavPath(ros::Publisher &pub, nav_msgs::Path& path_msg, const Eigen::Matrix4d &new_pose)
{
    path_msg.header.frame_id = "imu_link";
    geometry_msgs::PoseStamped poseStamped = constructPoseStampedFromEigenMatrix(new_pose);
    path_msg.poses.push_back(poseStamped);
    pub.publish(path_msg);
}

void publishOdometry(ros::Publisher &pub, const Eigen::Matrix4d &new_pose)
{
    nav_msgs::Odometry odom = computeOdometryFromEigenMatrix(new_pose);
    pub.publish(odom);
}

void performanceEvaluation(const std::vector<Eigen::Matrix4d>& handPoses, const std::vector<Eigen::Matrix4d>& transformedPoses)
{
    double rotErrorSum=0.0;
    double transErrorSum=0.0;
    double rotErrorMax=0.0;
    double transErrorMax=0.0;
    double rotErrorMin=double(INT_MAX);
    double transErrorMin=double(INT_MAX);


    int poseCount = handPoses.size();

    for(int i=0; i< poseCount; i++)
    {
        double rotError, transError;
        comparePoses2(handPoses[i], transformedPoses[i], rotError, transError);
        rotErrorSum+=rotError;
        transErrorSum+=transError;
        rotErrorMax = std::max(rotErrorMax, rotError);
        transErrorMax = std::max(transErrorMax, transError);
        rotErrorMin = std::min(rotErrorMin, rotError);
        transErrorMin = std::min(transErrorMin, transError);
    }

    std::cout << "Minimal rotation error in degree: " << rotErrorMin << std::endl;
    std::cout << "Minimal translation error in meter: " << transErrorMin << std::endl;
    std::cout << "Average rotation error in degree: " << rotErrorSum/double(poseCount) << std::endl;
    std::cout << "Average translation error in meter: " << transErrorSum/double(poseCount) << std::endl;
    std::cout << "Max rotation error in degree: " << rotErrorMax << std::endl;
    std::cout << "Max translation error in meter: " << transErrorMax << std::endl;
}

//void transformedAndOriginalPoseArrayMessages(const std::vector<Eigen::Matrix4d>& handPoses, const std::vector<Eigen::Matrix4d>& eyePoses,
//                                        const Eigen::Matrix4d& calibratedTransformation)
//{
//    assert(handPoses.size() == eyePoses.size() && "Transformation pair size does not match!");

//    std::vector<Eigen::Matrix4d> handAbsolutePoses;
//    std::vector<Eigen::Matrix4d> eyeAbsolutePoses;
//    Eigen::Matrix4d handReferencePose(Eigen::Matrix4d::Identity());
//    Eigen::Matrix4d eyeReferencePose(Eigen::Matrix4d::Identity());

//    handAbsolutePoses.push_back(Eigen::Matrix4d::Identity());
//    eyeAbsolutePoses.push_back(Eigen::Matrix4d::Identity());

//    for(int i=0; i< handPoses.size(); i++)
//    {
//        Eigen::Matrix4d handCurrentPose = handPoses[i]*handReferencePose;
////        Eigen::Matrix4d handCurrentPose = handPoses[i];
//        handReferencePose = handCurrentPose;
//        handAbsolutePoses.push_back(handCurrentPose);

//        geometry_msgs::Pose currHandPoseMsg;
//        currHandPoseMsg.position.x = handCurrentPose(0,3);
//        currHandPoseMsg.position.y = handCurrentPose(1,3);
//        currHandPoseMsg.position.z = handCurrentPose(2,3);

//        Eigen::Quaterniond qHand(handCurrentPose.block<3,3>(0,0));
//        currHandPoseMsg.orientation.x = qHand.x();
//        currHandPoseMsg.orientation.y = qHand.y();
//        currHandPoseMsg.orientation.z = qHand.z();
//        currHandPoseMsg.orientation.w = qHand.w();

//        originalPosesMsg.poses.push_back(currHandPoseMsg);

////        Eigen::Matrix4d eyeCurrentPose = eyePoses[i]*eyeReferencePose;
//        Eigen::Matrix4d eyeCurrentPose = eyePoses[i];
//        eyeCurrentPose = calibratedTransformation.inverse() * eyeCurrentPose * calibratedTransformation;
////        eyeCurrentPose = calibratedTransformation * eyeCurrentPose * calibratedTransformation.inverse();
//        eyeCurrentPose = eyePoses[i]*eyeReferencePose;
//        eyeReferencePose = eyeCurrentPose;
//        eyeAbsolutePoses.push_back(eyeCurrentPose);

//        geometry_msgs::Pose currEyePoseMsg;
//        currEyePoseMsg.position.x = eyeCurrentPose(0,3);
//        currEyePoseMsg.position.y = eyeCurrentPose(1,3);
//        currEyePoseMsg.position.z = eyeCurrentPose(2,3);

//        Eigen::Quaterniond qEye(eyeCurrentPose.block<3,3>(0,0));
//        currEyePoseMsg.orientation.x = qEye.x();
//        currEyePoseMsg.orientation.y = qEye.y();
//        currEyePoseMsg.orientation.z = qEye.z();
//        currEyePoseMsg.orientation.w = qEye.w();

//        transformedPosesMsg.poses.push_back(currEyePoseMsg);
//    }


//}

ros::Publisher transformedPathPub;
ros::Publisher originImuPathPub;


Eigen::Matrix4d computeTransformMatrix(const geometry_msgs::Pose& pose)
{
    Eigen::Matrix4d transform;
    transform.setZero();

    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    transform.block<3,3>(0,0) = q.matrix();
    transform.block<3,1>(0,3) << pose.position.x, pose.position.y, pose.position.z;
    transform(3,3) = 1.0;

    return transform;
}

std::vector<Eigen::Matrix4d> computeAbsolutePoses(const std::vector<Eigen::Matrix4d>& relativePoses)
{
    std::vector<Eigen::Matrix4d> absolutePoses;
    Eigen::Matrix4d referencePose = Eigen::Matrix4d::Identity();

    for(int i=0; i< relativePoses.size(); i++)
    {
        Eigen::Matrix4d currentPose = referencePose * relativePoses[i];
        referencePose = currentPose;
        absolutePoses.push_back(currentPose);
    }

    return absolutePoses;
}


std::vector<Eigen::Matrix4d> computeRelativeTransform(const std::vector<Eigen::Matrix4d>& absoluteTransforms)
{
    std::vector<Eigen::Matrix4d> relativeTransforms;
    Eigen::Matrix4d referenceFrame;
    Eigen::Matrix4d currentTransform;

    for(int i=0; i< absoluteTransforms.size(); i++)
    {
        if(i == 0)
        {
            referenceFrame = absoluteTransforms[i];
            relativeTransforms.push_back(referenceFrame);
            continue;
        }

        currentTransform = referenceFrame.inverse()*absoluteTransforms[i];
        relativeTransforms.push_back(currentTransform);
        referenceFrame = absoluteTransforms[i];
    }

    return relativeTransforms;

}

std::vector<Eigen::Matrix4d> computeTransformFromNewOrigin(const std::vector<Eigen::Matrix4d>& absoluteTransforms)
{
    std::vector<Eigen::Matrix4d> newTransforms;
    Eigen::Matrix4d referenceFrame;
    Eigen::Matrix4d currentTransform;

    for(int i=0; i< absoluteTransforms.size(); i=i+4)
    {
//        if(i % round == 0)
        if(i == 0)
        {
            referenceFrame = absoluteTransforms[i];
            continue;
        }

        currentTransform = referenceFrame.inverse()*absoluteTransforms[i];
        newTransforms.push_back(currentTransform);
//        referenceFrame = absoluteTransforms[i];

    }

    return newTransforms;
}

std::vector<Eigen::Matrix4d> computeAllRelativeTransforms(const std::vector<Eigen::Matrix4d>& absoluteTransforms)
{
    std::vector<Eigen::Matrix4d> relativeTransforms;
    Eigen::Matrix4d currentTransform;

    for(int i=0; i< absoluteTransforms.size(); i=i+4){
        for(int j=i+1; j< absoluteTransforms.size(); j=j+4){
            currentTransform = absoluteTransforms[i].inverse()*absoluteTransforms[j];
            relativeTransforms.push_back(currentTransform);
        }
    }
    return relativeTransforms;
}


std::vector<Eigen::Matrix4d> pruneTransform(const std::vector<Eigen::Matrix4d>& absoluteTransforms, double dataPercentage){
    std::vector<Eigen::Matrix4d> transformPruned;

    int count = absoluteTransforms.size()*dataPercentage;
    transformPruned.assign(absoluteTransforms.begin(), absoluteTransforms.begin()+count);

    return transformPruned;
}

nav_msgs::Path transformedLidarPathMsg, originImuPathMsg;
Eigen::Matrix4d lidarToImu;

void ImuLidarCallback(const nav_msgs::PathConstPtr &lidarPath, const nav_msgs::PathConstPtr &imuPath)
{

    nav_msgs::Path transformedLidarPathMsgTemp, originImuPathMsgTemp;
    {
        std::vector<Eigen::Matrix4d> lidarPoses, imuPoses;

        for(int i=0; i< lidarPath->poses.size(); i++)
        {
            lidarPoses.push_back(computeTransformMatrix(lidarPath->poses[i].pose));
        }

        for(int i=0; i< imuPath->poses.size(); i++)
        {
            imuPoses.push_back(computeTransformMatrix(imuPath->poses[i].pose));
        }


        std::cout << "Lidar original pose sizes: " << lidarPoses.size() << std::endl;
        std::cout << "imu original pose sizes: " << imuPoses.size() << std::endl;

        std::vector<Eigen::Matrix4d> lidarRelativePoses = computeRelativeTransform(lidarPoses);
        std::vector<Eigen::Matrix4d> imuRelativePoses = computeRelativeTransform(imuPoses);
        std::cout << "Lidar relative pose sizes: " << lidarRelativePoses.size() << std::endl;
        std::cout << "imu relative pose sizes: " << imuRelativePoses.size() << std::endl;

        if(lidarPath->poses.size() == 400)
            lidarToImu = handEyeCalibrationWithRefinement(imuPoses, lidarPoses, initialTransformGuess, useInitialGuess);

        std::vector<Eigen::Matrix4d> transformedLidarRelativePoses;
        for(int i=0; i< lidarRelativePoses.size(); i++)
        {
            Eigen::Matrix4d transformed = lidarToImu.inverse() * lidarRelativePoses[i] * lidarToImu;
            transformedLidarRelativePoses.push_back(transformed);
        }

        std::vector<Eigen::Matrix4d> transformedLidarAbsolutePoses = computeAbsolutePoses(transformedLidarRelativePoses);
        std::vector<Eigen::Matrix4d> imuAbsolutePoses = computeAbsolutePoses(imuRelativePoses);
        std::cout << "Lidar absolute pose sizes: " << transformedLidarAbsolutePoses.size() << std::endl;
        std::cout << "imu absolute pose sizes: " << imuAbsolutePoses.size() << std::endl;

        for(int i=0; i< transformedLidarAbsolutePoses.size(); i++)
        {
            transformedLidarAbsolutePoses[i] = lidarPoses[i] * lidarToImu;
        }

        for(int i=0; i< transformedLidarAbsolutePoses.size(); i++)
        {
            geometry_msgs::PoseStamped lidarPoseMsg = constructPoseStampedFromEigenMatrix(transformedLidarAbsolutePoses[i]);
            transformedLidarPathMsgTemp.poses.push_back(lidarPoseMsg);
        }

        for(int i=0; i< imuAbsolutePoses.size(); i++)
        {
            geometry_msgs::PoseStamped imuPoseMsg = constructPoseStampedFromEigenMatrix(imuAbsolutePoses[i]);
            originImuPathMsgTemp.poses.push_back(imuPoseMsg);
        }

    }
    transformedLidarPathMsgTemp.header.frame_id = "map";
    originImuPathMsgTemp.header.frame_id = "map";
    transformedPathPub.publish(transformedLidarPathMsgTemp);
    originImuPathPub.publish(*imuPath);
 }


std::vector<Eigen::Matrix4d> computeMatrixInverse(const std::vector<Eigen::Matrix4d>& matrices)
{
    std::vector<Eigen::Matrix4d> inverse;
    for(int i=0; i< matrices.size(); i++)
    {
        Eigen::Matrix4d inverMat = matrices[i].inverse();
        inverse.push_back(inverMat);
    }

    return inverse;
}

std::vector<Eigen::Matrix4d> transformPoseVectors(const std::vector<Eigen::Matrix4d> &poses, double roll, double pitch, double yaw)
{
    std::vector<Eigen::Matrix4d> retMatrix4d;

    Eigen::AngleAxisd rot_x(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d tl(0, 0, 0);
    Eigen::Matrix4d transform_matrix = (tl * rot_z * rot_y * rot_x).matrix();

    for(int i=0; i<poses.size(); i++)
    {
        Eigen::Matrix4d eigenPose = transform_matrix * poses[i];
        retMatrix4d.push_back(eigenPose);
    }

    return retMatrix4d;
}

void computeMeanVariance(const std::vector<double>& values, double& mean, double& variance)
{
    double sum  = std::accumulate(values.begin(), values.end(), 0.0);
    mean = sum / values.size();

    std::vector<double> diff(values.size());
    std::transform(values.begin(), values.end(), diff.begin(), [mean](double x) {return x-mean;});
    double squaredSum  = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    variance = std::sqrt(squaredSum / values.size());
}

void transformFilter(const std::vector<Eigen::Matrix4d>& handPoses, const std::vector<Eigen::Matrix4d>& eyePoses, const Eigen::Matrix4d& initialGuess,
                     std::vector<Eigen::Matrix4d>& filterHandPoses,std::vector<Eigen::Matrix4d>& filterEyePoses)
{

    std::vector<double> rotErrors, transErrors;
    for(int i=0; i<handPoses.size(); i++)
    {
        Eigen::Matrix4d eyeCurrentPose = initialGuess * eyePoses[i] * initialGuess.inverse();
        double rotError, transError;
        comparePoses(eyeCurrentPose, handPoses[i], rotError, transError);
        rotErrors.push_back(rotError);
        transErrors.push_back(transError);
    }

    double rotMean, transMean, rotStddev, transStddev;
    computeMeanVariance(rotErrors, rotMean, rotStddev);
    computeMeanVariance(transErrors, transMean, transStddev);
    std::cout<<"rotMean: " << rotMean << " rotStddev: " << rotStddev << " transMean: "<< transMean << " transStddev: "<< transStddev << std::endl;

//    double rt = rotMean+rotStddev;
//    double tt = transMean+transStddev;
    double rt = rotMean;
    double tt = transMean;

    for(int i=0; i<handPoses.size(); i++)
    {
        Eigen::Matrix4d eyeCurrentPose = initialGuess * eyePoses[i] * initialGuess.inverse();
        double rotError, transError;
        comparePoses(eyeCurrentPose, handPoses[i], rotError, transError);
        if(rotError<rt && transError<tt)
        {
            filterHandPoses.push_back(handPoses[i]);
            filterEyePoses.push_back(eyePoses[i]);
        }
    }

    std::cout<<"filtered size: " << filterEyePoses.size() << std::endl;
}

void printResults(const Eigen::Matrix4d &calibratedTransformation)
{
    Eigen::Quaterniond qc(calibratedTransformation.block<3,3>(0,0));
    tf::Quaternion qtfq(qc.x(), qc.y(), qc.z(), qc.w());
    tf::Matrix3x3 qmt33(qtfq);
    double troll, tpitch, tyaw;
    qmt33.getRPY(troll, tpitch, tyaw);
    std::cout << "Calibrated transformation: "<< calibratedTransformation << std::endl;
    std::cout << "Calibrated transformation in rpy: "<< std::endl;
    std::cout << "roll: " << troll << " pitch: " << tpitch << " yaw: " << tyaw << std::endl;

    std::cout << "Calibrated transformation inverse: "<< calibratedTransformation.inverse() << std::endl;

    Eigen::Matrix4d transformInverse = calibratedTransformation.inverse();
    Eigen::Quaterniond qc1(transformInverse.block<3,3>(0,0));
    tf::Quaternion qtfq1(qc1.x(), qc1.y(), qc1.z(), qc1.w());
    tf::Matrix3x3 qmt331(qtfq1);
    qmt331.getRPY(troll, tpitch, tyaw);
    std::cout << "Calibrated transformation inverse in rpy: "<< std::endl;
    std::cout << "roll: " << troll << " pitch: " << tpitch << " yaw: " << tyaw << std::endl;
}

std::pair<double, std::vector<int>> modelEvaluation(const Eigen::Matrix4d& calibrationMatrix, const std::vector<Eigen::Matrix4d> &handPoses,
                                                    const std::vector<Eigen::Matrix4d> &eyePoses, double theta, double t)
{
    std::pair<double, std::vector<int>> evaluationPair;
    std::vector<double> rotErrors, transErrors;

    for(int i=0; i<handPoses.size(); i++)
    {
        Eigen::Matrix4d eyeCurrentPose = calibrationMatrix * eyePoses[i] * calibrationMatrix.inverse();
        double rotError, transError;
        comparePoses(eyeCurrentPose, handPoses[i], rotError, transError);
        rotErrors.push_back(rotError);
        transErrors.push_back(transError);
    }

    double rotMean, transMean, rotStddev, transStddev;
    computeMeanVariance(rotErrors, rotMean, rotStddev);
    computeMeanVariance(transErrors, transMean, transStddev);
//    std::cout<<"rotMean: " << rotMean << " rotStddev: " << rotStddev << " transMean: "<< transMean << " transStddev: "<< transStddev << std::endl;
    std::vector<int> indices;

    int count = 0;
    for(int i=0; i<rotErrors.size(); i++)
    {
        if(rotErrors[i]<theta && transErrors[i]<t)
        {
            indices.push_back(i);
            count++;
        }
    }

    evaluationPair.first = double(count) / double(rotErrors.size());
    evaluationPair.second = indices;

    return evaluationPair;
}

std::vector<Eigen::AngleAxisd> computeAngleAxis(const std::vector<Eigen::Matrix4d> &transforms)
{
    std::vector<Eigen::AngleAxisd> angleAxisRepresentations;
    for(int i=0; i<transforms.size(); i++)
    {
        Eigen::AngleAxisd angleAxis(transforms[i].block<3,3>(0,0));
        angleAxisRepresentations.push_back(angleAxis);
    }

    return angleAxisRepresentations;
}

void prefiltering(std::vector<Eigen::Matrix4d> &handPoses, std::vector<Eigen::Matrix4d> &eyePoses, double theta_t)
{
    // angleaxis representation
    std::vector<Eigen::AngleAxisd> eyeAngleaxis = computeAngleAxis(eyePoses);
    std::vector<Eigen::AngleAxisd> handAngleaxis = computeAngleAxis(handPoses);

    int dataSize = handPoses.size();
    // pre-filtering
    std::vector<double> thetas;
    for(int i=0; i<handAngleaxis.size(); i++)
    {
        double theta = handAngleaxis[i].angle();
        thetas.push_back(theta);

        if( theta < theta_t || (theta > M_PI-theta_t && theta < M_PI+theta_t) || theta > 2*M_PI-theta_t)
        {
            handPoses.erase(handPoses.begin()+i);
            eyePoses.erase(eyePoses.begin()+i);
            handAngleaxis.erase(handAngleaxis.begin()+i);
            eyeAngleaxis.erase(eyeAngleaxis.begin()+i);
            i--;
        }
    }

    double mean, variance;
    computeMeanVariance(thetas, mean, variance);

    std::cout << "thetas: mean "<<mean << " variance: " << variance << std::endl;

    std::cout << "Pose count before prefiltering:" << dataSize << std::endl;
    std::cout << "handPose count after prefiltering:" << handPoses.size() << std::endl;
    std::cout << "eyePose count after prefiltering:" << eyePoses.size() << std::endl;
}

std::vector<EigenHypothesis> generateHypotheses(const std::vector<Eigen::Matrix4d> &handPoses, const std::vector<Eigen::Matrix4d> &eyePoses, int N, int sampleCount,
                                                int maxIter, double theta, double t, double tau)
{
    std::vector<EigenHypothesis> hypotheses;

#ifdef _USE_DATA_SELECTION_
    // angleaxis representation
    std::vector<std::pair<int, int>> selectedPairIndices;
    std::vector<Eigen::AngleAxisd> eyeAngleaxis = computeAngleAxis(eyePoses);
    std::vector<Eigen::AngleAxisd> handAngleaxis = computeAngleAxis(handPoses);

    for(int k=0; k<handAngleaxis.size(); k++)
    {
        std::cout << "Axis normalized : " << handAngleaxis[k].axis() << std::endl;
        std::cout << "Angle: " << handAngleaxis[k].angle() << std::endl;
    }

    // data selection
    std::vector<double> s_kls;
    for(int k=0; k<eyeAngleaxis.size(); k++)
    {
        for(int l=k+1; l<eyeAngleaxis.size(); l++)
        {
            const Eigen::AngleAxisd &currHandk = handAngleaxis[k];
            const Eigen::AngleAxisd &currHandl = handAngleaxis[l];
            double s_kl = std::abs(currHandk.axis().transpose() * currHandl.axis());
            s_kls.push_back(s_kl);

            if(s_kl < 0.3)
                selectedPairIndices.push_back(std::pair<int, int>(k,l));
        }
    }

    double mean, variance;
    computeMeanVariance(s_kls, mean, variance);
    std::cout << "s_kls: mean "<<mean << " variance: " << variance << std::endl;
    std::cout << "selectedPairIndices size: "<< selectedPairIndices.size() << std::endl;


    // generate hypotheses
    std::vector<Eigen::Matrix4d> handPosesSelected;
    std::vector<Eigen::Matrix4d> eyePosesSelected;

    // Randomness
    std::random_device seedDevice;
    std::mt19937 randEngine(seedDevice());
    std::uniform_int_distribution<> dis(0, selectedPairIndices.size()-1);

    // generate a number of hypotheses
    for(int i=0; i<N; i++)
    {
        std::cout << "---------------------Hypothesis: " << i << "--------------------------" << std::endl;
        handPosesSelected.clear();
        eyePosesSelected.clear();

        handPosesSelected.push_back(handPoses[selectedPairIndices[i].first]);
        eyePosesSelected.push_back(eyePoses[selectedPairIndices[i].first]);

        handPosesSelected.push_back(handPoses[selectedPairIndices[i].second]);
        eyePosesSelected.push_back(eyePoses[selectedPairIndices[i].second]);

        EigenHypothesis hand2EyeTransform;
        hand2EyeTransform.mHypothesis = handEyeCalibrationStandalone(handPosesSelected, eyePosesSelected);

        std::pair<double, std::vector<int>> evaluationResults = modelEvaluation(hand2EyeTransform.mHypothesis, handPoses, eyePoses, theta, t);

        std::cout << "---------------------Fraction of inliers: " << evaluationResults.first << "--------------------------" << std::endl;
        if(evaluationResults.first >= tau)
        {
            std::cout << "---------------------Found a good model--------------------------" << std::endl;
            hypotheses.push_back(hand2EyeTransform);
        }
    }
#else
    assert(handPoses.size() == eyePoses.size() && "Pose size does not match");
    std::vector<int> indexShuffle(handPoses.size());
    std::iota(indexShuffle.begin(), indexShuffle.end(), 0);

    std::vector<Eigen::Matrix4d> handPosesShuffle;
    std::vector<Eigen::Matrix4d> eyePosesShuffle;

    // Randomness
    std::random_device seedDevice;
    std::mt19937 randEngine(seedDevice());

    for(int i=0; i<N; i++)
    {
       // std::cout << "---------------------Hypothesis: " << i << "--------------------------" << std::endl;
        for(int j=0; j< maxIter; j++)
        {
            EigenHypothesis hand2EyeTransform;

           // std::cout << "---------------------Iteration: " << j << "--------------------------" << std::endl;
            std::shuffle(indexShuffle.begin(), indexShuffle.end(), randEngine);
            handPosesShuffle.clear();
            eyePosesShuffle.clear();
            for(int k=0; k<sampleCount; k++)
            {
                handPosesShuffle.push_back(handPoses[indexShuffle[k]]);
                eyePosesShuffle.push_back(eyePoses[indexShuffle[k]]);
            }
           // indexShuffle.erase(indexShuffle.begin(), indexShuffle.begin()+sampleCount);
            // std:: cout <<"index shuffle: " << indexShuffle[0] << indexShuffle[1] << indexShuffle[2] <<std::endl;
            hand2EyeTransform.mHypothesis = handEyeCalibrationStandalone(handPosesShuffle, eyePosesShuffle);
            // std::cout << "hand2EyeTransform.mHypothesis \n" << hand2EyeTransform.mHypothesis <<std::endl; 
            std::pair<double, std::vector<int>> evaluationResults = modelEvaluation(hand2EyeTransform.mHypothesis, handPoses, eyePoses, theta, t);

           // std::cout << "---------------------Fraction of inliers: " << evaluationResults.first << "--------------------------" << std::endl;
            if(evaluationResults.first >= tau)
            {
               // std::cout << "---------------------Found a good model--------------------------" << std::endl;
                hypotheses.push_back(hand2EyeTransform);
                break;
            }
        }
    }
#endif

    return hypotheses;
}

double computeScore(const Eigen::Matrix4d& hypothesis, const Eigen::Matrix4d& handPose, const Eigen::Matrix4d& eyePose)
{
    Eigen::Vector3d hypothRot = eigenRotToEigenVector3dAngleAxis(hypothesis.block<3,3>(0,0));
    Eigen::Vector3d hypothTrans = hypothesis.block<3,1>(0,3);

    Eigen::Vector3d handRot = eigenRotToEigenVector3dAngleAxis(handPose.block<3,3>(0,0));
    Eigen::Vector3d handTrans = handPose.block<3,1>(0,3);

    Eigen::Vector3d eyeRot = eigenRotToEigenVector3dAngleAxis(eyePose.block<3,3>(0,0));
    Eigen::Vector3d eyeTrans = eyePose.block<3,1>(0,3);

    SensorCalibration::DualQuaternion<double> dq(SensorCalibration::AngleAxisToQuaternion<double>(hypothRot), hypothTrans);

    SensorCalibration::DualQuaternion<double> dq1(SensorCalibration::AngleAxisToQuaternion<double>(handRot), handTrans);
    SensorCalibration::DualQuaternion<double> dq2(SensorCalibration::AngleAxisToQuaternion<double>(eyeRot), eyeTrans);
    SensorCalibration::DualQuaternion<double> dq1_ = dq * dq2 * dq.inverse();
    SensorCalibration::DualQuaternion<double> diff = (dq1.inverse() * dq1_).log();
    double score = diff.real().squaredNorm() + diff.dual().squaredNorm();

    return score;
}

void hypothesesPruning(std::vector<EigenHypothesis> &hypotheses, const std::vector<Eigen::Matrix4d> &handPoses,
                     const std::vector<Eigen::Matrix4d> &eyePoses, std::mt19937& randEngine, int sampleCount, int leftCount)
{
    std::vector<int> indexShuffle(handPoses.size());
    std::iota(indexShuffle.begin(), indexShuffle.end(), 0);

    std::shuffle(indexShuffle.begin(), indexShuffle.end(), randEngine);

    for(int i=0; i<hypotheses.size(); i++)
    {
        EigenHypothesis &currHypothesis = hypotheses[i];
        double score=0.0;

        for(int j=0; j<sampleCount; j++)
        {
            int index = indexShuffle[j];
            double currScore = computeScore(currHypothesis.mHypothesis, handPoses[index], eyePoses[index]);
            score+=currScore;
        }

        currHypothesis.mScore = score;
    }
    std::sort(hypotheses.begin(), hypotheses.end());
//    std::cout << "scores in descending order: " << std::endl;
   // for(int i=0; i<hypotheses.size(); i++)
   // {
   //     std::cout << hypotheses[i].mScore << std::endl;
   // }

    if(hypotheses.size() > leftCount)
        hypotheses.resize(leftCount);
}

bool checkInlier(const Eigen::Matrix4d hypothesis, const Eigen::Matrix4d& handPose, const Eigen::Matrix4d& eyePose, double rt, double tt)
{

    Eigen::Matrix4d eyeCurrentPose = hypothesis * eyePose * hypothesis.inverse();
    double rotError, transError;
    comparePoses(eyeCurrentPose, handPose, rotError, transError);
    if(rotError<rt && transError<tt)
    {
        return true;
    }

    return false;
}

void handEyeTransformsRefinement(std::vector<EigenHypothesis>& hypothesis, const std::vector<Eigen::Matrix4d>& handPoses, const std::vector<Eigen::Matrix4d>& eyePoses,
                                 std::mt19937& randEngine, int sampleCount, double rt, double tt)
{
    assert(handPoses.size() == eyePoses.size() && "Transformation pair size not equal!");

    ceres::Solver::Summary summary;

    std::vector<int> indexShuffle(handPoses.size());
    std::iota(indexShuffle.begin(), indexShuffle.end(), 0);

    std::shuffle(indexShuffle.begin(), indexShuffle.end(), randEngine);

    for(int i=0; i< hypothesis.size(); i++)
    {
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> handRotVecs;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> handTransVecs;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eyeRotVecs;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eyeTransVecs;

        for(int j=0; j< sampleCount; j++)
        {
            int index = indexShuffle[j];
            if(!checkInlier(hypothesis[i].mHypothesis, handPoses[index], eyePoses[index], rt, tt))
                continue;

            handRotVecs.push_back(eigenRotToEigenVector3dAngleAxis(handPoses[index].block<3,3>(0,0)));
            handTransVecs.push_back(handPoses[index].block<3,1>(0,3));

            eyeRotVecs.push_back(eigenRotToEigenVector3dAngleAxis(eyePoses[index].block<3,3>(0,0)));
            eyeTransVecs.push_back(eyePoses[index].block<3,1>(0,3));
        }

       // std::cout << "------------Inlier count: " << handRotVecs.size() << "------------" << std::endl;
        SensorCalibration::HandEye::HandEyeCalibration::handEyeTransformRefinement(handRotVecs, handTransVecs, eyeRotVecs, eyeTransVecs, hypothesis[i].mHypothesis, summary);
    }
}

Eigen::Matrix4d calibrationRansac(std::vector<EigenHypothesis>& hypotheses, const std::vector<Eigen::Matrix4d> &handPoses,
                                  const std::vector<Eigen::Matrix4d> &eyePoses, int sampleCount, int leftCount, double rt, double tt)
{
    assert(handPoses.size() == eyePoses.size() && "Pose size does not match");

    // Randomness
    std::random_device seedDevice;
    std::mt19937 randEngine(seedDevice());

    std::cout << "hypotheses pre-pruning" <<std::endl;

    hypothesesPruning(hypotheses,handPoses, eyePoses, randEngine, sampleCount, leftCount);
//    std::cout << "N: " << N << " theta: " << theta << " t: " << t << " tau: " << tau << " sampleCount: " << sampleCount << std::endl;
   std::cout << "---------------------------RANSAC starts---------------------------" << std::endl;
    while(hypotheses.size() > 1)
    {
       std::cout << "--------------------------In Ransac, hypotheses size: " << hypotheses.size() << "----------------------- "<< std::endl;
        // refinement
        handEyeTransformsRefinement(hypotheses, handPoses, eyePoses, randEngine, sampleCount, rt, tt);
        std::cout << "--------------------------In Ransac, after refine size: " << hypotheses.size() << "--------\n" ;
        hypothesesPruning(hypotheses,handPoses, eyePoses, randEngine, sampleCount, hypotheses.size()/2);
    }

   std::cout << "---------------------------RANSAC ends---------------------------" << std::endl;
    return hypotheses[0].mHypothesis;
}

std::vector<Eigen::Matrix4d> computePoseInverse(const std::vector<Eigen::Matrix4d> &poses)
{
    std::vector<Eigen::Matrix4d> poseInverse;
    for(int i=0; i<poses.size(); i++)
    {
        Eigen::Matrix4d temp = poses[i].inverse();
        poseInverse.push_back(temp);
    }

    return poseInverse;
}

void handEyePosesComparision(const std::vector<Eigen::Matrix4d>& handPoses, const std::vector<Eigen::Matrix4d>& eyePoses)
{
    for(int i=0; i<handPoses.size(); i++)
    {
        Eigen::AngleAxisd handAngle(handPoses[i].block<3,3>(0,0));
        Eigen::AngleAxisd eyeAngle(eyePoses[i].block<3,3>(0,0));
        std::cout << "------------------Pose index: " << i << "------------------" << std::endl;
        std::cout << "Gps pose: angle-"  << handAngle.angle() << std::endl;
        std::cout << "aixs-" << handAngle.axis() << std::endl;

        std::cout << "Lidar pose: angle-"  << eyeAngle.angle() << std::endl;
        std::cout << "aixs-" << eyeAngle.axis() << std::endl;
    }
}

void removeOutliers(std::vector<Eigen::Matrix4d>& handPoses, std::vector<Eigen::Matrix4d>& eyePoses)
{
    std::vector<Eigen::Matrix4d> handRelativePoses;
    std::vector<Eigen::Matrix4d> eyeRelativePoses;

    handRelativePoses = computeRelativeTransform(handPoses);
    eyeRelativePoses = computeRelativeTransform(eyePoses);

    std::vector<double> rotErrors, transErrors;
    for(int i=0; i<eyeRelativePoses.size(); i++)
    {
        double rotError, transError;
        const Eigen::Matrix4d& temp = eyeRelativePoses[i];
        transError = temp.block<3,1>(0,3).norm();
        Eigen::AngleAxisd aa( temp.block<3,3>(0,0));
        rotError = aa.angle() * 180 / M_PI;

        rotErrors.push_back(rotError);
        transErrors.push_back(transError);
    }

    double rotMean, transMean, rotStddev, transStddev;
    computeMeanVariance(rotErrors, rotMean, rotStddev);
    computeMeanVariance(transErrors, transMean, transStddev);
    std::cout<<"rotMean: " << rotMean << " rotStddev: " << rotStddev << " transMean: "<< transMean << " transStddev: "<< transStddev << std::endl;

    double rt = rotMean+2*rotStddev;
    double tt = transMean+2*transStddev;

    std::vector<Eigen::Matrix4d> filterHandPoses;
    std::vector<Eigen::Matrix4d> filterEyePoses;

    for(int i=0; i<eyeRelativePoses.size(); i++)
    {
        double rotError, transError;
        const Eigen::Matrix4d& temp = eyeRelativePoses[i];
        transError = temp.block<3,1>(0,3).norm();
        Eigen::AngleAxisd aa( temp.block<3,3>(0,0));
        rotError = aa.angle() * 180 / M_PI;

        if(rotError<rt && transError<tt)
        {
            filterHandPoses.push_back(handPoses[i]);
            filterEyePoses.push_back(eyePoses[i]);
        }
    }

    handPoses = filterHandPoses;
    eyePoses = filterEyePoses;
}


Eigen::Matrix4d computeTransformMatrixFromRT(double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd rot_x(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_y(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rot_z(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d tl(x, y, z);
  Eigen::Matrix4d transform_matrix = (tl * rot_z * rot_y * rot_x).matrix();
  return transform_matrix;
}


Eigen::Matrix4d imuLidarCalibration(const std::vector<Eigen::Matrix4d> &imuPoses,const std::vector<Eigen::Matrix4d> &lidarPoses, int roundCount, int hypothesesCount,
                                    int hypothesesSampleCount, int hypothesesMaxIters, double rotationThreshold, double translationThreshold, double hypothesesPercentage,
                                    int ransacSampleCount, int ransacPrunedCount)
{
    Eigen::Matrix4d averageTransform;
    std::vector<Eigen::Matrix4d> transfroms;

    for(int i=0; i<roundCount; i++)
    {
        std::cout << "----------------------Round: " << i << "----------------------" << std::endl;
        std::cout << "----------------------Generate hypotheses----------------------" << std::endl;

        std::vector<EigenHypothesis> hypotheses = generateHypotheses(imuPoses, lidarPoses, hypothesesCount,
                                                                     hypothesesSampleCount, hypothesesMaxIters, rotationThreshold,
                                                                     translationThreshold, hypothesesPercentage);
        std::cout << "-------------------Hypotheses count: " << hypotheses.size() << "--------------------" << std::endl;

        std::cout << "----------------------RANSAC starts----------------------" << std::endl;

        Eigen::Matrix4d currentTransform = calibrationRansac(hypotheses, imuPoses, lidarPoses, ransacSampleCount, ransacPrunedCount,
                                                             rotationThreshold, translationThreshold);
        transfroms.push_back(currentTransform);
        std::cout << "Current transform: " << currentTransform << std::endl;
        std::cout << "----------------------RANSAC ends----------------------" << std::endl;
    }

    Eigen::Vector3d averageTranslation(0.0,0.0,0.0);
    Eigen::Vector3d averageRotation(0.0,0.0,0.0);

    for(int i=0; i<roundCount; i++)
    {
        const Eigen::Matrix4d &currentTransform = transfroms[i];
        averageTranslation += currentTransform.block(0,3,3,1);

        tf::Matrix3x3 mat_tf;
        mat_tf.setValue(currentTransform(0, 0), currentTransform(0, 1), currentTransform(0, 2),
                        currentTransform(1, 0), currentTransform(1, 1), currentTransform(1, 2),
                        currentTransform(2, 0), currentTransform(2, 1), currentTransform(2, 2) );
        Eigen::Vector3d currRotation;
        mat_tf.getRPY(currRotation(0), currRotation(1), currRotation(2), 1);
        averageRotation += currRotation;


//        std::cout << "curTransform " << transfroms[i] << std::endl;
//        std::cout << "currRotation " << currRotation << std::endl;
//        std::cout << "currTranslation " << currentTransform.block(3,0,3,1) << std::endl;
//        std::cout << "averageRotation: " << averageRotation << std::endl;
//        std::cout << "averageTranslation: " << averageTranslation << std::endl;
    }

    averageTranslation /= double(roundCount);
    averageRotation /= double(roundCount);

   // std::cout << "averageRotation: " << averageRotation << std::endl;
   // std::cout << "averageTranslation: " << averageTranslation << std::endl;

    averageTransform = computeTransformMatrixFromRT(averageTranslation(0), averageTranslation(1), averageTranslation(2),
                                                    averageRotation(0), averageRotation(1), averageRotation(2));

//    std::cout << "averageTransform: " << averageTransform << std::endl;
    return averageTransform;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "handeye_example_node");
    ros::NodeHandle nh("~");

    ros::Publisher transformedLidarPathPublisher = nh.advertise<nav_msgs::Path>("/SensorCalibration/HandEye/TransformedLidarPath", 20);
    ros::Publisher gpsPathPublisher = nh.advertise<nav_msgs::Path>("/SensorCalibration/HandEye/GpsPath", 20);
    ros::Publisher lidarPathPublisher = nh.advertise<nav_msgs::Path>("/SensorCalibration/HandEye/LidarPath", 20);
    ros::Publisher lidarInversePathPublisher = nh.advertise<nav_msgs::Path>("/SensorCalibration/HandEye/LidarInversePath", 20);
    nav_msgs::Path transformedLidarNavPath, gpsNavPath, lidarNavPath, lidarInverseNavPath;
    ros::Rate r(10);

    transformedLidarNavPath.header.frame_id = "imu_link";
    gpsNavPath.header.frame_id = "imu_link";
    lidarNavPath.header.frame_id = "imu_link";

    int roundCount, hypothesesCount, hypothesesSampleCount, hypothesesMaxIters;
    double hypothesesPercentage, dataPercentage;

    int ransacSampleCount, ransacPrunedCount;

    double rotationThreshold, translationThreshold;

    std::vector<double>  initialTransGuess={};


    nh.param("transformationPariFile", transformationPariFile, std::string("/home/ran/Documents/Projects/catkin_ws/src/sensor_calibration/data/odometry/transformationParis.yaml"));
    nh.param("resultPath", resultPath, std::string("/home/ran/Documents/Projects/catkin_ws/src/sensor_calibration/result"));

    nh.param("dataPercentage", dataPercentage, 0.2);
//    nh.param("useRefinement", useRefinement, false);
//    nh.param("solverType", solverType,  1);
//    nh.param("useInitialGuess", useInitialGuess, false);

    nh.param("roundCount", roundCount, 10);

    nh.param("hypothesesCount", hypothesesCount,  512);
    nh.param("hypothesesSampleCount", hypothesesSampleCount, 2);
    nh.param("hypothesesMaxIters", hypothesesMaxIters,  50);
    nh.param("hypothesesPercentage", hypothesesPercentage, 0.6);

    nh.param("ransacSampleCount", ransacSampleCount, 500);
    nh.param("ransacPrunedCount", ransacPrunedCount,  128);

    nh.param("rotationThreshold", rotationThreshold, 5.0);
    nh.param("translationThreshold", translationThreshold,  0.7);
    nh.param("initialGuess", initialTransGuess, initialTransGuess);

    std::cout << "----------------------Configuration----------------------" << std::endl;
    std::cout << "transformationPariFile: " << transformationPariFile << std::endl;
    std::cout << "resultPath: " << resultPath << std::endl;
    std::cout << "round: " << roundCount << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
//    std::cout << "useRefinement: " << std::boolalpha << useRefinement <<std::endl;
//    std::cout << "useInitialGuess: " << std::boolalpha << useInitialGuess <<std::endl;
//    std::cout << "solverType: " << SensorCalibration::HandEye::solverTypeString[solverType]<<std::endl;
    std::cout << "hypothesesCount: " << hypothesesCount <<std::endl;
    std::cout << "hypothesesSampleCount: " << hypothesesSampleCount <<std::endl;
    std::cout << "hypothesesMaxIters: " << hypothesesMaxIters <<std::endl;
    std::cout << "hypothesesPercentage: " << hypothesesPercentage <<std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "ransacSampleCount: " << ransacSampleCount <<std::endl;
    std::cout << "ransacPrunedCount: " << ransacPrunedCount <<std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "rotationThreshold: " << rotationThreshold <<std::endl;
    std::cout << "translationThreshold: " << translationThreshold <<std::endl;
    std::cout << "----------------------Configuration----------------------" << std::endl;

    

    Eigen::AngleAxisd rollAngle(initialTransGuess[3], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(initialTransGuess[5], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(initialTransGuess[4], Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();

    initialTransformGuess.block<3,3>(0,0) = rotationMatrix;
    initialTransformGuess.block<3,1>(0,3) << initialTransGuess[0],initialTransGuess[1],initialTransGuess[2];

    std::cout << "----------------------Reading transformation pairs from the file----------------------" << std::endl;
    readTransformPairsFromFile(transformationPariFile);
    std::cout << "----------------------Data preprocessing----------------------" << std::endl;


    std::cout << "Before pruned data, hand size: " <<  handPoses.size() << "  eye size: "<< eyePoses.size() << std::endl;
    std::vector<Eigen::Matrix4d> eyePosesPruned = pruneTransform(eyePoses,dataPercentage);
    std::vector<Eigen::Matrix4d> handPosesPruned = pruneTransform(handPoses,dataPercentage);
    std::cout << "After pruned data, hand size: " <<  handPosesPruned.size() << "  eye size: "<< eyePosesPruned.size() << std::endl;

    //remove outliers
    removeOutliers(handPosesPruned,eyePosesPruned);
    std::cout << "after remove outliers, eye size: " << handPosesPruned.size() <<" hand size: " << eyePosesPruned.size() << std::endl;

    std::vector<Eigen::Matrix4d> eyePosesNewOrigin = computeTransformFromNewOrigin(eyePosesPruned);
    std::vector<Eigen::Matrix4d> handPosesNewOrigin = computeTransformFromNewOrigin(handPosesPruned);

    std::cout << "new origin eye size: " << handPosesNewOrigin.size() <<" new origin hand size: " << eyePosesNewOrigin.size() << std::endl;
    // compute all relative transformations
    std::vector<Eigen::Matrix4d> eyeRelativePoses = computeRelativeTransform(eyePosesNewOrigin);
    std::vector<Eigen::Matrix4d> handRelativePoses = computeRelativeTransform(handPosesNewOrigin);
    std::cout << "All relative pose count:" << eyeRelativePoses.size() << std::endl;

    // data prefiltering
    // prefiltering(handRelativePoses, eyeRelativePoses, 15.0 * M_PI /180.0);
    // data selection

    std::cout << "----------------------Lidar-Imu calibration starts----------------------" << std::endl;
    calibratedTransformation = imuLidarCalibration(handPosesNewOrigin, eyePosesNewOrigin, roundCount, hypothesesCount, hypothesesSampleCount, hypothesesMaxIters,
                        rotationThreshold, translationThreshold, hypothesesPercentage,
                        ransacSampleCount, ransacPrunedCount);
    std::cout << "----------------------Lidar-Imu calibration ends----------------------" << std::endl;
    printResults(calibratedTransformation);

    // double troll, tpitch, tyaw;
    // Eigen::Quaterniond qgt(initialTransformGuess.block<3,3>(0,0));
    // tf::Quaternion qtfqt(qgt.x(), qgt.y(), qgt.z(), qgt.w());
    // tf::Matrix3x3 qmt33gt(qtfqt);
    // qmt33gt.getRPY(troll, tpitch, tyaw);

    std::cout << "Ground truth: "<< initialTransformGuess << std::endl;
    // std::cout << "Ground truth in rpy: "<< std::endl;
    // std::cout << "roll: " << troll << " pitch: " << tpitch << " yaw: " << tyaw << std::endl;

    // std::cout << "----------------------Write calibrated transformation to the file----------------------" << std::endl;
    // writeCalibrationToFile(resultPath);

    // std::cout << "----------------------Publishing transformed poses----------------------" << std::endl;


    std::vector<Eigen::Matrix4d> calibratedEyePoses = transformPoses(eyePosesNewOrigin, calibratedTransformation);
    std::vector<Eigen::Matrix4d> initialTransformedEyePoses = transformPoses(eyePosesNewOrigin, initialTransformGuess);

    std::cout << "----------------------Error between gt and estimated transformation --------------------" << std::endl;
    double rotErrorTemp, transErrorTemp;
    comparePoses(initialTransformGuess, calibratedTransformation, rotErrorTemp,transErrorTemp);

    std::cout << "Rotation error(angle axis): " << rotErrorTemp << "Translation error: " << transErrorTemp <<std::endl;

    std::cout << "----------------------Before transformation --------------------" << std::endl;
    performanceEvaluation(handPosesNewOrigin, initialTransformedEyePoses);

    std::cout << "----------------------After transformation --------------------" << std::endl;
    performanceEvaluation(handPosesNewOrigin, calibratedEyePoses);

    assert(calibratedEyePoses.size() == eyePosesNewOrigin.size() && handPosesNewOrigin.size() == eyePosesNewOrigin.size() && "Pose size doesn't match");

    // ros::Publisher transformedLidarOdomPublisher = nh.advertise<nav_msgs::Odometry>("/transformed_lidar_odometry", 10000);
    // ros::Publisher lidarOdomPublisher = nh.advertise<nav_msgs::Odometry>("/lidar_odometry", 10000);
    // ros::Publisher lidarOdomInversePublisher = nh.advertise<nav_msgs::Odometry>("/lidar_odometry_inverse", 10000);
    // ros::Publisher gpsOdomPublisher = nh.advertise<nav_msgs::Odometry>("/gps_odometry", 10000);

//    handEyePosesComparision(handPosesNewOrigin ,eyePosesNewOrigin);

    int poseCount = eyePosesNewOrigin.size();
    int i=0;
    while(ros::ok())
    {

        publishNavPath(transformedLidarPathPublisher, transformedLidarNavPath, calibratedEyePoses[i]);
        publishNavPath(lidarPathPublisher, lidarNavPath, eyePosesNewOrigin[i]);
        publishNavPath(gpsPathPublisher, gpsNavPath, handPosesNewOrigin[i]);
        publishNavPath(lidarInversePathPublisher, lidarInverseNavPath, eyePosesNewOrigin[i].inverse());

        // publishOdometry(transformedLidarOdomPublisher, calibratedEyePoses[i]);
        // publishOdometry(lidarOdomPublisher,  eyePosesNewOrigin[i]);
        // publishOdometry(gpsOdomPublisher, handPosesNewOrigin[i]);
        // publishOdometry(lidarOdomInversePublisher, eyePosesNewOrigin[i].inverse());

        i++;
        if(i>=poseCount)
        {
            i=0;
            transformedLidarNavPath.poses.clear();
            gpsNavPath.poses.clear();
            lidarNavPath.poses.clear();
        }

        ros::spinOnce();
        r.sleep();
    }

    if(ros::isShuttingDown())
        std::cout << "----------------------ROS shut down----------------------" << std::endl;
    return 0;
}
