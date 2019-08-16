/*
 * Descriptiong: Multi-lidar optimization core
 * Author: Ran Tang
 * Date: June 06, 2018
*/
#ifndef _MULTILIDAROPTIMIZATION_H_
#define _MULTILIDAROPTIMIZATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ceres/ceres.h>

#include "sensor_calibration/ErrorTerm.h"

namespace SensorCalibration
{

class MultiLidarOptimization
{
public:
    MultiLidarOptimization();
    MultiLidarOptimization(const std::string& configFile);
    MultiLidarOptimization(double leafSize, double mRadius, int mMaxIterations, int mMaxNeighbors);
    ~MultiLidarOptimization();

    pcl::PointCloud<pcl::PointXYZI> optimize(std::vector<pcl::PointCloud<pcl::PointXYZI> >& lidarPoints, std::vector< Eigen::Matrix4d>& transformations, bool isSummary);
private:
    double mLeafSize;
    double mRadius;
    int mMaxIterations;
    int mMaxNeighbors;
    double mRotationEpsilon;
    double mTranslationEpsilon;


    double mRotationA[4];
    double mTranslationA[3];
    double mRotationB[4];
    double mTranslationB[3];



    std::vector<ErrorTerm*> mErrorTerms;


    std::vector<std::vector<int>> ceresOptimization(const std::vector<pcl::PointCloud<pcl::PointXYZI> >& lidarPoints, const std::vector<pcl::PointCloud<pcl::Normal>> &normals, std::vector< Eigen::Matrix4d>& transformations, bool isSummary);
    void g2oOptimization(const std::vector<pcl::PointCloud<pcl::PointXYZI> >& lidarPoints, const std::vector<pcl::PointCloud<pcl::Normal>> &normals, std::vector< Eigen::Matrix4d>& transformations, bool isSummary);
    void configureOptions(ceres::Solver::Options &options);
    void copyTransformationToArray(const Eigen::Matrix4d &transform, double rotation[4], double translation[3]);
    void comparePoses(const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2, double & rotError, double & transError);
    Eigen::Matrix4d computeTransformationFromArray(const double rotation[4], const double translation[3]);
    std::vector<pcl::PointCloud<pcl::Normal>> computeSurfaceNormal(const std::vector<pcl::PointCloud<pcl::PointXYZI> > &points);

};

}

#endif
