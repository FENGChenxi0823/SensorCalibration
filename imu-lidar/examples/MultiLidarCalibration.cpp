/*
 * Descriptiong: Multi-lidar calibration
 * Author: Ran Tang
 * Date: June 06, 2018
*/

#include<string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "sensor_calibration/MultiLidarOptimization.h"

void readDataFromFile(const std::string& filePath, std::vector< std::vector<pcl::PointCloud<pcl::PointXYZI> > >& lidarPoints,
                      std::vector< Eigen::Matrix4d>& transformations)
{
}

void writeDataToFile(const std::string& filePath, const std::vector<pcl::PointCloud<pcl::PointXYZI> >& lidarPoints,
                      const std::vector< Eigen::Matrix4d>& transformations)
{
}

int main(int argc, char** argv)
{
    assert(argc==2 && "Please provide config file!");

    std::string configFilePath = argv[1];

    std::vector< std::vector<pcl::PointCloud<pcl::PointXYZI> > > lidaraPoints;
    std::vector< Eigen::Matrix4d> transformations;


    readDataFromFile(configFilePath, lidaraPoints, transformations);

    SensorCalibration::MultiLidarOptimization multiLidarOptimizer(configFilePath);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> registeredPoints;

    for(int i=0; i< lidaraPoints.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI> currRegisteredPoints = multiLidarOptimizer.optimize(lidaraPoints[i], transformations);
        registeredPoints.push_back(currRegisteredPoints);
    }

    writeDataToFile(configFilePath, registeredPoints,transformations);

    return 0;
}
