#include<string>
#include <Eigen/Dense>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_calibration/calibration_tool.h>

#include "sensor_calibration/MultiLidarOptimization.h"

Eigen::Affine3f imu2Lidar1;
Eigen::Affine3f imu2Lidar2;
std::vector<pcl::PointCloud<pcl::PointXYZI>> points;
std::vector<pcl::PointCloud<pcl::PointXYZI>> pointsCalibration;
ros::Publisher pointPub, pointPub2, pointPub3;
SensorCalibration::MultiLidarOptimization multiLidarOptimizer;


Eigen::Matrix4f convertStringToTF(const std::string &str)
{
    std::vector<std::string> vec_str = split(str, ", []");
    double x, y, z, roll, pitch, yaw;
    x = std::atof(vec_str[0].c_str());
    y = std::atof(vec_str[1].c_str());
    z = std::atof(vec_str[2].c_str());
    roll = std::atof(vec_str[3].c_str());
    pitch = std::atof(vec_str[4].c_str());
    yaw = std::atof(vec_str[5].c_str());
    return computeTransformMatrix(x, y, z, roll, pitch, yaw);
}

void printResults(const Eigen::Matrix4f &transformation, const std::string &name)
{
    Eigen::Quaternionf qc(transformation.block<3,3>(0,0));
    tf::Quaternion qtfq(qc.x(), qc.y(), qc.z(), qc.w());
    tf::Matrix3x3 qmt33(qtfq);
    double troll, tpitch, tyaw;
    qmt33.getRPY(troll, tpitch, tyaw);
    std::cout << name << ": "<< transformation << std::endl;
    std::cout << name << " in rpy: "<< std::endl;
    std::cout << "roll: " << troll << " pitch: " << tpitch << " yaw: " << tyaw << std::endl;
}

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg1, const sensor_msgs::PointCloud2ConstPtr &msg2)
{
    std::cout << "receive point cloud msgs" << std::endl;
    pcl::PointCloud<pcl::PointXYZI> currentPoints1;
    pcl::fromROSMsg(*msg1, currentPoints1);
    pcl::PointCloud<pcl::PointXYZI> currentTransformedPoints1;
    pcl::transformPointCloud(currentPoints1, currentTransformedPoints1, imu2Lidar1);

    pcl::PointCloud<pcl::PointXYZI> currentPoints2;
    pcl::fromROSMsg(*msg2, currentPoints2);

    pcl::PointCloud<pcl::PointXYZI> currentTransformedPoints2;
    pcl::transformPointCloud(currentPoints2, currentTransformedPoints2, imu2Lidar2);

    currentTransformedPoints1 += currentTransformedPoints2;

    points.push_back(currentTransformedPoints1);

    std::vector<pcl::PointCloud<pcl::PointXYZI> >  lidaraPoints;
    lidaraPoints.push_back(currentPoints1);
    lidaraPoints.push_back(currentPoints2);
//    lidaraPoints.push_back(currentPoints2);
//    lidaraPoints.push_back(currentPoints1);

    std::vector< Eigen::Matrix4d> transformations;
    transformations.push_back(imu2Lidar1.matrix().cast<double>());
    transformations.push_back(imu2Lidar2.matrix().cast<double>());
//    transformations.push_back(imu2Lidar2.matrix().cast<double>());
//    transformations.push_back(imu2Lidar1.matrix().cast<double>());

    pcl::PointCloud<pcl::PointXYZI> currRegisteredPoints = multiLidarOptimizer.optimize(lidaraPoints, transformations, false);
//    pcl::PointCloud<pcl::PointXYZI> currRegisteredPoints;
    pointsCalibration.push_back(currRegisteredPoints);

    imu2Lidar1.matrix() = transformations[0].cast<float>();
    imu2Lidar2.matrix() = transformations[1].cast<float>();

//    std::cout << "imu2Lidar1: " << imu2Lidar1.matrix() << std::endl;
//    std::cout << "imu2Lidar2: " << imu2Lidar2.matrix() << std::endl;
    printResults(imu2Lidar1.matrix(), "imu2Lidar1");
    printResults(imu2Lidar2.matrix(), "imu2Lidar2");

    sensor_msgs::PointCloud2 pointsMsg;
    pcl::toROSMsg(currentTransformedPoints1, pointsMsg);
    pointPub.publish(pointsMsg);

    sensor_msgs::PointCloud2 pointsMsg2;
    pcl::toROSMsg(currRegisteredPoints, pointsMsg2);
    pointsMsg2.header.frame_id = "front_left_vel_link";
    pointPub2.publish(pointsMsg2);

    sensor_msgs::PointCloud2 pointsMsg3;
    pcl::toROSMsg(currentPoints2, pointsMsg3);
    pointsMsg3.header.frame_id = "front_left_vel_link";
    pointPub3.publish(pointsMsg3);
}

void serializePointCloud(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& pclPointLidar, const std::string& filePath)
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

    for(int i=0; i<pclPointLidar.size(); i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(5) << i;
        std::string filename = filePath + "/" + ss.str() + ".pcd";
        pcl::io::savePCDFileASCII (filename, pclPointLidar[i]);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "handeye_example_node");
    ros::NodeHandle nh("~");

    std::string imu2Lidar1String, imu2Lidar2String;
//    nh.param("imu_to_lidar1", imu2Lidar1String, std::string("[1.65441 1.22997 -5.4991 0.00810555 -0.00896183 -0.0172249]"));
//    nh.param("imu_to_lidar2", imu2Lidar2String, std::string("[1.69435 -1.18031 -2.06763 0.011769 -0.00641879 0.0044906]"));

    nh.param("imu_to_lidar1", imu2Lidar1String, std::string("[1.65441 1.22997 0.0 0.00810555 -0.00896183 -0.0172249]"));
    nh.param("imu_to_lidar2", imu2Lidar2String, std::string("[1.69435 -1.18031 0.2 0.011769 -0.00641879 0.0044906]"));

    imu2Lidar1.matrix() = convertStringToTF(imu2Lidar1String);
    imu2Lidar2.matrix() = convertStringToTF(imu2Lidar2String);

    Eigen::Matrix4f lidar1ToLidar2 = imu2Lidar1.matrix().inverse() * imu2Lidar2.matrix();
//    std::cout << "original imu2Lidar1" << imu2Lidar1.matrix() << std::endl;
//    std::cout << "original imu2Lidar2" << imu2Lidar2.matrix() << std::endl;
    printResults(imu2Lidar1.matrix(), "original imu2Lidar1");
    printResults(imu2Lidar2.matrix(), "original imu2Lidar2");
    printResults(lidar1ToLidar2, "lidar1ToLidar2");

    std::string lidarTopic1, lidarTopic2;
    nh.param("lidar_topic1", lidarTopic1, std::string("/rr/front_left/velodyne_points"));
    nh.param("lidar_topic2", lidarTopic2, std::string("/rr/front_right/velodyne_points"));

    std::string resultPath;
    nh.param("resultPath", resultPath, std::string("/home/ran/Documents/Projects/catkin_ws/src/imu-lidar/imu-lidar/result"));

    message_filters::Subscriber<sensor_msgs::PointCloud2> lidarSub1(nh, lidarTopic1, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidarSub2(nh, lidarTopic2, 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync2(syncPolicy(10), lidarSub1, lidarSub2);
    sync2.registerCallback(boost::bind(&lidarCallback, _1, _2));

    pointPub = nh.advertise<sensor_msgs::PointCloud2>("/registered_points", 10);
    pointPub2 = nh.advertise<sensor_msgs::PointCloud2>("/registered_points_calibration", 10);
    pointPub3 = nh.advertise<sensor_msgs::PointCloud2>("/right_points", 10);

    ros::spin();

    if(ros::isShuttingDown())
    {
        std::cout << "saving files" << std::endl;
        std::cout << "Original point cloud size: " << points.size() << std::endl;
        std::cout << "Calibrated point cloud size: " << pointsCalibration.size() << std::endl;

        // write registered point cloud to file
        std::string pointPath = resultPath + "/registeredCloud";
        serializePointCloud(points, pointPath);

        std::string pointPath2 = resultPath + "/registeredCloudCalibration";
        serializePointCloud(pointsCalibration, pointPath2);
    }

    return 0;
}
