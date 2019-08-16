#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>


int 
main (int argc, char** argv)
{

  std::string filepath = "../";
  if(argc == 2){
    filepath = filepath + argv[1];
  }
  else{
    std::cout<< argc <<std::endl;
    std::cerr <<"wrong input name"<< std::endl;
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPLYFile<pcl::PointXYZ> (filepath, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read ply file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from lidar_points.ply with the following fields: "
            << std::endl;
  pcl::visualization::CloudViewer viewer("Cloud viewer");

  viewer.showCloud(cloud);

  while (!viewer.wasStopped ())
  {
  }
  return (0);
}
