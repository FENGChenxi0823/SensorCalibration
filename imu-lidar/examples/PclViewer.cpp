#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cout << "Provide pcd file to be visualized" << std::endl;
    }
    std::string filename = argv[1];
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(filename, *cloud);

    pcl::visualization::CloudViewer viewer("Cloud Point Viewer");
    viewer.showCloud(cloud);

    while(!viewer.wasStopped())
    {

    }

    return 0;
}
