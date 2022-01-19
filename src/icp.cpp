#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/pcl_cpp_tutorial/pcd/madeby.pcd", *cloud_in); //sample
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/pcl_cpp_tutorial/pcd/madeby2.pcd", *cloud_out); //model


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    return (0);
}