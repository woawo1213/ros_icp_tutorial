#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/keypoints/harris_3d.h> 
#include <boost/thread/thread.hpp> 
#include <stdlib.h> 
#include <iostream> 
using namespace std ; 

int main(int argc, char** argv) 
{
    pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>) ; 
    pcl::io::loadPCDFile<pcl::PointXYZ>( "/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/incorner.pcd" , *cloud); boost:: shared_ptr <pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "3D Viewer" ));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud" ); 

    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris; 
    harris.setInputCloud(cloud); 

    harris.setNonMaxSupression( true ); 
    harris.setRadius( 0.6f ); 

    harris.setThreshold( 0.01f ); 

    pcl::PointCloud<pcl::PointXYZI>:: Ptr cloud_out_ptr(new pcl::PointCloud<pcl::PointXYZI>) ; 

    harris.compute(*cloud_out_ptr); 

    pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_harris_ptr(new pcl::PointCloud<pcl::PointXYZ>) ; 
    int size = cloud_out_ptr->size(); 
    pcl::PointXYZ point; 

    for ( int i = 0 ; i<size; ++i) 
    {
        point.x = cloud_out_ptr->at(i).x; 
        point.y = cloud_out_ptr->at(i).y; 
        point.z = cloud_out_ptr->at(i).z; 
        cloud_harris_ptr->push_back(point);
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler(cloud_harris_ptr, 0 , 255 , 0 ); 

    viewer->addPointCloud<pcl::PointXYZ>(cloud_harris_ptr, harris_color_handler, "harris" ); 

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10 , "harris" ); 
    while (!viewer->wasStopped())
    {
        viewer->spinOnce( 100 ); 
    }
    system( "pause" ); 
    return 0 ; 
}