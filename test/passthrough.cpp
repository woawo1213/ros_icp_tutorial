#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>

//Filtering a PointCloud using a PassThrough filter
//http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough

int main (int argc, char** argv)
{
    std::string read_file_name="corner.pcd";
    std::string write_file_name="corner_passthrough.pcd";
    std::string file_path="/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop.pcd)
    pcl::io::loadPCDFile<pcl::PointXYZ> (file_path+read_file_name, *cloud);

    // 포인트수 출력
    std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

    // 오브젝트 생성 
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);                
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.3, 0.7);
    pass.filter (*cloud_filtered);
    // pass.setInputCloud (cloud_filtered); 
    // pass.setFilterFieldName ("y");
    // pass.setFilterLimits (0.0, 0.3);
    // pass.filter (*cloud_filtered);
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");             
    pass.setFilterLimits (0.7, 1.9);          
    pass.setFilterLimitsNegative (true);     
    pass.filter (*cloud_filtered);             

    // 포인트수 출력
    std::cout << "Filtered :" << cloud_filtered->width * cloud_filtered->height  << std::endl;  

    // 저장 
    pcl::io::savePCDFile<pcl::PointXYZ>(file_path+write_file_name, *cloud_filtered); //Default binary mode save

    return (0);
}