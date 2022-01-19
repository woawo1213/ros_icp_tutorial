#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//Downsampling a PointCloud using a VoxelGrid filter
//http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    
    // *.PCD 파일 읽기 (https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd)
    std::string read_file_name="table_scene_lms400.pcd";
    std::string write_file_name="table_scene_lms400_downsampled.pcd";
    std::string file_path="/home/jm/workspace/icp_ws/src/pcl_cpp_tutorial/pcd/";

    if(pcl::io::loadPCDFile<pcl::PointXYZ> (file_path+read_file_name,*cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file table_scene_lms400.pcd \n");
        return (-1);
    }

    // 포인트수 출력
    std::cout << "Input : " << cloud->points.size () << " (" << pcl::getFieldsList (*cloud) <<")"<< std::endl;

    // 오브젝트 생성 
    //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);              //입력
    sor.setLeafSize (0.02f, 0.02f, 0.02f); //leaf size  1cm 
    sor.filter (*cloud_filtered);          //출력 

    // 생성된 포인트클라우드 수 출력 
    std::cout << "Output : " << cloud_filtered->points.size () << " (" << pcl::getFieldsList (*cloud_filtered) <<")"<< std::endl;

    // 생성된 포인트클라우드 저장 
    pcl::io::savePCDFile<pcl::PointXYZ>(file_path+write_file_name, *cloud_filtered);
    return (0);
}