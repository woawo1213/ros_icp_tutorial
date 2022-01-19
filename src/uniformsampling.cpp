#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>


//https://github.com/PointCloudLibrary/pcl/blob/master/test/filters/test_uniform_sampling.cpp
//The UniformSampling class creates a 3D voxel grid (think about a voxel grid as a set of tiny 3D boxes in space) over the input point cloud data. Then, in each voxel (i.e., 3D box), all the points present will be approximated (i.e., downsampled) with their centroid. This approach is a bit slower than approximating them with the center of the voxel, but it represents the underlying surface more accurately.
//https://github.com/PointCloudLibrary/pcl/blob/master/tools/uniform_sampling.cpp

int main (int argc, char** argv)
{
    std::string read_file_name="table_scene_lms400.pcd";
    std::string write_file_name="table_scene_lms400_uniformsampled.pcd";
    std::string file_path="/home/jm/workspace/icp_ws/src/pcl_cpp_tutorial/pcd/";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // *.PCD 파일 읽기 
  pcl::io::loadPCDFile<pcl::PointXYZ> (file_path+read_file_name, *cloud);

  // 포인트수 출력
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  // 오프젝트 생성
  pcl::UniformSampling<pcl::PointXYZ> filter ; 
  filter.setInputCloud (cloud) ;     // 입력 
  filter.setRadiusSearch (0.01F) ;   // 탐색 범위 0.01F
  filter.filter (*cloud_filtered) ;  // 필터 적용 

  // 포인트수 출력  
  std::cout << "Filtered :" << cloud_filtered->width * cloud_filtered->height  << std::endl;  

  // 생성된 포인트클라우드 저장 
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (file_path+write_file_name, *cloud_filtered, false);

  return (0);
}