#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <ctime>

//How to use a KdTree to search
//http://pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search
//Commnets : Hunjung, Lim (hunjung.lim@hotmail.com)

int main (int argc, char** argv)
{

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);    
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/jm/workspace/icp_ws/src/pcl_cpp_tutorial/pcd/cloud_cluster_0.pcd", *cloud);

  // 시각적 확인을 위해 색상 통일 (255,255,255)
  for (size_t i = 0; i < cloud->points.size(); ++i){
  cloud->points[i].r = 255;
  cloud->points[i].g = 255;
  cloud->points[i].b = 255;
  }

  //KdTree 오브젝트 생성 
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud);    //입력 

     //기준점(searchPoint) 설정 방법 #1(x,y,z 좌표 지정)
     //pcl::PointXYZRGB searchPoint;
     //searchPoint.x = 0.026256f;
     //searchPoint.y = -1.464739f;
     //searchPoint.z = 0.929567f;
  //기준점(searchPoint) 설정 방법 #2(3000번째 포인트)
  pcl::PointXYZRGB searchPoint = cloud->points[3000]; 

  //기준점 좌표 출력 
  std::cout << "searchPoint :" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z  << std::endl;


  //기준점에서 가까운 순서중 K번째까지의 포인트 탐색 (K nearest neighbor search)
  int K = 10;   // 탐색할 포인트 수 설정 
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    //시각적 확인을 위하여 색상 변경 (0,255,0)
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    {
      cloud->points[pointIdxNKNSearch[i]].r = 0;
      cloud->points[pointIdxNKNSearch[i]].g = 255;
      cloud->points[pointIdxNKNSearch[i]].b = 0;
    }
  }

  // 탐색된 점의 수 출력 
  std::cout << "K = 10: " << pointIdxNKNSearch.size() << std::endl;


  // 기준점에서 지정된 반경내 포인트 탐색 (Neighbor search within radius)
  float radius = 0.02; //탐색할 반경 설정(Set the search radius)
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    //시각적 확인을 위하여 색상 변경 (0,0,255)
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
        cloud->points[pointIdxRadiusSearch[i]].r = 0;
        cloud->points[pointIdxRadiusSearch[i]].g = 0;
        cloud->points[pointIdxRadiusSearch[i]].b = 255;
        }
  }

  // 탐색된 점의 수 출력 
  std::cout << "Radius 0.02 nearest neighbors: " << pointIdxRadiusSearch.size() << std::endl;

  // 생성된 포인트클라우드 저장 
  pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/jm/workspace/icp_ws/src/pcl_cpp_tutorial/pcd/Kdtree_AllinOne.pcd", *cloud);

  return 0;
}