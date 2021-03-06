#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//Plane model segmentation

int main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        inlierPoints (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        inlierPoints_neg (new pcl::PointCloud<pcl::PointXYZRGB>);

    // *.PCD 파일 읽기 
    pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/jm/workspace/icp_ws/src/pcl_cpp_tutorial/pcd/table_top_passthrough.pcd", *cloud);

    // 포인트수 출력
    std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

    // Object for storing the plane model coefficients.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


    // 오프젝트 생성 Create the segmentation object.
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
    seg.setInputCloud (cloud);                 //입력 
    seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델  // Configure the object to look for a plane.
    seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
    seg.setMaxIterations (1000);               //최대 실행 수
    seg.setDistanceThreshold (0.01);          //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
    //seg.setRadiusLimits(0, 0.1);     // cylinder경우, Set minimum and maximum radii of the cylinder.
    seg.segment (*inliers, *coefficients);    //세그멘테이션 적용 


    //추정된 평면 파라미터 출력 (eg. ax + by + cz + d = 0 ).
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, *inliers, *inlierPoints);
    pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/jm/workspace/icp_ws/src/pcl_cpp_tutorial/pcd/SACSegmentation_result.pcd", *inlierPoints);


    //[옵션]] 바닥 제거 결과 얻기 
    //Extracting indices from a PointCloud
    //http://pointclouds.org/documentation/tutorials/extract_indices.php
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);//false
    extract.filter (*inlierPoints_neg);
    pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/jm/workspace/icp_ws/src/pcl_cpp_tutorial/pcd/SACSegmentation_result_neg.pcd", *inlierPoints_neg);

    return (0);


}