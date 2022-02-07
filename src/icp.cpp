#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include<pcl/features/fpfh.h>//local
#include<pcl/features/cvfh.h>//global

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/cloud_viewer.h> 


void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> &color) 
{
    int N = pc.points.size();
    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i) 
    {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}


int main (int argc, char** argv)
{


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sum(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored3(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored4(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored5(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored6(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored7(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    Eigen::Matrix4f tf_mul;
    tf_mul<< 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
    int count = 0;

    for(int i = 0; i < 15; i++)
    {
        std::string file_name = "desk.pcd";
        std::string file_name_a;
        std::string file_name_b;
        std::string counting_a = std::to_string(count);
        std::string counting_b = std::to_string(count+1);
        file_name_a="00"+counting_a+"_"+file_name;
        file_name_b="00"+counting_b+"_"+file_name;

        pcl::PointCloud<pcl::PointXYZ>::Ptr init (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
        

        pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/0000_desk.pcd", *cloud_static);
        pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/"+file_name_b, *cloud_in); //source
        pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/"+file_name_a, *cloud_out); //target

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setInputCloud (cloud_in);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.1);
        seg.segment (*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_in);
        extract.setIndices (inliers);
        extract.setNegative (true);//false
        extract.filter (*init);



        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(cloud_in);
        gicp.setInputTarget(cloud_out);
        gicp.setMaximumIterations(2000);
        gicp.setTransformationEpsilon(1e-10);
        gicp.setMaxCorrespondenceDistance(10);
        gicp.setEuclideanFitnessEpsilon(1);
        gicp.setRANSACOutlierRejectionThreshold (1.5); 
        gicp.setRANSACIterations(10);

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> accgicp;
        accgicp.setInputSource(init);
        accgicp.setInputTarget(cloud_static);
        accgicp.setMaximumIterations(1000);
        accgicp.setTransformationEpsilon(1e-9);
        accgicp.setMaxCorrespondenceDistance(100);
        accgicp.setEuclideanFitnessEpsilon (1);
        accgicp.setRANSACOutlierRejectionThreshold (1e-5);
        accgicp.setRANSACIterations(10);

        pcl::PointCloud<pcl::PointXYZ> ::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> ::Ptr Acc (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> ::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);

        gicp.align(*Final);
        accgicp.align(*Acc);
        

        std::cout << "has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << std::endl;
        std::cout << gicp.getFinalTransformation() << std::endl;
        Eigen::Matrix4f src2dst = gicp.getFinalTransformation();
        // std::cout << accgicp.getFinalTransformation() << std::endl;
        // Eigen::Matrix4f src2dst = accgicp.getFinalTransformation();
        // tf_mul = tf_mul * src2dst;

        // pcl::transformPointCloud(*Final, *tgt, tf_mul);
        // pcl::transformPointCloud(*Acc, *tgt, tf_mul);
        colorize(*Acc, *align_colored1, {255, 255, 255});


        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

        // colorize(*cloud_in, *src_colored, {255, 0, 0});//r
        // colorize(*cloud_out, *tgt_colored, {0, 255, 0});//g
        
        // switch(i)
        // {
        //     case 0:
        //         colorize(*tgt, *align_colored1, {255, 0, 0});
        //     case 1:
        //         colorize(*tgt, *align_colored2, {255, 140, 0});
        //     case 5:
        //         colorize(*tgt, *align_colored3, {255, 255, 0});
        //     case 9:
        //         colorize(*tgt, *align_colored4, {0, 255, 0});
        //     case 10:
        //         colorize(*tgt, *align_colored5, {128, 0, 128});
        //     // case 6:
        //         // colorize(*tgt, *align_colored7, {128, 0, 128});
        //     case 17:
        //         colorize(*tgt, *align_colored7, {255, 255, 255});

        //     // default:
        //     //     colorize(*Final, *align_colored7, {255, 255, 255});
        // }
        // colorize(*Final, *align_colored, {255, 255, 255});//y
        *sum+=*align_colored1;
        // *sum += *tgt;
        count++;
    }

    pcl::visualization::CloudViewer viewer("cloud viewer");
    // viewer.showCloud(src_colored,"src");
    // viewer.showCloud(tgt_colored,"dst");
    // viewer.showCloud(align_colored, "final");
    viewer.showCloud(sum, "sum");
    // viewer.showCloud(align_colored1, "align1");
    // viewer.showCloud(align_colored2, "align2");
    // viewer.showCloud(align_colored3, "align3");
    // viewer.showCloud(align_colored4, "align4");
    // viewer.showCloud(align_colored5, "align5");
    // viewer.showCloud(align_colored6, "align6");
    // viewer.showCloud(align_colored7, "align7");

    while (!viewer.wasStopped ())
    {
    }
    return (0);
}