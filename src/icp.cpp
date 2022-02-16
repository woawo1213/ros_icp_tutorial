#include <iostream>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sum(new pcl::PointCloud<pcl::PointXYZRGB>);

    Eigen::Matrix4f tf_mul;

    tf_mul<< 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
    int count = 0;

    for(int i = 0; i < 13; i++)
    {
        std::string file_name = "desk.pcd";
        std::string file_name_a;
        std::string file_name_b;
        std::string counting_a = std::to_string(count);
        std::string counting_b = std::to_string(count+1);
        file_name_a="00"+counting_a+"_"+file_name;
        file_name_b="00"+counting_b+"_"+file_name;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/0000_desk.pcd", *cloud_static); //standard
        pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/"+file_name_b, *cloud_in); //source
        pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/"+file_name_a, *cloud_out); //target

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> t_gicp;
        t_gicp.setInputSource(cloud_in);
        t_gicp.setInputTarget(cloud_out);
        t_gicp.setMaximumIterations(2000);
        t_gicp.setTransformationEpsilon(1e-10);
        t_gicp.setMaxCorrespondenceDistance(10);
        t_gicp.setEuclideanFitnessEpsilon(1);
        t_gicp.setRANSACOutlierRejectionThreshold (1.5); 
        t_gicp.setRANSACIterations(10);

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> acc_gicp;
        acc_gicp.setInputSource(cloud_in);
        acc_gicp.setInputTarget(cloud_static);
        acc_gicp.setMaximumIterations(1000);
        acc_gicp.setTransformationEpsilon(1e-9);
        acc_gicp.setMaxCorrespondenceDistance(100);
        acc_gicp.setEuclideanFitnessEpsilon (1);
        acc_gicp.setRANSACOutlierRejectionThreshold (1e-5);
        acc_gicp.setRANSACIterations(10);

        pcl::PointCloud<pcl::PointXYZ> ::Ptr Align (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> ::Ptr Acc (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> ::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);

        t_gicp.align(*Align);
        acc_gicp.align(*Acc);

        std::cout << "has converged:" << t_gicp.hasConverged() << " score: " << t_gicp.getFitnessScore() << std::endl;
        std::cout << t_gicp.getFinalTransformation() << std::endl;
        Eigen::Matrix4f src2dst = t_gicp.getFinalTransformation();
        tf_mul = tf_mul * src2dst;

        // colorize(*cloud_in, *src_colored, {255, 0, 0});
        // colorize(*cloud_out, *tgt_colored, {0, 255, 0});
        // colorize(*Align, *align_colored, {0, 0, 255});
        colorize(*Acc, *align_colored, {255, 255, 255});//white
        *sum+=*align_colored;
        count++;
    }

    pcl::visualization::CloudViewer viewer("cloud viewer");
    // viewer.showCloud(src_colored,"src");
    // viewer.showCloud(tgt_colored,"dst");
    // viewer.showCloud(align_colored, "Align");
    viewer.showCloud(sum, "sum");

    while (!viewer.wasStopped ())
    {
    }
    return (0);
}