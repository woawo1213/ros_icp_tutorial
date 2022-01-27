#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

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
    int count=4;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sum(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored3(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored4(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored5(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored6(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored7(new pcl::PointCloud<pcl::PointXYZRGB>);

    Eigen::Matrix4f tf_mul;
    tf_mul<< 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    for(int i=0;i<6;i++)
    {
        std::string file_name = "desk.pcd";
        std::string file_name_a;
        std::string file_name_b;
        std::string counting_a = std::to_string(count);
        std::string counting_b = std::to_string(count+1);
        file_name_a="00"+counting_a+"_"+file_name;
        file_name_b="00"+counting_b+"_"+file_name;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/"+file_name_b, *cloud_in); //sample
        pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/"+file_name_a, *cloud_out); //model

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(cloud_in);
        gicp.setInputTarget(cloud_out);
        gicp.setRANSACIterations(100);
        gicp.setMaxCorrespondenceDistance(1.0);
        gicp.setTransformationEpsilon(1e-10);
        gicp.setMaximumIterations(1000);
        gicp.setEuclideanFitnessEpsilon (1);
        gicp.setRANSACOutlierRejectionThreshold (1.5);
        
        

        pcl::PointCloud<pcl::PointXYZ> ::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
        gicp.align(*Final);
        

        std::cout << "has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << std::endl;
        std::cout << gicp.getFinalTransformation() << std::endl;
        Eigen::Matrix4f src2dst = gicp.getFinalTransformation();
        tf_mul = tf_mul * src2dst;
        pcl::transformPointCloud(*Final, *Final, tf_mul);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

        colorize(*cloud_in, *src_colored, {255, 0, 0});//r
        colorize(*cloud_out, *tgt_colored, {0, 255, 0});//g
        switch(i)
        {
            case 0:
                colorize(*Final, *align_colored1, {255, 0, 0});
            case 1:
                colorize(*Final, *align_colored2, {255, 140, 0});
            case 2:
                colorize(*Final, *align_colored3, {255, 255, 0});
            case 3:
                colorize(*Final, *align_colored4, {0, 128, 0});
            case 4:
                colorize(*Final, *align_colored5, {0, 0, 255});
            case 5:
                colorize(*Final, *align_colored6, {75, 0, 130});
            case 6:
                colorize(*Final, *align_colored7, {128, 0, 128});
        }
        // colorize(*Final, *align_colored, {255, 255, 0});//y
        // *sum+=*align_colored;
        count++;
    }

    pcl::visualization::CloudViewer viewer("cloud viewer");
    // viewer.showCloud(src_colored,"src");
    // viewer.showCloud(tgt_colored,"dst");
    // viewer.showCloud(align_colored, "final");
    viewer.showCloud(sum, "sum");
    viewer.showCloud(align_colored1, "align1");
    viewer.showCloud(align_colored2, "align2");
    viewer.showCloud(align_colored3, "align3");
    viewer.showCloud(align_colored4, "align4");
    viewer.showCloud(align_colored5, "align5");
    viewer.showCloud(align_colored6, "align6");
    viewer.showCloud(align_colored7, "align7");

    while (!viewer.wasStopped ())
    {
    }
    return (0);
}