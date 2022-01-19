#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

// PCL specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//input terminal
#include <string>    
#include <vector>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class ICPTester
{
    ros::Publisher src_pub_; // source
    ros::Publisher dst_pub_; // destination
    ros::Publisher ali_pub_; // align
    ros::Publisher acc_pub_; // accumulate
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    pcl::PointCloud<pcl::PointXYZ> *acc_pc = new pcl::PointCloud<pcl::PointXYZ>; // accumulate point cloud 
    pcl::PointCloud<pcl::PointXYZ> *save_pc = new pcl::PointCloud<pcl::PointXYZ>; // save previous point cloud
    
    Eigen::Matrix4f tf_sum;// 초기 depth frame tf 넣어줘야하나?
    int count=0;

    tf::TransformListener tf_;

public:
    ICPTester()
    {
        //depth optical frame tf
        tf_sum<< 0, 0, 1, 0,
                -1, 0, 0, 0,
                 0,-1, 0, 0,
                 0, 0, 0, 1;

        // tf_sum<< 1, 0, 0, 0,
        //          0, 1, 0, 0,
        //          0, 0, 1, 0,
        //          0, 0, 0, 1;

        ali_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("align_output", 1000);
        src_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("source_output", 1000);
        dst_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("destination_output", 1000);
        acc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("accumulate_output", 1000);
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/astra/depth/points", 1, &ICPTester::cloud_cb, this);
    };

    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
    {
        // Container for original & filtered data
        // sensor_msgs::PointCloud2ConstPtr 는 택배상자
        // pcl::PCLPointCloud2 는 내용물
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        
        pcl::PCLPointCloud2 cloud_filtered;
        pcl::PCLPointCloud2 cloud_filtered2;
        pcl::PCLPointCloud2 cloud_filtered3;
        pcl::PCLPointCloud2 cloud_filtered4;

        pcl::PointCloud<pcl::PointXYZ>::Ptr src_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr dst_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tf_tgt(new pcl::PointCloud<pcl::PointXYZ>);
        
        sensor_msgs::PointCloud2 output;
        sensor_msgs::PointCloud2 output2;
        sensor_msgs::PointCloud2 output3;
        sensor_msgs::PointCloud2 output4;

        // Convert to PCL data type , 상자 까기
        pcl_conversions::toPCL(*cloud_msg, *cloud);

        // Perform the actual filtering
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloudPtr);
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
        sor.filter (cloud_filtered);

        //convert pcl::PCLPointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
        pcl::fromPCLPointCloud2(cloud_filtered, *dst_pc);
        
        // icp
        // skip initial source point cloud input
        if((count > 0)&&(count < 1000))
        {
            ROS_INFO("count: %d",count);
            *src_pc = *save_pc;

            // icp
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

            // Registration 시행
            // Set the input source and target
            icp.setInputSource(src_pc);
            icp.setInputTarget(dst_pc);
            // Set the max correspondence distance to 5cm
            // icp.setMaxCorrespondenceDistance(5);
            // Set the transformation epsilon (criterion 2)
            icp.setTransformationEpsilon(1e-8);
            // Set the maximum number of iterations (criterion 1)
            icp.setMaximumIterations(50);
            // Perform the alignment
            icp.align(*align);

            // Eigen::Isometry3f::
            // Obtain the transformation that aligned src_pc to align
            Eigen::Matrix4f src2dst = icp.getFinalTransformation();
            

            double score = icp.getFitnessScore();
            bool is_converged = icp.hasConverged();

            std::cout<<"icp matrix: "<<std::endl;
            std::cout<<src2dst<<std::endl;

            // ROS_INFO("XYZ: %f %f %f ",src2tgt(0,3),src2tgt(1,3),src2tgt(2,3));

            std::cout<<"score: "<<score<<std::endl;
            std::cout<<"converged: "<<is_converged<<std::endl;
            std::cout<<"###############################################"<<std::endl;

            //multiple tf
            tf_sum = src2dst * tf_sum;
            std::cout<<"tf matrix: "<<std::endl;
            std::cout<< tf_sum <<std::endl;

            //tranform init source cloud
            pcl::transformPointCloud(*dst_pc, *tf_tgt, tf_sum);

            //accumulate aling point cloud
            acc_pc->header.frame_id = dst_pc->header.frame_id;
            *acc_pc += *tf_tgt;
            
        }

        //convert pcl::PointCloud<pcl::PointXYZ> -> convert pcl::PCLPointCloud2
        pcl::toPCLPointCloud2(*src_pc, cloud_filtered2);
        pcl::toPCLPointCloud2(*align, cloud_filtered3);
        pcl::toPCLPointCloud2(*acc_pc, cloud_filtered4);

        //convert convert pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
        pcl_conversions::moveFromPCL(cloud_filtered, output);
        pcl_conversions::moveFromPCL(cloud_filtered2, output2);
        pcl_conversions::moveFromPCL(cloud_filtered3, output3);
        pcl_conversions::moveFromPCL(cloud_filtered4, output4);

        //publish
        dst_pub_.publish(output);
        src_pub_.publish(output2);
        ali_pub_.publish(output3);
        acc_pub_.publish(output4); 

        //save src point cloud
        *save_pc = *dst_pc;
        count++;
    }

};


int main (int argc, char** argv)
{
    ros::init (argc, argv, "test_icp");

    ICPTester it;
    ros::Rate r(1);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}