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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// PCL specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <string>

class ICPTester
{
    ros::Publisher src_pub_; // source
    ros::Publisher dst_pub_; // destination
    ros::Publisher ali_pub_; // align
    ros::Publisher acc_pub_; // accumulate
    ros::Publisher tgt_pub_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    pcl::PointCloud<pcl::PointXYZ> *acc_pc = new pcl::PointCloud<pcl::PointXYZ>; // accumulate point cloud 
    pcl::PointCloud<pcl::PointXYZ> *save_pc = new pcl::PointCloud<pcl::PointXYZ>; // save previous point cloud
    
    Eigen::Matrix4f tf_mul;// 초기 depth frame tf or identity 넣어줘야하나?
    std::string file_path="/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/";
    
    int count=0;

public:
    ICPTester()
    {
        //depth optical frame tf
        // tf_mul<< 0, 0, 1, 0,
        //         -1, 0, 0, 0,
        //          0,-1, 0, 0,
        //          0, 0, 0, 1;

        // identity
        tf_mul<< 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

        ali_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("align_output", 1000);
        src_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("source_output", 1000);
        dst_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("destination_output", 1000);
        acc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("accumulate_output", 1000);
        tgt_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("tgt_output", 1000);
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
        pcl::PCLPointCloud2 cloud_filtered5;

        pcl::PointCloud<pcl::PointXYZ>::Ptr src_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr dst_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tf_tgt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd (new pcl::PointCloud<pcl::PointXYZ>);
        
        sensor_msgs::PointCloud2 output;
        sensor_msgs::PointCloud2 output2;
        sensor_msgs::PointCloud2 output3;
        sensor_msgs::PointCloud2 output4;
        sensor_msgs::PointCloud2 output5;

        // std::string write_file_name = "desk.pcd";
        // std::string counting = std::to_string(count);
        // write_file_name="0"+counting+"_"+write_file_name;
        // Convert to PCL data type , 상자 까기
        pcl_conversions::toPCL(*cloud_msg, *cloud);

        // Perform the actual filtering
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloudPtr);
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
        sor.filter (cloud_filtered);

        // //convert pcl::PCLPointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
        pcl::fromPCLPointCloud2(cloud_filtered, *src_pc);
        

        // pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (src_pc));
        // pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        // ransac.setDistanceThreshold (.01);
        // ransac.computeModel();
        // ransac.getInliers(inliers);
        

        // pcl::copyPointCloud<pcl::PointXYZ>(*src_pc, inliers, *final);

        // remove floor
        // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        // // Create the segmentation object
        // pcl::SACSegmentation<pcl::PointXYZ> seg;
        // // Optional
        // seg.setOptimizeCoefficients (true);
        // // Mandatory
        // seg.setInputCloud (src_pc);
        // seg.setModelType (pcl::SACMODEL_PLANE);
        // seg.setMethodType (pcl::SAC_RANSAC);
        // seg.setMaxIterations (1000);
        // seg.setDistanceThreshold (0.1);
        // seg.segment (*inliers, *coefficients);

        // pcl::ExtractIndices<pcl::PointXYZ> extract;
        // extract.setInputCloud (src_pc);
        // extract.setIndices (inliers);
        // extract.setNegative (true);//false
        // extract.filter (*fixed_pc);


        // icp
        // skip initial source point cloud input
        if((count > 0)&&(count < 1000))
        {
            ROS_INFO("count: %d",count);
            //allocate previous point cloud
            *dst_pc = *save_pc;

            // icp
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            // Registration 시행
            // Set the input source and target
            icp.setInputSource(src_pc);
            icp.setInputTarget(dst_pc);
            // Set the max correspondence distance, src dst 사이의 최대거리 설정, 결과에 영향을 많이줌
            icp.setMaxCorrespondenceDistance(10);
            // Set the transformation epsilon ,
            icp.setTransformationEpsilon(1e-10);
            // Set the maximum number of iterations,  registration 될때까지 반복횟수
            icp.setMaximumIterations(1000);
            // Perform the alignment
            icp.align(*align);

            
            // Obtain the transformation that aligned src_pc to align
            Eigen::Matrix4f src2dst = icp.getFinalTransformation();
            // Eigen::Isometry 로 바꿔서도 해보기
            
            double score = icp.getFitnessScore();
            bool is_converged = icp.hasConverged();

            ROS_INFO("icp matrix");
            std::cout<<src2dst<<std::endl;

            ROS_INFO("score: %lf",score);//src dst 거리의 평균, 작을수록 icp good
            // ROS_INFO("converged: ",is_converged);
            std::cout<<"###############################################"<<std::endl;

            //multiple tf
            // src2dst=src2dst.reverse();
            tf_mul = tf_mul * src2dst;
            ROS_INFO("tf matrix");
            std::cout<< tf_mul <<std::endl;

            //print camera pose
            std::cout<<"###############################################"<<std::endl;
            ROS_INFO("camera pose t x:%f y:%f z:%f", tf_mul(0,3), tf_mul(1,3), tf_mul(2,3));
            // ROS_INFO("camera pose t: ");
            // std::cout<<"w: "<<tf_mul.<<std::endl;

            Eigen::Isometry3f i3f;
            std::cout<<"transpose"<<std::endl;
            std::cout<<tf_mul.transpose()<<std::endl;

            std::cout<<"###############################################"<<std::endl;

            //tranform init source cloud
            // tf_mul=tf_mul.reverse();
            pcl::transformPointCloud(*align, *tf_tgt, tf_mul);

            //accumulate aling point cloud
            acc_pc->header.frame_id = src_pc->header.frame_id;
            *acc_pc += *tf_tgt;
            
        }

        //convert pcl::PointCloud<pcl::PointXYZ> -> convert pcl::PCLPointCloud2
        pcl::toPCLPointCloud2(*src_pc, cloud_filtered2);
        pcl::toPCLPointCloud2(*align, cloud_filtered3);
        pcl::toPCLPointCloud2(*acc_pc, cloud_filtered4);
        pcl::toPCLPointCloud2(*tf_tgt, cloud_filtered5);

        //convert convert pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
        pcl_conversions::moveFromPCL(cloud_filtered, output);
        pcl_conversions::moveFromPCL(cloud_filtered2, output2);
        pcl_conversions::moveFromPCL(cloud_filtered3, output3);
        pcl_conversions::moveFromPCL(cloud_filtered4, output4);
        pcl_conversions::moveFromPCL(cloud_filtered5, output5);

        //publish
        dst_pub_.publish(output);
        src_pub_.publish(output2);
        ali_pub_.publish(output3);
        acc_pub_.publish(output4); 
        tgt_pub_.publish(output5);

        // pcl::io::savePCDFile<pcl::PointXYZ>(file_path+write_file_name,*dst_pc);
        // save src point cloud
        *save_pc = *src_pc;
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