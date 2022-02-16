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
#include <pcl/registration/gicp.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>

// PCL specific includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <string>
#include<vector>

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
    pcl::PointCloud<pcl::PointXYZ> *dst_pc = new pcl::PointCloud<pcl::PointXYZ>;
    // pcl::PointCloud<pcl::PointXYZ> *final = new pcl::PointCloud<pcl::PointXYZ>;

    Eigen::Matrix4f tf_mul;// 초기 depth frame tf or identity 넣어줘야하나?
    // std::string file_path="/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/desk/";
    
    
    int count=0;

public:
    ICPTester()
    {
        // identity
        tf_mul<< 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

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
        pcl::PCLPointCloud2 cloud_filtered5;

        pcl::PointCloud<pcl::PointXYZ>::Ptr src_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr fixed_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr align_t(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tf_tgt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr assign_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr before_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> inliers;

        
        sensor_msgs::PointCloud2 output;
        sensor_msgs::PointCloud2 output2;
        sensor_msgs::PointCloud2 output3;
        sensor_msgs::PointCloud2 output4;
        sensor_msgs::PointCloud2 output5;

        static tf::TransformBroadcaster br;
        tf::Transform transform;
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

        //convert pcl::PCLPointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
        pcl::fromPCLPointCloud2(cloud_filtered, *src_pc);

        if(count==0)
        {
            *dst_pc = *src_pc;
        }
        
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (src_pc));
        // pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model_r(new pcl::SampleConsensusModelRegistration<plc::PointXYZ>(src_pc));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        ransac.setDistanceThreshold (.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
        

        pcl::copyPointCloud<pcl::PointXYZ>(*src_pc, inliers, *final);

        // remove floor
        // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        // Create the segmentation object
        // pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        // seg.setOptimizeCoefficients (true);
        // Mandatory
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
            *before_pc = *save_pc;
            *assign_pc = *dst_pc;

            // icp
            // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp; //general icp
            // Registration 시행
            // Set the input source and target
            gicp.setInputSource(src_pc);
            gicp.setInputTarget(assign_pc);
            // gicp.setRANSACIterations(1000);
            // Set the max correspondence distance, src dst 사이의 최대거리 설정, 결과에 영향을 많이줌
            gicp.setMaxCorrespondenceDistance(1.0);
            // Set the transformation epsilon ,
            gicp.setTransformationEpsilon(1e-10);
            // Set the maximum number of iterations,  registration 될때까지 반복횟수
            gicp.setMaximumIterations(1000);
            //알고리즘이 수렴된 것으로 간주되기 전에 ICP 루프의 연속된 두 단계 사이에 허용되는 최대 유클리드 오류를 설정합니다. 
            //오차는 유클리드 의미에서 대응 간 차이의 합계를 대응 개수로 나눈 값으로 추정된다.
            gicp.setEuclideanFitnessEpsilon (1);
            gicp.setRANSACOutlierRejectionThreshold (1e-5);
            //이 방법은 대상 데이터 인덱스와 변환된 소스 인덱스 사이의 거리가 주어진 내부 거리 임계값보다 작을 경우 점을 내부 값으로 간주합니다. 이 값은 기본적으로 0.05m로 설정됩니다.
            gicp.setRANSACIterations(10);
            // Perform the alignment
            gicp.align(*align);

            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icpTransform; 
            icpTransform.setInputSource(src_pc);
            icpTransform.setInputTarget(before_pc);
            icpTransform.setMaxCorrespondenceDistance(1.0);
            icpTransform.setTransformationEpsilon(1e-10);
            icpTransform.setMaximumIterations(1000);
            icpTransform.setEuclideanFitnessEpsilon (1);
            icpTransform.setRANSACOutlierRejectionThreshold (1e-5);
            icpTransform.setRANSACIterations(10);
            icpTransform.align(*align_t);

            
            // Obtain the transformation that aligned src_pc to align
            Eigen::Matrix4f src2dst = icpTransform.getFinalTransformation();
            // Eigen::Isometry 로 바꿔서도 해보기
            
            double score = gicp.getFitnessScore();
            bool is_converged = gicp.hasConverged();

            ROS_INFO("score: %lf",score);//src dst 거리의 평균, 작을수록 icp good
            ROS_INFO("icp matrix");
            std::cout<<src2dst<<std::endl;

            //multiple tf
            tf_mul = tf_mul * src2dst;

            //tranform init source cloud
            // pcl::transformPointCloud(*align, *tf_tgt, tf_mul);
            
            //getcamera pose
            ROS_INFO("camera pose t x:%f y:%f z:%f", tf_mul(0,3), tf_mul(1,3), tf_mul(2,3));
            Eigen::Matrix4f rev;
            rev=tf_mul.inverse();
            Eigen::Matrix3f rot;
            rot=rev.block<3,3>(0,0);
            Eigen::Quaternionf q(rot);

            //send Transform
            transform.setOrigin(tf::Vector3(rev(0,3), rev(1,3), rev(2,3)));
            transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
            br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"astra_rgb_frame","astra_rgb_optical_frame"));// pframe/cframe

            //accumulate aling point cloud
            acc_pc->header.frame_id = src_pc->header.frame_id;
            // *acc_pc += *tf_tgt;
            *acc_pc += *align;
            
        }

        //convert pcl::PointCloud<pcl::PointXYZ> -> convert pcl::PCLPointCloud2
        pcl::toPCLPointCloud2(*dst_pc, cloud_filtered2);
        pcl::toPCLPointCloud2(*align, cloud_filtered3);
        pcl::toPCLPointCloud2(*acc_pc, cloud_filtered4);

        //convert convert pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
        pcl_conversions::moveFromPCL(cloud_filtered, output);
        pcl_conversions::moveFromPCL(cloud_filtered2, output2);
        pcl_conversions::moveFromPCL(cloud_filtered3, output3);
        pcl_conversions::moveFromPCL(cloud_filtered4, output4);

        //publish
        src_pub_.publish(output);
        dst_pub_.publish(output2);
        ali_pub_.publish(output3);
        acc_pub_.publish(output4); 

        // pcl::io::savePCDFile<pcl::PointXYZ>(file_path+write_file_name,*dst_pc);
        // save src point cloud
        *save_pc = *src_pc;
        count++;

        if(count==20)
        {
            pcl::io::savePCDFile<pcl::PointXYZ>("/home/jm/workspace/icp_ws/src/icp_tutorial/pcd/acc.pcd",*acc_pc);
        }

        
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