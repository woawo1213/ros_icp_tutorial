#include<ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"b_node");
    ros::NodeHandle nh;
    ros::Rate r(0.5);
    int num=0;

    while(ros::ok())
    {
        ROS_INFO("hello %d",num);
        num++;
        r.sleep();
    }
    return 0;
}