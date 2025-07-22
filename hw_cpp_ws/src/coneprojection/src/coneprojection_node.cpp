#include "ros/ros.h"
#include "projector.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "coneprojection_node");
    ros::NodeHandle nh;
    // 传入nh，初始化Projector类
    Projector projector(nh);
    ros::spin();
    return 0;
}