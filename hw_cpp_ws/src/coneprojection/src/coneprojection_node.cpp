#include "ros/ros.h"
#include "tools.hpp"
#include "projector.hpp"
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "coneprojection_node");
    ros::NodeHandle nh;
    Projector projector(nh);
    ros::spin();
    return 0;
}