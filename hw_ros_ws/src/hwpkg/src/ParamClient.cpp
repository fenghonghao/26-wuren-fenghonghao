#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include <map>
int main(int argc, char *argv[])
{

    // 1.初始化 ROS 节点
    ros::init(argc, argv, "TurtleParamClient");

    // 2.创建 ROS 句柄
    ros::NodeHandle nh;

    // 3.创建发布者对象
    ros::Publisher Twistpub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    // 4.组织被发布的消息，编写发布逻辑并发布消息
    std::map<std::string, int> color;
    nh.getParam("/myturtle/Color", color); // 可以传map进去获取参数
    ROS_INFO("Map Size: %lld", color.size());
    for (const auto &pair : color)
    {
        ROS_INFO("Key:%s, Value:%d", pair.first.c_str(), pair.second);
    }
    nh.setParam("/turtlesim", color);
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    clearClient.call(srv);

    geometry_msgs::Twist twist;
    nh.getParam("/myturtle/Twist/linear/x", twist.linear.x);
    nh.getParam("/myturtle/Twist/linear/y", twist.linear.y);
    nh.getParam("/myturtle/Twist/linear/z", twist.linear.z);
    nh.getParam("/myturtle/Twist/angular/x", twist.angular.x);
    nh.getParam("/myturtle/Twist/angular/y", twist.angular.y);
    nh.getParam("/myturtle/Twist/angular/z", twist.angular.z);

    // 5.设置循环频率
    ros::Rate rate(1);

    // 6.发布消息
    while (ros::ok())
    {
        Twistpub.publish(twist);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}