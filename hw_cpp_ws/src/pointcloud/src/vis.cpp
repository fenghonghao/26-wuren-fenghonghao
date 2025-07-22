#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl-1.10/pcl/point_cloud.h"
#include "pcl-1.10/pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
pcl::PointCloud<pcl::PointXYZ> lidar_up, lidar_mid, lidar_down, lidar_all;
int main(int argc,char* argv[])
{
    ros::init(argc, argv, "visualisepcl");
    ros::NodeHandle nh;
    // 订阅全部三个点云话题
    ros::Subscriber pclup_sub = nh.subscribe<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar_up", 10, 
        [&](const sensor_msgs::PointCloud2ConstPtr& msg) {
            pcl::fromROSMsg(*msg, lidar_up);
            ROS_INFO("Received lidar_up with %zu points", lidar_up.size());
        });
    ros::Subscriber pclmid_sub = nh.subscribe<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar_mid", 10, 
        [&](const sensor_msgs::PointCloud2ConstPtr& msg) {            pcl::fromROSMsg(*msg, lidar_mid);
            ROS_INFO("Received lidar_mid with %zu points", lidar_mid.size());
        });
    ros::Subscriber pcldown_sub = nh.subscribe<sensor_msgs::PointCloud2>("/carla/ego_vehicle/lidar_down", 10, 
        [&](const sensor_msgs::PointCloud2ConstPtr& msg) {
            pcl::fromROSMsg(*msg, lidar_down);
            ROS_INFO("Received lidar_down with %zu points", lidar_down.size());
        });
    ros::Publisher pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/lidar_points", 10);

    // 大概0.05s有一贞
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        lidar_all.clear();
        lidar_all += lidar_up;
        lidar_all += lidar_mid;
        lidar_all += lidar_down;

        ROS_INFO("Received point clouds: up %zu points, mid %zu points, down %zu points, all %zu points",
                 lidar_up.size(), lidar_mid.size(), lidar_down.size(), lidar_all.size());

        sensor_msgs::PointCloud2 pcl_all_msg;
        pcl::toROSMsg(lidar_all, pcl_all_msg); // 将点云转换为ROS的Pointloud2格式
        pcl_all_msg.header.frame_id = "lidar";
        pcl_publisher.publish(pcl_all_msg);
        ROS_INFO("msg published, total points: %u", pcl_all_msg.width * pcl_all_msg.height);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}