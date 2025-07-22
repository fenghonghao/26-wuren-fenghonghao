#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl-1.10/pcl/point_cloud.h"
#include "pcl-1.10/pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
using namespace message_filters;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> mySyncPolicy;
typedef Synchronizer<mySyncPolicy> mySynchronizer;
boost::shared_ptr<mySynchronizer> synchronizer;
ros::Publisher pcl_publisher;

void callback(const sensor_msgs::PointCloud2ConstPtr& pclup_msg,
              const sensor_msgs::PointCloud2ConstPtr& pclmid_msg,
              const sensor_msgs::PointCloud2ConstPtr& pcldown_msg)
{
    pcl::PointCloud<pcl::PointXYZ> lidar_up, lidar_mid, lidar_down, lidar_all;
    pcl::fromROSMsg(*pclup_msg, lidar_up);
    pcl::fromROSMsg(*pclmid_msg, lidar_mid);
    pcl::fromROSMsg(*pcldown_msg, lidar_down);

    ROS_INFO("Timestamps: up %f, mid %f, down %f", 
             pclup_msg->header.stamp.toSec(), 
             pclmid_msg->header.stamp.toSec(), 
             pcldown_msg->header.stamp.toSec());

    lidar_all += lidar_up;
    lidar_all += lidar_mid;
    lidar_all += lidar_down;

    ROS_INFO("Received point clouds: up %zu points, mid %zu points, down %zu points, all %zu points",
             lidar_up.size(), lidar_mid.size(), lidar_down.size(), lidar_all.size());

    sensor_msgs::PointCloud2 pcl_all_msg;
    pcl::toROSMsg(lidar_all, pcl_all_msg);
    pcl_all_msg.header.frame_id = "lidar";
    pcl_publisher.publish(pcl_all_msg);
    ROS_INFO("msg published, total points: %zu", pcl_all_msg.width * pcl_all_msg.height);
}

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "visualisepcl");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pclup(nh, "/carla/ego_vehicle/lidar_up", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pclmid(nh, "/carla/ego_vehicle/lidar_mid", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcldown(nh, "/carla/ego_vehicle/lidar_down", 10);
    pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/lidar_pointss", 10);
    synchronizer = boost::shared_ptr<mySynchronizer>(new mySynchronizer(mySyncPolicy(10), pclup, pclmid, pcldown));
    synchronizer->registerCallback(boost::bind(&callback, _1, _2, _3));
    ros::spin();
    return 0;
}