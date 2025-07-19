#include "ros/ros.h"
#include "fsd_common_msgs/ConeDetections.h"
#include "fsd_common_msgs/Cone.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ConeCounter");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
    ros::Subscriber posesub = nh.subscribe<fsd_common_msgs::CarState>("/estimation/slam/state", 10, [&](fsd_common_msgs::CarStateConstPtr msg){
        const std_msgs::Header& header = msg->header;
        const geometry_msgs::Pose2D& car_state = msg->car_state;
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(car_state.x, car_state.y, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, car_state.theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, header.stamp, header.frame_id, "rslidar"));
        ROS_INFO("Car position: x=%.2f, y=%.2f", car_state.x, car_state.y);
    });
    ros::Subscriber conesub = nh.subscribe<fsd_common_msgs::ConeDetections>("/perception/lidar/cone_side", 10, [&](fsd_common_msgs::ConeDetectionsConstPtr msg){
        int cntb = 0, cntr = 0;
        const std::vector<fsd_common_msgs::Cone>& cones = msg->cone_detections;
        const std_msgs::Header& header = msg->header;
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.id = 0;
        clear_marker.ns = "Cone";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        for(int i = 0; i < cones.size(); ++i)
        {
            const auto& cone = cones[i];
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "Cone";
            marker.id = i + 1;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = cone.position;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            // marker.lifetime = ros::Duration(1);
            if(cone.color.data == "b")
            {
                cntb++;
                marker.mesh_resource = "package://conecounter/meshes/blue.stl";
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;
            }
            else if(cone.color.data == "r")
            {
                cntr++;
                marker.mesh_resource = "package://conecounter/meshes/red.stl";
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
            }
            marker_array.markers.push_back(marker);
        }
        ROS_INFO("Header: seq=%u, frame_id=%s", header.seq, header.frame_id.c_str());
        ROS_INFO("All:%ld",cones.size());
        ROS_INFO("Blue:%d",cntb);
        ROS_INFO("Red:%d",cntr);
        marker_pub.publish(marker_array);
    });
    ros::spin();
    return 0;
}