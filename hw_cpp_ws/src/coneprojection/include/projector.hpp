#ifndef _PROJECTOR_HPP_
#define _PROJECTOR_HPP_

#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "tools.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "fsd_common_msgs/Cone.h"
#include "fsd_common_msgs/ConeDetections.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "tools.hpp"
#include "cv_bridge/cv_bridge.h"

class Projector
{
    ros::NodeHandle nh_;
    ros::Publisher imgpub;
    ros::Subscriber conesub;
    Eigen::Matrix<double, 3, 4> extrinsic_matrix; // 外参矩阵
    Eigen::Matrix<double, 3, 3> intrinsic_matrix; // 内参矩阵

public:
    void callback(const fsd_common_msgs::ConeDetections::ConstPtr&);
    Projector(ros::NodeHandle&);
};

#endif // _PROJECTOR_HPP_