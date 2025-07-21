#include "projector.hpp"

Projector::Projector(ros::NodeHandle& nh) : nh_(nh)
{
    imgpub = nh_.advertise<sensor_msgs::Image>("/cone2img", 10);
    conesub = nh_.subscribe<fsd_common_msgs::ConeDetections>("/perception/fusion/cone_fusion", 10, &Projector::callback, this);

    intrinsic_matrix << 532.795, 0.0, 632.15,
                        0.0, 532.72, -3.428,
                        0.0, 0.0, 1.0 ;
    extrinsic_matrix << 3.5594209875121074e-03, -9.9987761481865733e-01,
                        -1.5234365979146680e-02, 8.9277270417879417e-02,
                        1.9781062410225703e-03, 1.5241472820252011e-02,
                        -9.9988188532544631e-01, 9.1100499695349946e-01,
                        9.9999170877459420e-01, 3.5288653732390984e-03,
                        2.0321149683686368e-03, 1.9154049062915668e+00;
}

void Projector::callback(const fsd_common_msgs::ConeDetections::ConstPtr& msg)
{
    std_msgs::Header header = msg->header;
    std::vector<fsd_common_msgs::Cone> cones = msg->cone_detections;
    cv::Mat img(360, 1280, CV_8UC3, cv::Scalar(255, 255, 255));
    int cntb = 0, cntr = 0;
    for(auto cone : cones)
    {
        Eigen::Vector3d cone_coordinates_in_world(cone.position.x, cone.position.y, cone.position.z);
        Eigen::Vector3d cone_coordinates_in_pixel = intrinsic_matrix * (extrinsic_matrix * cone_coordinates_in_world.homogeneous());
        cone_coordinates_in_pixel /= cone_coordinates_in_pixel(2); // 归一化
        cv::Scalar color;
        if (cone.color.data == "r")
        {
            color = cv::Scalar(0, 0, 255);
            cntr++;
        }
        else if (cone.color.data == "b")
        {
            color = cv::Scalar(255, 0, 0);
            cntb++;
        }
        // ROS_INFO("x_coordinate%f, y_coordinate%f", cone_coordinates_in_pixel(0), cone_coordinates_in_pixel(1));
        tools::circle(img, cv::Point(cone_coordinates_in_pixel(0), cone_coordinates_in_pixel(1)), 10, color, cv::FILLED);
    }
    ROS_INFO("All:%ld; Red:%d; Blue:%d", cones.size(), cntr, cntb);
    sensor_msgs::ImageConstPtr ros_img = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    imgpub.publish(ros_img);
}