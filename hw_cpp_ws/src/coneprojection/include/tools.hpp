#ifndef _TOOLS_HPP_
#define _TOOLS_HPP_

#include "opencv2/opencv.hpp"
namespace tools
{
    // 画线
    inline void line(
        cv::Mat& img, const cv::Point& st, const cv::Point ed,
        const cv::Scalar& color = cv::Scalar(0, 0, 255),
        const int& thickness = 4, const int& lineType = 8, const int& shift = 0)
    {
        cv::line(img, st, ed, color, thickness, lineType, shift);
        // shift 为左移位数, 意味着输入点的坐标(整数)会 * 2^(-shift)
    }

    // 画圆
    inline void circle(cv::Mat& img, const cv::Point point, const int& radius,
        const cv::Scalar color = cv::Scalar(0, 0, 255),
        const int& thickness = 4, const int& lineType = 8, const int& shift = 0)
    {
        cv::circle(img, point, radius, color, thickness, lineType, shift);
    }

    // 画框, 传矩形
    inline void rectangle(cv::Mat &img, const cv::Rect& rect,
        const cv::Scalar color = cv::Scalar(0, 0, 255),
        const int& thickness = 4, const int& lineType = 8, const int& shift = 0)
    {
        cv::rectangle(img, rect, color, thickness, lineType, shift);
    }
    
    // 画框, 传左上角、宽、高
    inline void rectangle(cv::Mat& img, const cv::Point& point, const int& width, const int& height,
        const cv::Scalar color = cv::Scalar(0, 0, 255),
        const int& thickness = 4, const int& lineType = 8, const int& shift = 0)
    {
        cv::rectangle(img, cv::Rect(point.x, point.y, width, height), color, thickness, lineType, shift);
    }
    
    // 画框, 传左上角、右下角
    inline void rectangle(cv::Mat& img, const cv::Point& pt1, const cv::Point& pt2,
        const cv::Scalar color = cv::Scalar(0, 0, 255),
        const int& thickness = 4, const int& lineType = 8, const int& shift = 0)
    {
        cv::rectangle(img, pt1, pt2, color, thickness, lineType, shift);
    }

    // 画框, 传中心点、宽、高
    inline void rectanglemd(cv::Mat& img, const cv::Point& point, const int& width, const int& height,
        const cv::Scalar color = cv::Scalar(0, 0, 255),
        const int& thickness = 4, const int& lineType = 8, const int& shift = 0)
    {
        cv::rectangle(img, cv::Rect(point, cv::Size(width, height)), color, thickness, lineType, shift);
    }

    // 画点, 传点集
    inline void darw_points(cv::Mat& img, const std::vector<cv::Point>& points,
        const cv::Scalar& color = cv::Scalar(0, 0, 255), const int& thickness = 2)
    {
        std::vector<std::vector<cv::Point>> contours;
        contours.emplace_back(points);
        cv::drawContours(img, contours, -1, color, thickness);
        // drawContours:把轮廓用线连起来
    }

    // 画点, 传点集
    inline void darw_points(cv::Mat& img, const std::vector<cv::Point2f> & points,
        const cv::Scalar& color = cv::Scalar(0, 0, 255), const int& thickness = 2)
    {
        std::vector<cv::Point> int_points(points.begin(), points.end());
        darw_points(img, int_points, color, thickness);
    }
}

#endif // _TOOLS_HPP_