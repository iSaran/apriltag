#ifndef _PINHOLE_CAMERA_H_
#define _PINHOLE_CAMERA_H_

#include <iostream>

#include <opencv2/opencv.hpp>


namespace cv_tools {

class PinholeCamera{
private:
    double fx_, fy_, cx_, cy_;
    cv::Size size_;

public:
    PinholeCamera(double fx, double fy, double cx, double cy, cv::Size size)
    {
        fx_ = fx;
        fy_ = fy;
        cx_ = cx;
        cy_ = cy;
        size_ = size;
    }


    cv::Mat getMatrix() const
    {
        cv::Mat cam_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
        cam_matrix.at<double>(0, 0) = fx_; cam_matrix.at<double>(1, 1) = fy_;
        cam_matrix.at<double>(0, 2) = cx_; cam_matrix.at<double>(1, 2) = cy_;
        cam_matrix.at<double>(2, 2) = 1.0;

        return cam_matrix;
    }


    cv::Mat getDistortionCoeffs() const
    {
        cv::Mat dist = cv::Mat::zeros(1, 8, CV_64F);
        return dist;
    }


    cv::Point3d backProject(cv::Point2d point, double depth) const
    {
        double z = depth / 1000.0;
        double x = (point.x - cx_) * z / fx_;
        double y = (point.y - cy_) * z / fy_;
        cv::Point3d bp(x,y,z);

        return bp;
    }


    double fx() const {return fx_;}
    double fy() const {return fy_;}
    double cx() const {return cx_;}
    double cy() const {return cy_;}
    int height() const {return size_.height;}
    int width() const {return size_.width;}
    cv::Size size() const {return size_;}
};

}

#endif
