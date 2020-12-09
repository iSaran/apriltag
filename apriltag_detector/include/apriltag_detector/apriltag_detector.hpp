#ifndef _APRILTAG_DETECTOR_
#define _APRILTAG_DETECTOR_


// system
#include <iostream>
#include <vector>

// opencv
#include <opencv2/opencv.hpp>

// eigen
#include <eigen3/Eigen/Geometry>

// apriltag
#include <apriltag.h>
#include <tag36h11.h>

// cv_tools
#include "cv_tools/geometry/pinhole_camera.hpp"


using namespace cv_tools;


class AprilTagDetector{
public:
    struct AprilTagParameters
    {
        cv::Size board_size_;
        float tag_size_;
        float tag_border_;

        AprilTagParameters()
        {
            board_size_ = cv::Size(7, 10);
            tag_size_ = 0.04;
            tag_border_ = 0.01;
        }
    };

    struct Result
    {
        uint64_t id;
        cv::Point2f pts[4];
    };

    AprilTagDetector();

    void detect(const cv::Mat &img, std::vector<AprilTagDetector::Result> &results);

    bool findPoints(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                    cv::Mat &img, const AprilTagParameters &params);

    void drawPoints(cv::Mat &img, std::vector<cv::Point2f> &pts);

private:
    apriltag_detector* td_;
};


class ChessBoardDetector{
public:
    struct ChessBoardParamaters
    {
        cv::Size board_size_;
        double square_size_;

        ChessBoardParamaters()
        {
            board_size_ = cv::Size(9, 6);
            square_size_ = 0.025;   //2.5cm
        }
    };

    ChessBoardDetector(){}

    bool findPoints(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                    const cv::Mat &img, const ChessBoardParamaters &params);
};



class PoseEstimator{

public:
    PoseEstimator(){}

    Eigen::Affine3d estimate(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                             const PinholeCamera &camera);

    void drawAxes(cv::Mat &img, PinholeCamera &camera, cv::Point2f &corner);

private:
    cv::Mat r_vec_, t_vec_;

};


#endif
