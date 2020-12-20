#ifndef APRILTAG_ROS_DETECTOR_H
#define APRILTAG_ROS_DETECTOR_H


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


namespace apriltag_ros
{
struct AprilTagParameters
{
    cv::Size board_size_;
    float tag_size_;
    float tag_border_;

    AprilTagParameters(const std::vector<int>& size, float tag_size, float tag_border);
};

/**
 * \brief Wrapper class of Apriltag for detecting a tag
 * 
 * \param params The apriltag parameters
 * \param refine_pose Parameter for better accuracy with the cost of
 * performance. Set to 0 for fast detection with low accuracy and to 1 for slow
 * detection with high accuracy
 */
class AprilTagDetector
{
public:

    struct Result
    {
        uint64_t id;
        cv::Point2f pts[4];
    };

    // AprilTagDetector();

    AprilTagDetector(const AprilTagParameters& params, int refine_pose = 0);

    void detect(const cv::Mat &img, std::vector<AprilTagDetector::Result> &results);

    bool findPoints(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                    const cv::Mat &img);

    void drawPoints(cv::Mat &img, std::vector<cv::Point2f> &pts);

private:
    apriltag_detector* td_;
    AprilTagParameters params;
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
                             const cv_tools::PinholeCamera &camera);

    void drawAxes(cv::Mat &img, cv_tools::PinholeCamera &camera, cv::Point2f &corner);

private:
    cv::Mat r_vec_, t_vec_;

};
}


#endif  // APRILTAG_ROS_DETECTOR_H
