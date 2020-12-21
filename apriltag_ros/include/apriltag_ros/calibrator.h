#ifndef APRILTAG_ROS_CALIBRATOR_H
#define APRILTAG_ROS_CALIBRATOR_H

#include <opencv/cv.h>
#include <Eigen/Dense>
#include <apriltag_ros/detector.h>

namespace apriltag_ros
{
class Calibrator
{
public:
    Calibrator(const cv::Size& board_size, float tag_size, float tag_border,
               const PinholeCamera& camera, double nmarkers=24);

    Calibrator(const cv::Size& board_size, float tag_size, float tag_border,
               float fx, float fy, float cx, float cy, unsigned int size_x, unsigned int size_y,
               double nmarkers=24);

    Eigen::Matrix4f doCalibration(const std::vector<Eigen::Vector3f> &robot_points,
                                  const std::vector<Eigen::Vector3f> &tag_points);

    Eigen::Affine3f run(const cv::Mat &rgb, const cv::Mat &depth,
                        const std::vector<Eigen::Vector3f> &robot_points, bool plot=false);

private:
    PinholeCamera camera;
    int nmarkers_;
    AprilTagDetector tag_detector;
};
}  // namespace apriltag_ros
#endif  // APRILTAG_ROS_CALIBRATOR_H
