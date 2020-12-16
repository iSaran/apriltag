#ifndef APRILTAG_CORE_CALIBRATOR_H
#define APRILTAG_CORE_CALIBRATOR_H

#include <opencv/cv.h>
#include <Eigen/Dense>
#include <apriltag_core/detector.h>

namespace apriltag_core
{
class Calibrator
{
public:
    Calibrator(const AprilTagParameters& tag_params, double fx = 575, double fy = 575, double cx = 319.5,
               double cy = 239.5, double nmarkers=24);

    Eigen::Matrix4f doCalibration(const std::vector<Eigen::Vector3f> &robot_points,
                                  const std::vector<Eigen::Vector3f> &tag_points);

    Eigen::Affine3f run(const cv::Mat &rgb, const cv::Mat &depth,
                        const std::vector<Eigen::Vector3f> &robot_points, bool plot=false);

    Eigen::Vector3f backproject(const Eigen::Vector2f& img_point, const cv::Mat& depth);


    std::vector<Eigen::Vector2f> getTagPoints2D(const cv::Mat &rgb, bool plot=false);
    std::vector<Eigen::Vector3f> getTagPoints(const cv::Mat &rgb, const cv::Mat &depth, bool plot=false);

private:
    float fx_, fy_, cx_, cy_;
    int nmarkers_;
    AprilTagDetector tag_detector;
};
}  // namespace apriltag_core
#endif  // APRILTAG_CORE_CALIBRATOR_H
