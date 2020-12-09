#include "apriltag_detector/apriltag_detector.hpp"

int main()
{
  cv::Mat rgb = cv::imread("/home/marios/Desktop/test.jpg");

  PinholeCamera camera(525.0, 525.0, 320.0, 240.0, cv::Size(640, 480));

  AprilTagDetector::AprilTagParameters apriltagParams;
  AprilTagDetector tagDetector;
  PoseEstimator estimator;

  std::vector<cv::Point2f> pts;
  std::vector<cv::Point3f> objs;
  if(tagDetector.findPoints(pts, objs, rgb, apriltagParams) == false)
    std::cout << "Cannot detect tags" << std::endl;

  Eigen::Affine3d tr = estimator.estimate(pts, objs, camera);

  estimator.drawAxes(rgb, camera, pts[0]);

  return 0;
}
