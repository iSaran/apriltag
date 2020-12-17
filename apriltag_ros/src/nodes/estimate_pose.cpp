#include <apriltag_ros/detector.h>

int main()
{
  cv::Mat rgb = cv::imread("/home/marios/Desktop/test.jpg");

  cv_tools::PinholeCamera camera(525.0, 525.0, 320.0, 240.0, cv::Size(640, 480));

  std::vector<int> size(2);
  size.at(0) = 7;
  size.at(1) = 10;

  apriltag_ros::AprilTagParameters apriltagParams(size, float(0.04), float(0.01));
  apriltag_ros::AprilTagDetector tagDetector(apriltagParams);
  apriltag_ros::PoseEstimator estimator;

  std::vector<cv::Point2f> pts;
  std::vector<cv::Point3f> objs;
  if(tagDetector.findPoints(pts, objs, rgb) == false)
    std::cout << "Cannot detect tags" << std::endl;

  Eigen::Affine3d tr = estimator.estimate(pts, objs, camera);

  estimator.drawAxes(rgb, camera, pts[0]);

  return 0;
}
