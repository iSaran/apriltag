#include <apriltag_ros/utils.h>

namespace apriltag_ros
{
geometry_msgs::TransformStamped toROS(const Eigen::Affine3d& frame,
                                      const std::string& frame_name_1,
                                      const std::string& frame_name_2)
{
  geometry_msgs::TransformStamped transformStamped;
  Eigen::Quaterniond quat(frame.linear().matrix());
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = frame_name_1;
  transformStamped.child_frame_id = frame_name_2;
  transformStamped.transform.translation.x = frame.translation()(0);
  transformStamped.transform.translation.y = frame.translation()(1);
  transformStamped.transform.translation.z = frame.translation()(2);
  transformStamped.transform.rotation.x = quat.x();
  transformStamped.transform.rotation.y = quat.y();
  transformStamped.transform.rotation.z = quat.z();
  transformStamped.transform.rotation.w = quat.w();
  return transformStamped;

}

sensor_msgs::ImagePtr toROS(const cv::Mat& img, const std_msgs::Header& header, const std::string& encoding)
{
  cv_bridge::CvImage out_msg;
  cv::Mat bgr;
  cv::cvtColor(img, bgr, CV_RGB2BGR);
  out_msg.encoding = encoding;
  out_msg.image    = bgr;
  out_msg.header   = header;
  return out_msg.toImageMsg();
}

}  // namespace apriltag_detector