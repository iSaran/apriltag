#ifndef APRILTAG_ROS_UTILS_H
#define APRILTAG_ROS_UTILS_H

#include <geometry_msgs/TransformStamped.h>

namespace apriltag_ros
{
geometry_msgs::TransformStamped toROS(const Eigen::Affine3d& frame,
                                      const std::string& frame_name_1,
                                      const std::string& frame_name_2);
}  // namespace apriltag_ros
#endif  // APRILTAG_ROS_UTILS_H
