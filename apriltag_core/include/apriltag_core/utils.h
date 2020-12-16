#ifndef APRILTAG_CORE_UTILS_H
#define APRILTAG_CORE_UTILS_H

#include <geometry_msgs/TransformStamped.h>

namespace apriltag_core
{
geometry_msgs::TransformStamped toROS(const Eigen::Affine3d& frame,
                                      const std::string& frame_name_1,
                                      const std::string& frame_name_2);
}  // namespace apriltag_core
#endif  // APRILTAG_CORE_UTILS_H
