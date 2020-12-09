#ifndef _EIGEN_UTILS_HPP_
#define _EIGEN_UTILS_HPP_

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <vector>

// Ros
#include <geometry_msgs/Pose.h>

namespace cv_tools{

std::vector<Eigen::Vector3d> extractPoints(const std::vector<Eigen::Vector3d> &pts,
                                           std::vector<int> &indices);

std::vector<Eigen::Vector3d> transformPoints(std::vector<Eigen::Vector3d> &pts,
                                             Eigen::Matrix4d &pose);

void getMinMax(const std::vector<Eigen::Vector3d> &pts,
               Eigen::Vector3d &min_pt,
               Eigen::Vector3d &max_pt);

geometry_msgs::Pose poseEigenToMsg(const Eigen::Matrix4d &eigen_pose);

Eigen::Matrix4d poseMsgToEigen(const geometry_msgs::Pose &msg_pose);
}

#endif
