#include "cv_tools/util/eigen_utils.hpp"

#include<float.h>
#include <eigen_conversions/eigen_msg.h>

namespace cv_tools{

std::vector<Eigen::Vector3d> extractPoints(const std::vector<Eigen::Vector3d> &pts,
                                           std::vector<int> &indices)
{
    std::vector<Eigen::Vector3d> out_pts;
    for(unsigned int i = 0; i < indices.size(); i++)
    {
        out_pts.push_back(pts[indices[i]]);
    }

    return out_pts;
}


std::vector<Eigen::Vector3d> transformPoints(std::vector<Eigen::Vector3d> &pts,
                                             Eigen::Matrix4d &pose)
{
    std::vector<Eigen::Vector3d> out_pts;
    for(unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector4d p = Eigen::Vector4d(pts[i].x(), pts[i].y(), pts[i].z(), 1.0);
        out_pts.push_back( (pose * p).head<3>());
    }

    return out_pts;
}


void getMinMax(const std::vector<Eigen::Vector3d> &pts,
               Eigen::Vector3d &min_pt,
               Eigen::Vector3d &max_pt)
{
    min_pt = Eigen::Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
    max_pt = Eigen::Vector3d(-DBL_MAX, -DBL_MAX, -DBL_MAX);

    for(unsigned int i = 0; i < pts.size(); i++)
    {
        // min_pt
        if(pts[i].x() < min_pt.x())
            min_pt.x() = pts[i].x();

        if(pts[i].y() < min_pt.y())
            min_pt.y() = pts[i].y();

        if(pts[i].z() < min_pt.z())
            min_pt.z() = pts[i].z();

        // max_pt
        if(pts[i].x() > max_pt.x())
            max_pt.x() = pts[i].x();

        if(pts[i].y() > max_pt.y())
            max_pt.y() = pts[i].y();

        if(pts[i].z() > max_pt.z())
            max_pt.z() = pts[i].z();
    }
}


geometry_msgs::Pose poseEigenToMsg(const Eigen::Matrix4d &eigen_pose)
{
  Eigen::Vector3d position = eigen_pose.block(0, 3, 3, 1);
  Eigen::Matrix3d rotation = eigen_pose.block(0, 0, 3, 3);
  Eigen::Quaterniond quat(rotation);

  if(quat.w() < 0)
  {
    quat.x() *= -1;
    quat.y() *= -1;
    quat.z() *= -1;
    quat.w() *= -1;
  }

  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = position.x();
  pose_msg.position.y = position.y();
  pose_msg.position.z = position.z();
  pose_msg.orientation.w = quat.w();
  pose_msg.orientation.x = quat.x();
  pose_msg.orientation.y = quat.y();
  pose_msg.orientation.z = quat.z();

  return pose_msg;
}


Eigen::Matrix4d poseMsgToEigen(const geometry_msgs::Pose &msg_pose)
{
  Eigen::Quaterniond quat(msg_pose.orientation.w,
                          msg_pose.orientation.x,
                          msg_pose.orientation.y,
                          msg_pose.orientation.z);
  Eigen::Matrix3d rotation = quat.normalized().toRotationMatrix();

  Eigen::Vector3d position(msg_pose.position.x,
                           msg_pose.position.y,
                           msg_pose.position.z);

  Eigen::Matrix4d eigen_pose = Eigen::Matrix4d::Identity();
  eigen_pose.block(0, 3, 3, 1) = position;
  eigen_pose.block(0, 0, 3, 3) = rotation;

  return eigen_pose;
}

//ToDo: change the function so as to take into consideration the shape


}
