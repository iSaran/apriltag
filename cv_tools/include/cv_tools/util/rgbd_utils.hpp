#ifndef _UTIL_H_
#define _UTIL_H_

// pcl
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

// OpenCV
#include "opencv2/highgui/highgui.hpp"

// cv_tools
#include "cv_tools/geometry/pinhole_camera.hpp"
#include "cv_tools/geometry/point_list.hpp"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace cv_tools {

PointCloudT::Ptr depthToPointCloud(const cv::Mat &depth, const PinholeCamera &camera);

std::vector<Eigen::Vector3d> depthToPoints(const cv::Mat &depth, const PinholeCamera &camera);

PointCloudT::Ptr pointListToPointCloud(PointList &point_list);

std::vector<Eigen::Vector3d> cloudToPoints(PointCloudT::Ptr cloud);

PointCloudT::Ptr pointsToPointCloud(std::vector<Eigen::Vector3d> &pts);
}

#endif
