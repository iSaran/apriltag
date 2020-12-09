#include "cv_tools/util/rgbd_utils.hpp"


namespace cv_tools {

PointCloudT::Ptr depthToPointCloud(const cv::Mat& depth, const PinholeCamera &camera)

{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    unsigned short depth_value;
    double x, y, z;
    for(int i = 0; i < depth.rows; i++)
    {
        for(int j = 0; j < depth.cols; j++)
        {
            depth_value = depth.at<unsigned short>(i,j);
            if(depth_value != 0)
            {
                z = double(depth_value) / 1000.0;
                x = (j - camera.cx()) * z / camera.fx();
                y = (i - camera.cy()) * z / camera.fy();
                cloud->points.push_back(pcl::PointXYZ(x, y, z));
            }
        }
    }

    return cloud;
}


std::vector<Eigen::Vector3d> depthToPoints(const cv::Mat& depth, const PinholeCamera &camera)

{
    std::vector<Eigen::Vector3d> points;

    unsigned short depth_value;
    double x, y, z;
    for(int i = 0; i < depth.rows; i++)
    {
        for(int j = 0; j < depth.cols; j++)
        {
            depth_value = depth.at<unsigned short>(i,j);
            if(depth_value != 0)
            {
                z = double(depth_value) / 1000.0;
                x = (j - camera.cx()) * z / camera.fx();
                y = (i - camera.cy()) * z / camera.fy();
                points.push_back(Eigen::Vector3d(x, y, z));
            }
        }
    }

    return points;
}


PointCloudT::Ptr pointListToPointCloud(PointList &point_list)
{
    std::vector<Eigen::Vector3d> pts = point_list.getPoints();

    PointCloudT::Ptr cloud(new PointCloudT);
    for(unsigned int i = 0; i < point_list.size(); i++)
    {
        cloud->points.push_back(PointT(pts[i].x(), pts[i].y(), pts[i].z()));
    }
    return  cloud;
}


std::vector<Eigen::Vector3d> cloudToPoints(PointCloudT::Ptr cloud)
{
  std::vector<Eigen::Vector3d> pts;
  for(unsigned int i = 0; i < cloud->points.size(); i++)
  {
    Eigen::Vector3d p(cloud->points[i].x,
                      cloud->points[i].y,
                      cloud->points[i].z);
    pts.push_back(p);
  }
  return pts;
}


PointCloudT::Ptr pointsToPointCloud(std::vector<Eigen::Vector3d> &pts)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    for(unsigned int i = 0; i < pts.size(); i++)
    {
        cloud->points.push_back(PointT(pts[i].x(), pts[i].y(), pts[i].z()));
    }
    return  cloud;
}

}

