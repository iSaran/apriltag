#include "cv_tools/geometry/normal_estimator.hpp"

// pcl
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>


// ToDo: Calculate surface normals using integral images if possible

namespace cv_tools {

NormalEstimator::NormalEstimator()
{
    radius_ = 0.01f;
}


void NormalEstimator::setPoints(std::vector<Eigen::Vector3d> pts)
{
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for(unsigned int i = 0; i < pts.size(); i++)
    {
        cloud_->points.push_back(pcl::PointXYZ(pts[i].x(),
                                               pts[i].y(),
                                               pts[i].z()));
    }
}

void NormalEstimator::compute(bool reverse)
{
    normals_.clear();

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius_);
    ne.compute(*cloud_normals);

    for(unsigned int i = 0; i < cloud_normals->points.size(); i++)
    {
        Eigen::Vector3d n(cloud_normals->points[i].normal_x,
                          cloud_normals->points[i].normal_y,
                          cloud_normals->points[i].normal_z);

        if(reverse)
            normals_.push_back(-n);
        else
            normals_.push_back(n);
    }
}

}
