#ifndef NORMAL_ESTIMATOR_HPP_
#define NORMAL_ESTIMATOR_HPP_


#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace cv_tools {

class NormalEstimator{
public:
    NormalEstimator();

    ~NormalEstimator(){}

    void setPoints(std::vector<Eigen::Vector3d> pts);

    void setRadius(float radius)
    {
        radius_ = radius;
    }

    void compute(bool reverse);

    std::vector<Eigen::Vector3d> normals() const { return normals_; }

private:
    std::vector<Eigen::Vector3d> normals_;
    float radius_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

}

#endif
