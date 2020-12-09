#ifndef _POINT_LIST_HPP_
#define _POINT_LIST_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <vector>


namespace cv_tools{
/**
 * The PointList class
 * @brief This class represents a list of points by storing the points and their surface normals
 */
class PointList{
public:
    PointList(){}

    PointList(std::vector<Eigen::Vector3d> &points,
              std::vector<Eigen::Vector3d> &normals):
        points_(points), normals_(normals){}

    ~PointList()
    {
        points_.clear();
        normals_.clear();
    }

    PointList slice(std::vector<int> &indices);

    PointList transformPointList(Eigen::Matrix4d &pose);

    void clear()
    {
        points_.clear();
        normals_.clear();
    }

    void plot();

    std::vector<Eigen::Vector3d> getPoints() const{ return points_; }

    std::vector<Eigen::Vector3d> getNormals() const{ return normals_; }

    unsigned int size() const { return points_.size(); }

private:
    std::vector<Eigen::Vector3d> points_;
    std::vector<Eigen::Vector3d> normals_;
};


PointList concatenate(PointList &a, PointList &b);

}

#endif
