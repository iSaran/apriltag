#ifndef _KDTREE_HPP_
#define _KDTREE_HPP_

// Eigen
#include <Eigen/Dense>

// Boost
#include <boost/filesystem.hpp>

// System
#include <iostream>
#include <vector>
#include <memory>

class Kdtree{
public:
    Kdtree(){}

    ~Kdtree(){}

    void setInput(std::vector<Eigen::Vector3d> &input_pts){}

    void knearest(int &k){}

    void withinRadius(double &radius){}
private:
};

#endif
