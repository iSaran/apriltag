#ifndef _VIEWPOINT_SAMPLER_HPP_
#define _VIEWPOINT_SAMPLER_HPP_


#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace cv_tools{

class ViewpointSampler{
public:
    ViewpointSampler()
    {
        radius_ = 1.0;
        min_azimuth_ = 0;
        max_azimuth_ = 2 * M_PI;
        min_elevation_ = M_PI / 6;
        max_elevation_ = M_PI / 2;
        resolution_ = 5;
    }

    ~ViewpointSampler(){}

    void generateViews();

    void setRadius(double radius)
    {
        radius_ = radius;
    }

    void setAzimuths(double min, double max)
    {
        min_azimuth_ = min;
        max_azimuth_ = max;
    }

    void setElevations(double min, double max)
    {
        min_elevation_ = min;
        max_elevation_ = max;
    }

    void setResolution(int resolution)
    {
        resolution_ = resolution;
    }

    void getViews(std::vector<Eigen::Matrix4d> &views)
    {
        views = views_;
    }

private:
    std::vector<Eigen::Matrix4d> views_;

    double radius_;         //!< sphere radius in meters
    double min_azimuth_;    //!< minimum azimuth in rads
    double max_azimuth_;    //!< maximum azimuth in rads
    double min_elevation_;  //!< minimum elevation in rads
    double max_elevation_;  //!< maximum elevation in rads

    int resolution_;        //!< number of azimuths and elevations
};

}
#endif
