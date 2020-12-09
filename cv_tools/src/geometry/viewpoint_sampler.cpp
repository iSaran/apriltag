#include "cv_tools/geometry/viewpoint_sampler.hpp"


namespace cv_tools {

void ViewpointSampler::generateViews()
{
    double azimuth_step = (max_azimuth_ - min_azimuth_) / resolution_;
    double elevation_step = (max_elevation_ - min_elevation_) / resolution_;

    for(double elevation = min_elevation_; elevation < max_elevation_; elevation+=elevation_step)
    {
        for(double azimuth = min_azimuth_; azimuth < max_azimuth_; azimuth+=azimuth_step)
        {
            // calculate cartesians coordinates from spherical ones
            // (R, azimuth, elevation) -> (x, y, z)
            double x, y, z;
            x = radius_ * std::sin(elevation) * std::cos(azimuth);
            y = radius_ * std::sin(elevation) * std::sin(azimuth);
            z = radius_ * std::cos(elevation);

            // calculate the unit vectors of camera
            Eigen::Vector3d r(x, y, z);
            Eigen::Vector3d zc = -r.normalized();
            Eigen::Vector3d xc = ( zc.cross(Eigen::Vector3d(0.0, 0.0, 1.0)) ).normalized();
            Eigen::Vector3d yc = zc.cross(xc);

            if (zc.norm() == 0.0) zc.z() = 1;
            if (xc.norm() == 0.0) xc.x() = 1;
            if (yc.norm() == 0.0) yc.y() = 1;

            // set the transformation matrix
            Eigen::Matrix4d view = Eigen::Matrix4d::Identity();
            view(0, 0) = xc.x(); view(1, 0) = xc.y(); view(2, 0) = xc.z();
            view(0, 1) = yc.x(); view(1, 1) = yc.y(); view(2, 1) = yc.z();
            view(0, 2) = zc.x(); view(1, 2) = zc.y(); view(2, 2) = zc.z();
            view(0, 3) = r.x();  view(1, 3) = r.y();  view(2, 3) = r.z();

            views_.push_back(view.inverse());
        }
    }
}

}
