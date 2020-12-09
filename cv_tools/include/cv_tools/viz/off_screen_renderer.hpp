#ifndef _OFF__SCREEN_RENDERER_HPP_
#define _OFF__SCREEN_RENDERER_HPP_

// vtk
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>

// opencv
#include "opencv2/highgui/highgui.hpp"

// boost
#include "boost/filesystem.hpp"

// eigen
#include <Eigen/Geometry>

#include "cv_tools/geometry/pinhole_camera.hpp"

namespace cv_tools{

class OffScreenRenderer{
public:
    OffScreenRenderer(const PinholeCamera &camera, const double z_near = 0.001,
                      const double z_far = 100);

    ~OffScreenRenderer();

    void addMesh(boost::filesystem::path& model_path);

    void render(Eigen::Matrix4d& pose);

    void transformCamera(Eigen::Matrix4d& pose);

    void setClippingRange(double &z_near, double &z_far)
    {
        z_near_ = z_near;
        z_far_ = z_far;
    }

    cv::Mat getRgb() const;

    cv::Mat getDepth() const;

private:
    PinholeCamera cam_;
    vtkSmartPointer<vtkCamera> vtk_camera_;
    vtkSmartPointer<vtkPolyData>  poly_data_;
    vtkSmartPointer<vtkRenderWindow> render_win_;

    double z_near_, z_far_; //!< near and far clipping plane
};
}
#endif
