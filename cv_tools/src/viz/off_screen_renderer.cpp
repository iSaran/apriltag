#include "cv_tools/viz/off_screen_renderer.hpp"

// pcl
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

// vtk
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkVersion.h>
#include <vtkProperty.h>


namespace cv_tools{

OffScreenRenderer::OffScreenRenderer(const PinholeCamera& camera, const double z_near,
                                     const double z_far):
    cam_(camera), z_near_(z_near), z_far_(z_far)
{
    vtk_camera_ = vtkSmartPointer<vtkCamera>::New();
    render_win_ = vtkSmartPointer<vtkRenderWindow>::New();

    // set virtual camera parameters
    vtk_camera_->SetClippingRange(z_near_, z_far_);

    // camera view angle
    double fovy = 2 * atan((cam_.height()/2) / cam_.fy());
    vtk_camera_->SetViewAngle(fovy * 180/M_PI);

    // default transformation for xtion: reverse the y,z axes
    vtkSmartPointer<vtkMatrix4x4> xtion_to_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
    xtion_to_vtk->Identity();
    xtion_to_vtk->SetElement(1, 1, -1);
    xtion_to_vtk->SetElement(2, 2, -1);

    vtkSmartPointer<vtkTransform> rot_axes = vtkSmartPointer<vtkTransform>::New();
    rot_axes->Concatenate(xtion_to_vtk);
    rot_axes->Concatenate(vtk_camera_->GetViewTransformMatrix());
    vtk_camera_->ApplyTransform(rot_axes);
}


OffScreenRenderer::~OffScreenRenderer()
{
    poly_data_ = NULL;
}


void OffScreenRenderer::addMesh(boost::filesystem::path& model_path)
{
    pcl::PolygonMesh mesh;
//    pcl::io::loadPolygonFilePLY(model_path.string(), mesh);
    pcl::io::loadPolygonFileSTL(model_path.string(), mesh);

    poly_data_ = vtkPolyData::New();
    pcl::VTKUtils::mesh2vtk(mesh, poly_data_);
}


void OffScreenRenderer::render(Eigen::Matrix4d& pose)
{
    // model-to-camera
    vtkSmartPointer<vtkMatrix4x4> vtk_pose = vtkSmartPointer<vtkMatrix4x4>::New();
    for(int i = 0;i < 4;i++)
        for(int j = 0;j < 4;j++)
            vtk_pose->SetElement(i, j, pose(i,j));

    // transform the virtual camera
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Identity();
    transform->SetMatrix(vtk_pose);
    transform->Inverse();
    vtk_camera_->ApplyTransform(transform);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
    mapper->SetInput(poly_data_);
#else
    mapper->SetInputData(poly_data_);
#endif
    mapper->Update();

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    render_win_->SetSize(cam_.width(), cam_.height());

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetActiveCamera(vtk_camera_);

    render_win_->AddRenderer(renderer);
    render_win_->Render();
}


void OffScreenRenderer::transformCamera(Eigen::Matrix4d& pose)
{
    // model-to-camera
    vtkSmartPointer<vtkMatrix4x4> vtk_pose = vtkSmartPointer<vtkMatrix4x4>::New();
    for(int i = 0;i < 4;i++)
        for(int j = 0;j < 4;j++)
            vtk_pose->SetElement(i, j, pose(i,j));

    // transform the virtual camera
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Identity();
    transform->SetMatrix(vtk_pose);
    transform->Inverse();
    vtk_camera_->ApplyTransform(transform);
}


cv::Mat OffScreenRenderer::getRgb() const
{
    cv::Mat rgb = cv::Mat::zeros(cam_.height(), cam_.width(), CV_64FC3);

    vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    window_to_image_filter->SetInput(render_win_);
    window_to_image_filter->SetMagnification(1);
    window_to_image_filter->SetInputBufferTypeToRGB();

    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName("screenshot.png");
    writer->SetInputConnection(window_to_image_filter->GetOutputPort());
    writer->Write();
    rgb = cv::imread("screenshot.png", CV_LOAD_IMAGE_COLOR);

    return rgb;
}


cv::Mat OffScreenRenderer::getDepth() const
{
    cv::Mat depth = cv::Mat::zeros(cam_.height(), cam_.width(), CV_16UC1);

    // save depth mat
    float* z_buffer = new float[cam_.width() * cam_.height()]();
    render_win_->GetZbufferData(0, 0, cam_.width() - 1, cam_.height() - 1, z_buffer);

    for(int x = 0; x < cam_.width(); x++)
    {
        for(int y = 0; y < cam_.height(); y++)
        {
            float z = z_buffer[y * cam_.width() + x];
            if(z != 1.0f)
            {
                double depth_value = -1 * (z_far_ * z_near_) / (z * (z_far_ - z_near_) - z_far_);
                depth.at<int16_t>(y, x) = (int16_t) (depth_value * 1000.0f);
            }
        }
    }
    cv::flip(depth, depth, 0);

    delete[] z_buffer;

    return depth;
}

}
