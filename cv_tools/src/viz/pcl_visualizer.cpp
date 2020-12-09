#include "cv_tools/viz/pcl_visulaizer.hpp"

// pcl
#include <pcl/io/vtk_lib_io.h>


namespace cv_tools {

PclVisualizer::PclVisualizer()
{
    viewer_.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_->setBackgroundColor(0.0, 0.0, 0.0);
    viewer_->addCoordinateSystem(0.15);
//    addMesh("/home/marios/catkin_ws/src/c_shape_grasp_planner/models/meshes/C-shape-gripper.stl");

    cloud_id_ = 0;
}


void PclVisualizer::addCloud(std::vector<Eigen::Vector3d>& points)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    for(int i = 0; i < points.size(); i++)
    {
        cloud->points.push_back(PointT(points[i].x(),
                                       points[i].y(),
                                       points[i].z()));
    }

    std::stringstream ss;
    ss << cloud_id_;
    cloud_id_++;

    int r, g, b;
    if(cloud_id_==1)
    {
      r = 255;
      g = 255;
      b = 255;
    }
    else if(cloud_id_==2)
    {
      r = 255;
      g = 0;
      b = 0;
    }
    else
    {
      r = 0;
      g = 255;
      b = 0;
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, r, g, b);
    viewer_->addPointCloud(cloud, rgb, ss.str());
//    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());

}


void PclVisualizer::addNormals(std::vector<Eigen::Vector3d>& points,
                               std::vector<Eigen::Vector3d>& normals)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    for(int i = 0; i < points.size(); i++)
    {
        cloud->points.push_back(PointT(points[i].x(), points[i].y(), points[i].z()));
        cloud_normals->points.push_back(pcl::Normal(normals[i].x(), normals[i].y(), normals[i].z()));
    }

    std::stringstream ss;
    ss << cloud_id_;
    cloud_id_++;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, cloud_id_*1.0, (cloud_id_-1)*1.0, 0);

    viewer_->addPointCloud(cloud, rgb, ss.str());
    viewer_->addPointCloudNormals<PointT, pcl::Normal>(cloud, cloud_normals, 1, 0.01, "normals"+ss.str());
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, cloud_id_*255, (cloud_id_-1)*255, 0, "normals"+ss.str());
}

void PclVisualizer::addFrame(Eigen::Matrix4d &frame)
{
    pcl::PointXYZ p, x, y, z;
    p.x = frame(0,3);
    p.y = frame(1,3);
    p.z = frame(2,3);

    double line_legth = 0.1;
    x.x = p.x + line_legth * frame(0,0);
    x.y = p.y + line_legth * frame(1,0);
    x.z = p.z + line_legth * frame(2,0);

    y.x = p.x + line_legth * frame(0,1);
    y.y = p.y + line_legth * frame(1,1);
    y.z = p.z + line_legth * frame(2,1);

    z.x = p.x + line_legth * frame(0,2);
    z.y = p.y + line_legth * frame(1,2);
    z.z = p.z + line_legth * frame(2,2);

    viewer_->addLine<pcl::PointXYZ>(p, x, 255, 0, 0, "x");
    viewer_->addLine<pcl::PointXYZ>(p, y, 0, 255, 0, "y");
    viewer_->addLine<pcl::PointXYZ>(p, z, 0, 0, 255, "z");

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "x");
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "y");
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "z");
}

void PclVisualizer::addMesh(const std::string &mesh_path)
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(mesh_path, mesh);
    viewer_->addPolygonMesh(mesh);
}


void PclVisualizer::addLine(Eigen::Vector3d &p1, Eigen::Vector3d &p2)
{
    pcl::PointXYZ t1, t2;
    t1.x = p1.x();
    t1.y = p1.y();
    t1.z = p1.z();

    t2.x = p2.x();
    t2.y = p2.y();
    t2.z = p2.z();

    viewer_->addLine<pcl::PointXYZ>(t1, t2, 255, 0, 0, "line");
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "line");
}


void PclVisualizer::visualize()
{
    while (!viewer_->wasStopped())
    {
        viewer_->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    std::cout << "viewer closing..." << std::endl;
}

}
