#ifndef _PCL_VISUALIZER_HPP_
#define _PCL_VISUALIZER_HPP_

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace cv_tools {

class PclVisualizer{
public:
    PclVisualizer();
    ~PclVisualizer(){}

    void addCloud(std::vector<Eigen::Vector3d>& points);

    void addNormals(std::vector<Eigen::Vector3d>& points,
                    std::vector<Eigen::Vector3d>& normals);

    void addFrame(Eigen::Matrix4d &frame);

    void addMesh(const std::string &mesh_path);

    void addLine(Eigen::Vector3d &p1, Eigen::Vector3d &p2);

    void visualize();
private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    int cloud_id_;
};

}

#endif
