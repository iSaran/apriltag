#include "cv_tools/geometry/point_list.hpp"
#include "cv_tools/util/eigen_utils.hpp"
#include "cv_tools/viz/pcl_visulaizer.hpp"


namespace cv_tools {

PointList PointList::slice(std::vector<int> &indices)
{
    std::vector<Eigen::Vector3d> out_pts = extractPoints(points_, indices);
    std::vector<Eigen::Vector3d> out_normals = extractPoints(normals_, indices);

    return PointList(out_pts, out_normals);
}


PointList PointList::transformPointList(Eigen::Matrix4d &pose)
{
    std::vector<Eigen::Vector3d> out_pts = transformPoints(points_, pose);

    Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
    rotation.block(0, 0, 3, 3) = pose.block(0, 0, 3, 3);
    std::vector<Eigen::Vector3d> out_normals = transformPoints(normals_, rotation);
    return PointList(out_pts, out_normals);
}

void PointList::plot()
{
    std::shared_ptr<cv_tools::PclVisualizer> viz(new cv_tools::PclVisualizer());
    viz->addNormals(points_, normals_);
    viz->visualize();
}


PointList concatenate(PointList &a, PointList &b)
{
  std::vector<Eigen::Vector3d> pts_b = b.getPoints();
  std::vector<Eigen::Vector3d> normals_b = b.getNormals();

  std::vector<Eigen::Vector3d> pts_a = a.getPoints();
  std::vector<Eigen::Vector3d> normals_a = a.getNormals();

  pts_a.insert(pts_a.end(), pts_b.begin(), pts_b.end() );
  normals_a.insert(normals_a.end(), normals_b.begin(), normals_b.end());

  return PointList(pts_a, normals_a);
}

}
