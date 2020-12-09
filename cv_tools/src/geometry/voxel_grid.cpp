#include "cv_tools/geometry/voxel_grid.hpp"
#include "cv_tools/util/eigen_utils.hpp"

#include <float.h> // for DBL_MAX


namespace cv_tools {

void VoxelGrid::setPointList(cv_tools::PointList &point_list)
{
  point_list_ = point_list;

  // Initialize voxel_grid_map_
  voxel_grid_map_.clear();
  for (int x = 0; x < voxel_grid_resolution_; x++)
  {
    for (int y = 0; y < voxel_grid_resolution_; y++)
    {
      for (int z = 0; z < voxel_grid_resolution_; z++)
      {
        std::vector<int> key = {x, y, z};
        Voxel voxel;
        voxel.occupied_ = false;
        voxel.centroid_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        voxel.avg_normal_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        std::pair<std::vector<int>, Voxel> pair = std::make_pair(key, voxel);
        voxel_grid_map_.insert(pair);
      }
    }
  }

  std::vector<Eigen::Vector3d> pts = point_list_.getPoints();

  // Find voxel size
  cv_tools::getMinMax(pts, min_pt_, max_pt_);

  voxel_size_ = std::max( max_pt_.x()-min_pt_.x(), std::max(max_pt_.y()-min_pt_.y(), max_pt_.z()-min_pt_.z()) ) / (voxel_grid_resolution_ - 1);
  voxel_size_ = std::round(voxel_size_ * 10000.0) / 10000.0; // round to 4 decimal
}


PointList VoxelGrid::filter()
{
  std::vector<Eigen::Vector3d> pts = point_list_.getPoints();
  std::vector<Eigen::Vector3d> normals = point_list_.getNormals();

  int x_d = int((max_pt_.x() - min_pt_.x()) / voxel_size_);
  int y_d = int((max_pt_.y() - min_pt_.y()) / voxel_size_);
  int z_d = int((max_pt_.z() - min_pt_.z()) / voxel_size_);

  for(unsigned int i = 0; i < point_list_.size(); i++)
  {
    // Compute voxel indices
    int x = int((pts[i].x() - min_pt_.x()) / voxel_size_);
    int y = int((pts[i].y() - min_pt_.y()) / voxel_size_);
    int z = int((pts[i].z() - min_pt_.z()) / voxel_size_);

    // Point out of bounds
    if(x >= voxel_grid_resolution_ || y >= voxel_grid_resolution_  ||
       z >= voxel_grid_resolution_ )
      continue;

//    if(x >= x_d || y >= y_d || z >= z_d )
//      continue;

    // Check for Nan normals
    if(std::isnan(normals[i].norm()))
      continue;

    // Update Voxel
    std::vector<int> key = {x, y, z};
    voxel_grid_map_[key].counts_.push_back(i);
    voxel_grid_map_[key].centroid_ += pts[i];
    voxel_grid_map_[key].avg_normal_ += normals[i];

    if(voxel_grid_map_[key].occupied_)
      continue;
    else
      voxel_grid_map_[key].occupied_ = true;
  }

  std::vector<Eigen::Vector3d> voxelized_pts, voxelized_normals;

  std::map<std::vector<int>, Voxel>::iterator it;
  for(it = voxel_grid_map_.begin(); it != voxel_grid_map_.end(); ++it)
  {
    if(it->second.occupied_)
    {
      std::vector<int> key = it->first;
      it->second.centroid_ /= it->second.counts_.size();
      it->second.avg_normal_ /= it->second.avg_normal_.norm();

      voxelized_pts.push_back(it->second.centroid_);
      voxelized_normals.push_back(it->second.avg_normal_);
      occupied_voxels_.push_back(Eigen::Vector3i(key[0], key[1], key[2]));
    }
  }

  return PointList(voxelized_pts, voxelized_normals);
}

}
