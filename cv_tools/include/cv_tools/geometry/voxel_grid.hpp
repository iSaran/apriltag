#ifndef VOXEL_GRID_HPP_
#define VOXEL_GRID_HPP_

// Eigen
#include <Eigen/Dense>

// System
#include <iostream>
#include <vector>
#include <map>

// cv_tools
#include "cv_tools/geometry/point_list.hpp"


namespace cv_tools{

/**
 * @brief The Voxel struct
 *
 * In each voxel, all the points present will be approximated with their centroid
 * and all the normals with the average one (resultant vector).
 */
struct Voxel
{
  int x_, y_, z_; //!< voxel's coordinates
  bool occupied_; //!< occupancy flag
  Eigen::Vector3d avg_normal_; //!< average normal vector
  Eigen::Vector3d centroid_; //!< centroid of the voxel
  std::vector<unsigned int> counts_; //!< id of the point
};

/**
 * @brief The VoxelGrid class
 *
 * This class creates a 3D voxel grid over the input point list data
 */
class VoxelGrid{
public:
  /**
   * @brief Default constructor
   */
  VoxelGrid(){}

  /**
   * @brief Constructor
   * @param voxel_size the voxel size
   * @param voxel_grid_resolution the voxel grid resolution
   */
  VoxelGrid(double &voxel_size, int &voxel_grid_resolution):
    voxel_size_(voxel_size), voxel_grid_resolution_(voxel_grid_resolution){}

  ~VoxelGrid(){}

  /**
   * @brief set the point list for voxelization
   * @param point_list the input point list
   */
  void setPointList(PointList &point_list);

  /**
   * @brief voxelize the input point list
   * @return the voxelized point list object
   */
  PointList filter();

  /**
   * @brief get the voxel size
   * @return the voxel size
   */
  double getVoxelSize() const
  {
    return voxel_size_;
  }

  /**
   * @brief get the voxel grid resolution
   * @return the grid resolution
   */
  int getVoxelGridResolution() const
  {
    return voxel_grid_resolution_;
  }

  /**
   * @brief get the occupied voxels
   * @return a vector of points that contains the keys of occupies voxels
   */
  std::vector<Eigen::Vector3i> getOccupiedVoxels()
  {
    return occupied_voxels_;
  }

  /**
   * @brief set the voxel size
   * @param voxel_size the voxel size
   */
  void setVoxelSize(double &voxel_size)
  {
    voxel_size_ = voxel_size;
  }

  /**
   * @brief set the voxel grid resolution
   * @param voxel_grid_resolution the grid resolution
   */
  void setVoxelGridResolution(int &voxel_grid_resolution)
  {
    voxel_grid_resolution_ = voxel_grid_resolution;
  }

  Eigen::Vector3d getMinPt()
  {
      return min_pt_;
  }

  std::map<std::vector<int>, Voxel> getVoxelGridMap()
  {
      return voxel_grid_map_;
  }

  void getMinMax(Eigen::Vector3d &min_pt, Eigen::Vector3d &max_pt)
  {
      min_pt = min_pt_;
      max_pt = max_pt_;
  }

  void setMinMax(Eigen::Vector3d &min_pt, Eigen::Vector3d &max_pt)
  {
      min_pt_ = min_pt;
      max_pt_ = max_pt;
  }

private:
  double voxel_size_;  //!< voxel size
  int voxel_grid_resolution_;  //!< voxel grid resolution

  std::map<std::vector<int>, Voxel> voxel_grid_map_;
  std::vector<Eigen::Vector3i> occupied_voxels_;

  PointList point_list_; //!< input point list
  Eigen::Vector3d min_pt_, max_pt_;
};

}

#endif
