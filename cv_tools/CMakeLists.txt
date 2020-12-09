cmake_minimum_required(VERSION 2.8.3)
project(cv_tools)

# Use C++  for initialise an std::vector without a constructor
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH} )

# Build in debug mode
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenGL)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL 1.4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  roslib
  geometry_msgs
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

###########
## Build ##
###########

# Computer Vision Tools Library
add_library(${PROJECT_NAME}
  src/viz/off_screen_renderer.cpp
  src/viz/pcl_visualizer.cpp

  src/util/rgbd_utils.cpp
  src/util/eigen_utils.cpp

  src/geometry/voxel_grid.cpp
  src/geometry/point_list.cpp
  src/geometry/viewpoint_sampler.cpp
  src/geometry/normal_estimator.cpp
  src/geometry/kdtree.cpp
  src/geometry/octree.cpp

  src/imgproc/rgbd.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
