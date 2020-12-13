cmake_minimum_required(VERSION 2.8.3)
project(rosba_object_detection)

# Use C++  for initialise an std::vector without a constructor
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Build in debug mode
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.4 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  tf2
  tf2_ros
  sensor_msgs
  cv_bridge
  apriltag_detector
  pcl_conversions
  rosba_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS apriltag_detector
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(${PROJECT_NAME}_detect src/detect.cpp)
add_dependencies(${PROJECT_NAME}_detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_detect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}

)

add_executable(tracker src/track.cpp)
add_dependencies(tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Run roslint with `catkin_make roslint`
# Filter out errors with respect C++11 (for ROS-indigo)
set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references,-build/include_what_you_use")
# Roslint macro
roslint_cpp()
