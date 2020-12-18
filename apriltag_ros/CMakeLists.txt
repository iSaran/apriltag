cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ros)

# Use C++  for initialise an std::vector without a constructor
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH} )
message("!!!!!!!!!!!!!!! Package CMAKE_CURRENT_SOURCE_DIR" ${CMAKE_CURRENT_SOURCE_DIR})

# Build in debug mode
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Apriltag REQUIRED)
find_package(PCL 1.4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  std_msgs
  cv_tools
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf2_ros
  pcl_conversions
)

message("!!!!!!!!!!!!!!! Include dires from CMAKELISTS" ${Apriltag_INCLUDE_DIRS})
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include ${Apriltag_INCLUDE_DIRS}
  CATKIN_DEPENDS cv_tools
  DEPENDS OpenCV Apriltag
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Apriltag_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


set(SRCS
    src/detector.cpp
    src/calibrator.cpp
)

add_library(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Apriltag_LIBRARIES}
)

add_executable(${PROJECT_NAME}_estimate_pose src/nodes/estimate_pose.cpp ${SRCS})
add_dependencies(${PROJECT_NAME}_estimate_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_estimate_pose
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Apriltag_LIBRARIES}
)

add_executable(make_pattern src/nodes/make_pattern.cpp ${SRCS})
add_dependencies(make_pattern ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(make_pattern
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Apriltag_LIBRARIES}
)

add_executable(calibrate src/nodes/calibrate.cpp ${SRCS})
add_dependencies(calibrate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calibrate
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Apriltag_LIBRARIES}
)

add_executable(publish_calibration src/publish_calibration.cpp ${SRCS})
add_dependencies(publish_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Apriltag_LIBRARIES}
)

## Run roslint with `catkin_make roslint`
# Filter out errors with respect C++11 (for ROS-indigo)
set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references,-build/include_what_you_use")
# Roslint macro
roslint_cpp()

add_executable(tracker src/track.cpp ${SRCS})
add_dependencies(tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Apriltag_LIBRARIES}
)
