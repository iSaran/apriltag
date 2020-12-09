cmake_minimum_required(VERSION 2.8.3)
project(apriltag_detector)

# Use C++  for initialise an std::vector without a constructor
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH} )

# Build in debug mode
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  std_msgs
  cv_tools
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include 3rdparty/apriltag
  CATKIN_DEPENDS cv_tools
  DEPENDS OpenCV
)

include_directories(
  include
  3rdparty/apriltag
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

ADD_SUBDIRECTORY(3rdparty/apriltag)

set(SRCS
    src/apriltag_detector.cpp
)

add_library(${PROJECT_NAME} src/apriltag_detector.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  apriltag
)

add_executable(${PROJECT_NAME}_estimate_pose src/nodes/estimate_pose.cpp ${SRCS})
add_dependencies(${PROJECT_NAME}_estimate_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_estimate_pose
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  apriltag
)

add_executable(${PROJECT_NAME}_make_pattern src/nodes/make_pattern.cpp ${SRCS})
add_dependencies(${PROJECT_NAME}_make_pattern ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_make_pattern
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  apriltag
)

## Run roslint with `catkin_make roslint`
# Filter out errors with respect C++11 (for ROS-indigo)
set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references,-build/include_what_you_use")
# Roslint macro
roslint_cpp()
