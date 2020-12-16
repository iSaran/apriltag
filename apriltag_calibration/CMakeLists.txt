cmake_minimum_required(VERSION 2.8.3)
project(apriltag_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH} )

# Build in debug mode
set(CMAKE_BUILD_TYPE Debug)

find_package(PCL 1.7.2 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VTK 6 REQUIRED)
find_package(Apriltag REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  tf2
  tf2_ros
  sensor_msgs
  cv_bridge
  pcl_conversions
)


catkin_package(
  INCLUDE_DIRS include ${Apriltag_INCLUDE_DIRS}
)


include_directories(
    include
    /usr/local/include/eigen3        
    ${Apriltag_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}        
)

link_directories(../apriltag)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# add_executable(calibrate src/calibrate.cpp)
# add_dependencies(calibrate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_detect
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${PCL_LIBRARIES}

# )

# add_executable(tracker src/track.cpp)
# add_dependencies(tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(tracker
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# ## Run roslint with `catkin_make roslint`
# # Filter out errors with respect C++11 (for ROS-indigo)
# set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references,-build/include_what_you_use")
# # Roslint macro
# roslint_cpp()

file(GLOB INCLUDE_LIST "./include/*.h" "./include/*.hpp" "./include/cuda/*.h")
aux_source_directory(./src SRC_LIST)

add_executable(calibrate ${SRC_LIST} ${INCLUDE_LIST})

link_directories(./lib)

target_link_libraries(calibrate
    ${BOOST_LIBRARIES}
    ${OpenCV_LIBS}
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    # ${PROJECT_SOURCE_DIR}/lib/libapriltags.a
    ${Apriltag_LIBRARIES}
)