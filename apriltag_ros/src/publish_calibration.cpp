/*******************************************************************************
 * Copyright (c) 2019 Automation and Robotics Lab, AUTh
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <apriltag_ros/calibrator.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>
#include <ros/package.h>

// // #include <std_srvs/Trigger.h>
// // #include <tf2_ros/transform_listener.h>
// // #include <rosba_msgs/ModifyPose.h>
// // #include <pcl_conversions/pcl_conversions.h>
// // #include "cv_tools/util/rgbd_utils.hpp"
// // #include <stdexcept>

// // system
// #include <thread>

// // apriltag_detector
// #include <apriltag_ros/detector.h>

// //opencv
// #include <opencv/highgui.h>

#include <opencv/highgui.h>

cv::Mat fromROS(const sensor_msgs::ImageConstPtr& img, bool depth=false)
{
  cv_bridge::CvImageConstPtr bgr_cv = cv_bridge::toCvCopy(img, img->encoding);
  std::cout << "endodinggggggggggggggg: " << img->encoding << std::endl;
  cv::Mat rgb;
  if (depth)
  {
    rgb = bgr_cv->image;
  }
  else
  {
    cv::cvtColor(bgr_cv->image, rgb, CV_BGR2RGB);
  }
  return rgb;
}

geometry_msgs::TransformStamped toROS(const Eigen::Affine3f& frame,
                                      const std::string& frame_name_1,
                                      const std::string& frame_name_2)
{
  geometry_msgs::TransformStamped transform_stamped;
  Eigen::Quaternionf quat(frame.linear().matrix());
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = frame_name_1;
  transform_stamped.child_frame_id = frame_name_2;
  transform_stamped.transform.translation.x = frame.translation()(0);
  transform_stamped.transform.translation.y = frame.translation()(1);
  transform_stamped.transform.translation.z = frame.translation()(2);
  transform_stamped.transform.rotation.x = quat.x();
  transform_stamped.transform.rotation.y = quat.y();
  transform_stamped.transform.rotation.z = quat.z();
  transform_stamped.transform.rotation.w = quat.w();
  return transform_stamped;
}




void publishTF(const Eigen::Affine3f& frame,
               const std::string& base_frame,
               const std::string& target_frame)
{
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped = toROS(frame, base_frame, target_frame);
  br.sendTransform(transformStamped);
}






template<class Matrix>
void writeBinary(const std::string& filename, const Matrix& matrix){
    std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
    out.write((char*) (&rows), sizeof(typename Matrix::Index));
    out.write((char*) (&cols), sizeof(typename Matrix::Index));
    out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
    out.close();
}


template<class Matrix>
void readBinary(const std::string& filename, Matrix& matrix){
    std::ifstream in(filename, std::ios::in | std::ios::binary);
    typename Matrix::Index rows=0, cols=0;
    in.read((char*) (&rows),sizeof(typename Matrix::Index));
    in.read((char*) (&cols),sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
    in.close();
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "apriltag_object_tracker");
  ros::NodeHandle n;

  std::string camera_name;
  n.getParam("apriltag_config/camera/use", camera_name);

  std::string camera_frame;
  n.getParam("apriltag_config/camera/" + camera_name + "/tf_name", camera_frame);

  std::string setup;
  n.getParam("apriltag_config/calibration/setup", setup);

  std::string base_frame;
  n.getParam("apriltag_config/calibration/" + setup + "/base_frame", base_frame);

  Eigen::Affine3f transformation;

  // Read calibration from file
  std::string file_path = ros::package::getPath("apriltag_ros") + "/output/calibration_transform.bin";
  readBinary<Eigen::Matrix4f>(file_path, transformation.matrix());
  ros::Rate rate(200);
  while(ros::ok())
  {
    // ROS_INFO_STREAM("Publishing calibration matrix of frames: " << base_frame << " , " << camera_frame);
    publishTF(transformation, base_frame, camera_frame);
    rate.sleep();
  }

  return 0;
}
