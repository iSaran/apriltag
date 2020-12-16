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
#include <apriltag_core/calibrator.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
// // #include <std_srvs/Trigger.h>
// // #include <tf2_ros/transform_listener.h>
// // #include <rosba_msgs/ModifyPose.h>
// // #include <pcl_conversions/pcl_conversions.h>
// // #include "cv_tools/util/rgbd_utils.hpp"
// // #include <stdexcept>

// // system
// #include <thread>

// // apriltag_detector
// #include <apriltag_core/detector.h>

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
  ROS_INFO_STREAM("publishTF: " << base_frame << " , " << target_frame);
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped = toROS(frame, base_frame, target_frame);
  br.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "apriltag_object_tracker");
  ros::NodeHandle n;

  // Which camera we use
  std::string camera_name;
  n.getParam("apriltag_config/camera/use", camera_name);

  // Read intrinsics of camera from parameter server
  double fx, fy, cx, cy;
  std::vector<int> size;
  n.getParam("apriltag_config/camera/" + camera_name + "/fx", fx);
  n.getParam("apriltag_config/camera/" + camera_name + "/fy", fy);
  n.getParam("apriltag_config/camera/" + camera_name + "/cx", cx);
  n.getParam("apriltag_config/camera/" + camera_name + "/cy", cy);
  n.getParam("apriltag_config/camera/" + camera_name + "/size", size);
  ROS_INFO_STREAM("Calibrating with " << camera_name << " camera, with intrinsics: "
                   << fx << ", " << fy << ", " << cx << ", " << cy
                   << ", size: " <<  size.at(0) << "x" << size.at(1));

  // Read topic to read RGB, depth
  std::string camera_rgb_topic;
  n.getParam("apriltag_config/camera/" + camera_name + "/topic_rgb", camera_rgb_topic);

  std::string camera_depth_topic;
  n.getParam("apriltag_config/camera/" + camera_name + "/topic_depth", camera_depth_topic);


  // sensor_msgs::ImageConstPtr rgb = ros::topic::waitForMessage<sensor_msgs::Image>(camera_rgb_topic, ros::Duration(1.0));
  // cv::Mat rgb_cv = fromROS(rgb);


  // Read robot measurements for calibration
  std::string setup;
  n.getParam("apriltag_config/calibration/setup", setup);
  int n_tags;
  n.getParam("apriltag_config/calibration/" + setup + "/n_tags", n_tags);
  std::vector<Eigen::Vector3f> robot_points(n_tags * 4);
  std::vector<double> robot_point;
  for (int i = 0; i < n_tags; i++)
  {
    n.getParam("apriltag_config/calibration/" + setup + "/id" + std::to_string(i) + "/A", robot_point);
    robot_points.at(i * 4 + 0)[0] = robot_point.at(0);
    robot_points.at(i * 4 + 0)[1] = robot_point.at(1);
    robot_points.at(i * 4 + 0)[2] = robot_point.at(2);

    n.getParam("apriltag_config/calibration/" + setup + "/id" + std::to_string(i) + "/B", robot_point);
    robot_points.at(i * 4 + 1)[0] = robot_point.at(0);
    robot_points.at(i * 4 + 1)[1] = robot_point.at(1);
    robot_points.at(i * 4 + 1)[2] = robot_point.at(2);

    n.getParam("apriltag_config/calibration/" + setup + "/id" + std::to_string(i) + "/C", robot_point);
    robot_points.at(i * 4 + 2)[0] = robot_point.at(0);
    robot_points.at(i * 4 + 2)[1] = robot_point.at(1);
    robot_points.at(i * 4 + 2)[2] = robot_point.at(2);

    n.getParam("apriltag_config/calibration/" + setup + "/id" + std::to_string(i) + "/D", robot_point);
    robot_points.at(i * 4 + 3)[0] = robot_point.at(0);
    robot_points.at(i * 4 + 3)[1] = robot_point.at(1);
    robot_points.at(i * 4 + 3)[2] = robot_point.at(2);
  }

  std::vector<int> board_size;
  float tile_size, tile_border;
  n.getParam("apriltag_config/pattern/board_size", board_size);
  n.getParam("apriltag_config/pattern/tile_size", tile_size);
  n.getParam("apriltag_config/pattern/tile_border", tile_border);
  apriltag_core::AprilTagParameters apriltag_params(board_size, tile_size, tile_border);

  bool plot;
  n.getParam("apriltag_config/calibration/plot", plot);

  // Start calibration

  sensor_msgs::ImageConstPtr rgb = ros::topic::waitForMessage<sensor_msgs::Image>(camera_rgb_topic, ros::Duration(1.0));
  cv::Mat rgb_cv = fromROS(rgb);

  sensor_msgs::ImageConstPtr depth = ros::topic::waitForMessage<sensor_msgs::Image>(camera_depth_topic, ros::Duration(1.0));
  cv::Mat depth_cv = fromROS(depth, true);

  apriltag_core::Calibrator calibrator(apriltag_params, fx, fy, cx, cy, 3);
  Eigen::Affine3f transformation = calibrator.run(rgb_cv, depth_cv, robot_points, plot);
  transformation.translation()[0] += 0.03;
  transformation.translation()[1] += 0.03;
  while(ros::ok())
  publishTF(transformation, "world", "asus_xtion_rgb_optical_frame");
  // std::cout << "Calibration transformation: " << std::endl << transformation << std::endl;

  return 0;
}
