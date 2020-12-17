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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include "cv_tools/util/rgbd_utils.hpp"
#include <stdexcept>

// system
#include <thread>

// apriltag_detector
#include <apriltag_ros/detector.h>

//opencv
#include <opencv/highgui.h>

// Params
std::string rgbd_a = "/asus_xtion/rgb/image_raw";
std::string rgbd_b = "/asus_xtion/depth/image_raw";

ros::Publisher detection_points_pub;

geometry_msgs::TransformStamped toROS(const Eigen::Affine3d& frame,
                                      const std::string& frame_name_1,
                                      const std::string& frame_name_2)
{
  geometry_msgs::TransformStamped transformStamped;
  Eigen::Quaterniond quat(frame.linear().matrix());
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = frame_name_1;
  transformStamped.child_frame_id = frame_name_2;
  transformStamped.transform.translation.x = frame.translation()(0);
  transformStamped.transform.translation.y = frame.translation()(1);
  transformStamped.transform.translation.z = frame.translation()(2);
  transformStamped.transform.rotation.x = quat.x();
  transformStamped.transform.rotation.y = quat.y();
  transformStamped.transform.rotation.z = quat.z();
  transformStamped.transform.rotation.w = quat.w();
  return transformStamped;
}

sensor_msgs::ImagePtr toROS(const cv::Mat& img, const std_msgs::Header& header, const std::string& encoding)
{
  cv_bridge::CvImage out_msg;
  cv::Mat bgr;
  cv::cvtColor(img, bgr, CV_RGB2BGR);
  out_msg.encoding = encoding;
  out_msg.image    = bgr;
  out_msg.header   = header;
  return out_msg.toImageMsg();
}

cv::Mat fromROS(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImageConstPtr bgr_cv = cv_bridge::toCvCopy(img, img->encoding);
  cv::Mat rgb;
  cv::cvtColor(bgr_cv->image, rgb, CV_BGR2RGB);
  return rgb;
}

Eigen::Affine3d fromROS(const geometry_msgs::TransformStamped& pose_ros)
{
  Eigen::Affine3d pose;
  Eigen::Quaterniond quat(pose_ros.transform.rotation.w, pose_ros.transform.rotation.x,
                          pose_ros.transform.rotation.y, pose_ros.transform.rotation.z);
  pose.linear() = quat.toRotationMatrix();
  pose.translation()(0) = pose_ros.transform.translation.x;
  pose.translation()(1) = pose_ros.transform.translation.y;
  pose.translation()(2) = pose_ros.transform.translation.z;
  return pose;
}

Eigen::Affine3d getPoseFromTF(const std::string& frame_base, const std::string& frame_target)
{
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tfListener(tf_buffer);
  geometry_msgs::TransformStamped tf_frame;
  try
  {
    tf_frame = tf_buffer.lookupTransform(frame_base, frame_target, ros::Time(0), ros::Duration(10));
  }
  catch (tf2::TransformException &ex)
  {
    throw "TF trasform between " + frame_base + " and " + frame_target + " wasn't found:" + ex.what();
  }
  return fromROS(tf_frame);
}


/**
 * Input: RGB image, Output: POse of apriltag
 */
Eigen::Affine3d estimatePose(const sensor_msgs::ImageConstPtr& rgb_ros, const cv::Mat& rgb, const cv_tools::PinholeCamera& camera,
                             const apriltag_ros::AprilTagParameters& params, bool plot=false)
{
  // Detect tag
  apriltag_ros::AprilTagDetector tag_detector(params);
  apriltag_ros::PoseEstimator estimator;
  std::vector<cv::Point2f> pts;
  std::vector<cv::Point3f> objs;
  bool points_found;
  points_found = tag_detector.findPoints(pts, objs, rgb);


  if (plot)
  {
    cv::Mat img = rgb;
    if (points_found)
    {
      cv::circle(img, pts[0], 5, cv::Scalar(0, 0, 255), 3);
      cv::circle(img, pts[1], 5, cv::Scalar(0, 0, 255), 3);
      cv::circle(img, pts[2], 5, cv::Scalar(0, 0, 255), 3);
      cv::circle(img, pts[3], 5, cv::Scalar(0, 0, 255), 3);
    }
    else
    {
      img.setTo(cv::Scalar(0, 0, 0));
    }

    detection_points_pub.publish(toROS(img, rgb_ros->header, rgb_ros->encoding));
  }

  if(!points_found)
  { 
    throw std::runtime_error("Cannot detect tags!");
  }

  Eigen::Affine3d frame_wrt_camera = estimator.estimate(pts, objs, camera);
  return frame_wrt_camera;
}

void publishTF(const Eigen::Affine3d& frame,
               const std::string& base_frame,
               const std::string& target_frame)
{
  ROS_INFO_STREAM("publishTF: " << base_frame << " , " << target_frame);
  static tf2_ros::TransformBroadcaster br;
  auto transformStamped = toROS(frame, base_frame, target_frame);
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

  // Read intrinsics of camera
  double fx, fy, cx, cy;
  std::vector<int> size;
  n.getParam("apriltag_config/camera/" + camera_name + "/fx", fx);
  n.getParam("apriltag_config/camera/" + camera_name + "/fy", fy);
  n.getParam("apriltag_config/camera/" + camera_name + "/cx", cx);
  n.getParam("apriltag_config/camera/" + camera_name + "/cy", cy);
  n.getParam("apriltag_config/camera/" + camera_name + "/size", size);
  cv_tools::PinholeCamera camera(fx, fy, cx, cy, cv::Size(size.at(0), size.at(1)));
  ROS_INFO_STREAM("Start tracking for " << camera_name << " camera, with intrinsics: "
                   << fx << ", " << fy << ", " << cx << ", " << cy
                   << ", size: " <<  size.at(0) << "x" << size.at(1));

  // Read camera TF name
  std::string camera_tf_name;
  n.getParam("apriltag_config/camera/" + camera_name + "/tf_name", camera_tf_name);

  // Read topic to read RGB
  std::string camera_rgb_topic;
  n.getParam("apriltag_config/camera/" + camera_name + "/topic_rgb", camera_rgb_topic);


  std::vector<int> board_size;
  float tile_size, tile_border;
  n.getParam("apriltag_config/pattern/board_size", board_size);
  n.getParam("apriltag_config/pattern/tile_size", tile_size);
  n.getParam("apriltag_config/pattern/tile_border", tile_border);
  apriltag_ros::AprilTagParameters apriltag_params(board_size, tile_size, tile_border);
  ROS_INFO_STREAM("Tag params: board_size: " << board_size.at(0) << "x" << board_size.at(1)
                  << ", tile_size" << tile_size * 100 << "cm, "
                  << ", tile_border" << tile_border * 100 << "cm");

  bool plot_pts;
  n.getParam("apriltag_config/track/plot_points/use", plot_pts);
  std::string pts_topic;
  n.getParam("apriltag_config/track/plot_points/topic", pts_topic);

  detection_points_pub = n.advertise<sensor_msgs::Image>(pts_topic, 0, true);

  ros::Rate rate(60); // ROS Rate at 60Hz
  sensor_msgs::ImageConstPtr rgb;
  while (ros::ok())
  {
    rgb = ros::topic::waitForMessage<sensor_msgs::Image>(camera_rgb_topic, ros::Duration(1.0));
    cv::Mat rgb_cv = fromROS(rgb);

    Eigen::Affine3d pose;
    try
    {
      pose = estimatePose(rgb, rgb_cv, camera, apriltag_params, plot_pts);
    }
    catch(const std::exception& e)
    {
      ROS_WARN_THROTTLE(1, e.what());
      continue;
    }
    
    publishTF(pose, camera_tf_name, "target_object");
    ros::spinOnce();
    rate.sleep();
  }

  // ros::ServiceServer service = n.advertiseService("estimate_pose", estimatePose);
  // ros::ServiceServer service2 = n.advertiseService("modify_pose", modifyPose);
  // pub = n.advertise<sensor_msgs::PointCloud2>("rosba_pc", 0, true);

  return 0;
}
