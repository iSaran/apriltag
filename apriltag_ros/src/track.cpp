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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <apriltag_ros/detector.h>

//opencv
#include <opencv/highgui.h>

#include <chrono>

// Params
std::string rgbd_a = "/asus_xtion/rgb/image_raw";
std::string rgbd_b = "/asus_xtion/depth/image_raw";

ros::Publisher detection_points_pub;

geometry_msgs::TransformStamped toROS(const Eigen::Affine3f& frame,
                                      const std::string& frame_name_1,
                                      const std::string& frame_name_2)
{
  geometry_msgs::TransformStamped transformStamped;
  Eigen::Quaternionf quat(frame.linear().matrix());
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

cv::Mat fromROS(const sensor_msgs::ImageConstPtr& img, bool depth=false)
{
  cv_bridge::CvImageConstPtr bgr_cv = cv_bridge::toCvCopy(img, img->encoding);
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


Eigen::Affine3f fromROS(const geometry_msgs::TransformStamped& pose_ros)
{
  Eigen::Affine3f pose;
  Eigen::Quaternionf quat(pose_ros.transform.rotation.w, pose_ros.transform.rotation.x,
                          pose_ros.transform.rotation.y, pose_ros.transform.rotation.z);
  pose.linear() = quat.toRotationMatrix();
  pose.translation()(0) = pose_ros.transform.translation.x;
  pose.translation()(1) = pose_ros.transform.translation.y;
  pose.translation()(2) = pose_ros.transform.translation.z;
  return pose;
}

Eigen::Affine3f getPoseFromTF(const std::string& frame_base, const std::string& frame_target)
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
Eigen::Affine3f estimatePose(const sensor_msgs::ImageConstPtr& rgb_ros, const cv::Mat& rgb, const cv::Mat& depth, const apriltag_ros::PinholeCamera& camera,
                             const apriltag_ros::AprilTagDetector& tag_detector, bool plot=false)
{
  // apriltag_ros::Timer t_1("whole function");

  // apriltag_ros::Timer t_3("EstimatePose:creating objects");
  // Detect tag
  // t_3.existenceTime();
  std::vector<cv::Point2f> pts;
  std::vector<cv::Point3f> objs;

  std::vector<Eigen::Vector2f> points2d;
  std::vector<Eigen::Vector3f> points3d;


  bool points_found;

  Eigen::Affine3f frame_wrt_camera;
  cv::Mat img = rgb;
  try
  {

    // apriltag_ros::Timer t_2("EstimatePose:getPose");

    std::vector<Eigen::Affine3f> frames_wrt_camera =
      tag_detector.getPose(rgb, depth, camera, points2d, points3d, false);
      
    // t_2.existenceTime();


    frame_wrt_camera = frames_wrt_camera.at(0);

    // apriltag_ros::Timer t_3("EstimatePose:plot");
    if (plot)
    {
      cv::circle(img, cv::Point2f(points2d.at(0)(0), points2d.at(0)(1)), 5, cv::Scalar(0, 0, 255), 3);
      cv::circle(img, cv::Point2f(points2d.at(1)(0), points2d.at(1)(1)), 5, cv::Scalar(0, 0, 255), 3);
      cv::circle(img, cv::Point2f(points2d.at(2)(0), points2d.at(2)(1)), 5, cv::Scalar(0, 0, 255), 3);
      cv::circle(img, cv::Point2f(points2d.at(3)(0), points2d.at(3)(1)), 5, cv::Scalar(0, 0, 255), 3);
      detection_points_pub.publish(toROS(img, rgb_ros->header, rgb_ros->encoding));
    }
    // t_3.existenceTime();
  }
  catch(const std::exception& e)
  {
    if (plot)
    {
      img.setTo(cv::Scalar(0, 0, 0));
      detection_points_pub.publish(toROS(img, rgb_ros->header, rgb_ros->encoding));
    }
    throw std::runtime_error(e.what());
  }

  // t_1.existenceTime();

  return frame_wrt_camera;
}

void publishTF(const Eigen::Affine3f& frame,
               const std::string& base_frame,
               const std::string& target_frame)
{
  ROS_INFO_STREAM("publishTF: " << base_frame << " , " << target_frame);
  static tf2_ros::TransformBroadcaster br;
  auto transformStamped = toROS(frame, base_frame, target_frame);
  br.sendTransform(transformStamped);
}

sensor_msgs::ImageConstPtr rgb, depth;

void rgbCb(const sensor_msgs::Image::ConstPtr& msg)
{
  rgb = msg;
}

void depthCb(const sensor_msgs::Image::ConstPtr& msg)
{
  depth = msg;
}

void rgbdCb(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::Image::ConstPtr& img_depth)
{
  rgb = img;
  depth = img_depth;
}

cv::Mat alignDepth(const cv::Mat& depth, const apriltag_ros::PinholeCamera& camera)
{
  cv::Mat depth_translated = cv::Mat::zeros(camera.size_y, camera.size_x, CV_16UC1);
  for (int i = 0; i < camera.size_x; i++)
  {
    for (int j =0; j < camera.size_y; j++)
    {
      float depth_value = (float)depth.at<unsigned short>(j, i);
      // if (depth_value > 0)
      // {
        Eigen::Vector2f img(i, j);
        Eigen::Vector3f point3d = camera.backProject(img, depth_value);
        point3d(0) -= 0.025;
        Eigen::Vector3f rgb_xy = camera.project(point3d);
        // rgb_xy(0) /= rgb_xy(2);
        // rgb_xy(1) /= rgb_xy(2);
        // std::cout << "calculate " << i << ", " << j << ": " << point3d.transpose() << ", 2d: " << rgb_xy.transpose() << std::endl;
        if (rgb_xy(0) > 0 && rgb_xy(1) > 0 && rgb_xy(0) <= camera.size_x && rgb_xy(1) <= camera.size_y)
        {
          depth_translated.at<unsigned short>((int)rgb_xy(1), (int)rgb_xy(0)) = rgb_xy(2);
        }
      // }
    }
  }
  cv::Mat depth_translated_ = depth_translated;
  for (int i = 0; i < camera.size_x; i++)
  {
    for (int j =0; j < camera.size_y; j++)
    {
      if (depth_translated.at<unsigned short>(j, i) < 50)
      {
        // if (i + 1 <= camera.size_x)
        // {
        //   depth_translated.at<unsigned short>(j, i) = depth_translated.at<unsigned short>(j, i + 1);
        // }
        // else if (i - 1 >=0 && depth_translated.at<unsigned short>(j, i - 1) > 50)
        // {
        //   depth_translated.at<unsigned short>(j, i) = depth_translated.at<unsigned short>(j, i - 1);
        // }
        if (j + 1 < camera.size_y)
        {
          depth_translated.at<unsigned short>(j, i) = depth_translated.at<unsigned short>(j + 1, i);
        }
        // else if (j - 1 >=0 && depth_translated.at<unsigned short>(j - 1, i) > 50)
        // {
        //   depth_translated.at<unsigned short>(j, i) = depth_translated.at<unsigned short>(j - 1, i);
        // }
      }
    }
  }
  return depth_translated;
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
  apriltag_ros::PinholeCamera camera(fx, fy, cx, cy, size.at(0), size.at(1));
  ROS_INFO_STREAM("Start tracking for " << camera_name << " camera, with intrinsics: "
                   << fx << ", " << fy << ", " << cx << ", " << cy
                   << ", size: " <<  size.at(0) << "x" << size.at(1));

  // Read camera TF name
  std::string camera_tf_name;
  n.getParam("apriltag_config/camera/" + camera_name + "/tf_name", camera_tf_name);

  // Read topic to read RGB
  std::string camera_rgb_topic;
  n.getParam("apriltag_config/camera/" + camera_name + "/topic_rgb", camera_rgb_topic);
  std::string camera_depth_topic;
  n.getParam("apriltag_config/camera/" + camera_name + "/topic_depth", camera_depth_topic);


  std::vector<int> board_size;
  float tile_size, tile_border;
  n.getParam("apriltag_config/pattern/board_size", board_size);
  n.getParam("apriltag_config/pattern/tile_size", tile_size);
  n.getParam("apriltag_config/pattern/tile_border", tile_border);
  // apriltag_ros::AprilTagParameters apriltag_params(board_size, tile_size, tile_border);
  ROS_INFO_STREAM("Tag params: board_size: " << board_size.at(0) << "x" << board_size.at(1)
                  << ", tile_size" << tile_size * 100 << "cm, "
                  << ", tile_border" << tile_border * 100 << "cm");

  bool plot_pts;
  n.getParam("apriltag_config/track/plot_points/use", plot_pts);
  std::string pts_topic;
  n.getParam("apriltag_config/track/plot_points/topic", pts_topic);

  detection_points_pub = n.advertise<sensor_msgs::Image>(pts_topic, 0, true);

  ros::Rate rate(100); // ROS Rate at 100Hz

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, camera_rgb_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, camera_depth_topic, 1);
  // message_filters::Subscriber<sensor_msgs::Image> depth_sub = n.subscribe(camera_depth_topic, 1000, depthCb);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&rgbdCb, _1, _2));

  int t = 0;
  ROS_INFO_STREAM("Tracking node is starting...");
  while (ros::ok() && t < 1000)
  {
    t++;
    ros::spinOnce();
    usleep(1 * 1000);
  }
  
  apriltag_ros::AprilTagDetector tag_detector(cv::Size(board_size.at(0), board_size.at(1)), tile_size, tile_border);

  while (ros::ok())
  {

    cv::Mat rgb_cv = fromROS(rgb);
    // cv::imshow("img", rgb_cv);
    // cv::waitKey();



    double minVal; 
    double maxVal; 
    cv::Point minLoc; 
    cv::Point maxLoc;

    cv::Mat depth_cv = fromROS(depth, true);
    cv::minMaxLoc(depth_cv, &minVal, &maxVal, &minLoc, &maxLoc );
    // std::cout << "min val: " << minVal << std::endl;
    // std::cout << "max val: " << maxVal << std::endl;
    double alpha = 255.0/(maxVal-minVal);
    cv::Mat depth_normalized;
    depth_cv.convertTo(depth_normalized, CV_8U, alpha, -minVal*alpha);
    // cv::imshow("img", depth_normalized);
    // cv::waitKey();

    cv::Mat depth_aligned = alignDepth(depth_cv, camera);

    cv::minMaxLoc(depth_aligned, &minVal, &maxVal, &minLoc, &maxLoc );
    std::cout << "min val: " << minVal << std::endl;
    std::cout << "max val: " << maxVal << std::endl;
    alpha = 255.0/(maxVal-minVal);
    depth_aligned.convertTo(depth_normalized, CV_8U, alpha, -minVal*alpha);
    // cv::imshow("img", depth_normalized);
    // cv::waitKey();



    Eigen::Affine3f pose;
    try
    {
      // using clock = std::chrono::steady_clock;
      // clock::time_point start = clock::now();
      pose = estimatePose(rgb, rgb_cv, depth_aligned, camera, tag_detector, plot_pts);
      // clock::time_point end = clock::now();
      // std::chrono::duration<double, std::milli> execution_time = end - start;  // in ms
      // std::cout << "Estimate pose in loop took " << execution_time.count() << " ms" << std::endl;
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
