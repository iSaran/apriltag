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
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <rosba_msgs/ModifyPose.h>
#include <pcl_conversions/pcl_conversions.h>
#include "cv_tools/util/rgbd_utils.hpp"

// system
#include <thread>

// apriltag_detector
#include "apriltag_detector/apriltag_detector.hpp"

//opencv
#include <opencv/highgui.h>

const unsigned int MAX_DETECTIONS = 20;
ros::Publisher pub;
geometry_msgs::TransformStamped transformStamped;
Eigen::Affine3d target_frame_world;

geometry_msgs::TransformStamped toROS(const Eigen::Affine3d& frame,
                                      const std::string& frame_name_1,
                                      const std::string& frame_name_2)
{
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

void publishPose(const Eigen::Affine3d& frame,
                 const std::string& frame_name_1,
                 const std::string& frame_name_2)
{
  static tf2_ros::StaticTransformBroadcaster br;
  auto transformStamped = toROS(frame, frame_name_1, frame_name_2);
  br.sendTransform(transformStamped);
}

bool modifyPose(rosba_msgs::ModifyPose::Request &req,
                rosba_msgs::ModifyPose::Response &res)
{
  Eigen::Vector3d displacement = target_frame_world.linear().matrix() * Eigen::Vector3d(req.x, req.y, 0);
  target_frame_world.translation() += displacement;
  publishPose(target_frame_world, "world", "target_object");
  res.success = true;
  return true;
}

bool estimatePose(std_srvs::Trigger::Request &req,
                  std_srvs::Trigger::Response &res)
{
  // Capture data
//  boost::shared_ptr<sensor_msgs::PointCloud2 const> pcd;
//  pcd = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("", ros::Duration(1.0));

  for (unsigned int i = 0; i < MAX_DETECTIONS; i++)
  {
    boost::shared_ptr<sensor_msgs::Image const> bgr;
    bgr = ros::topic::waitForMessage<sensor_msgs::Image>("/asus_xtion/rgb/image_raw", ros::Duration(1.0));

    boost::shared_ptr<sensor_msgs::Image const> depth;
    depth = ros::topic::waitForMessage<sensor_msgs::Image>("/asus_xtion/depth/image_raw", ros::Duration(1.0));

    cv_bridge::CvImageConstPtr depth_cv = cv_bridge::toCvCopy(depth, depth->encoding);

    cv_bridge::CvImageConstPtr cvPtr1 = cv_bridge::toCvCopy(bgr, bgr->encoding);

    // get an RGB-D image
    cv::Mat rgb;
    cv::cvtColor(cvPtr1->image, rgb, CV_BGR2RGB);

    // cv::imshow("rgb", rgb);
    // cv::waitKey();

    double alpha = 1.0;
    double beta = -50.0;
    // rgb.convertTo(rgb, -1, alpha, beta);
    // cv::imshow("rgb_converted", rgb);
    // cv::waitKey();

    // detect apriltags
    AprilTagDetector::AprilTagParameters apriltagParams;
    AprilTagDetector tagDetector;
    PoseEstimator estimator;

    std::vector<cv::Point2f> pts;
    std::vector<cv::Point3f> objs;
    if(tagDetector.findPoints(pts, objs, rgb, apriltagParams) == false)
    {
      std::cout << "Cannot detect tags!" << std::endl;
      res.success = false;
    }
    else
    {
      // estimate affine transformation
      PinholeCamera camera(543.951475, 545.531957, 320, 240, cv::Size(640, 480));
      Eigen::Affine3d tr = estimator.estimate(pts, objs, camera);

      // estimator.drawAxes(rgb, camera, pts[0]);

      int i = pts[0].x;
      int j = pts[0].y;
      cv::Mat depth_map = depth_cv->image;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
      cloud = cv_tools::depthToPointCloud(depth_map, camera);
      pcl::PCLPointCloud2 pc2;
      pcl::toPCLPointCloud2(*cloud, pc2);
      sensor_msgs::PointCloud2 pc2_ros;
      pcl_conversions::fromPCL(pc2, pc2_ros);
      pc2_ros.header.frame_id = "asus_xtion_depth_optical_frame";
      pub.publish(pc2_ros);

      unsigned short depth_value = depth_map.at<unsigned short>(j, i);
      double z = double(depth_value) / 1000.0;
      double x = (i - camera.cx()) * z / camera.fx();
      double y = (j - camera.cy()) * z / camera.fy();


      // Eigen::Vector3d position = tr.translation();
      Eigen::Vector3d position(x, y, z);
      Eigen::Matrix3d rotation = tr.rotation();
      Eigen::Affine3d target_frame_camera;
      target_frame_camera.translation() << position;
      target_frame_camera.linear() << rotation;
      // std::cout << "position:" << position.transpose() << std::endl;
      // std::cout << "rotation:" << rotation << std::endl;

      // Read pose of camera
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      geometry_msgs::TransformStamped tf_camera;
      try
      {
        tf_camera = tfBuffer.lookupTransform("world", "asus_xtion_rgb_optical_frame", ros::Time(0), ros::Duration(10));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.5).sleep();
          ROS_ERROR("Camera frame wasn't found!!!!");
          res.success = false;
      }

      // Transform object to world
      Eigen::Affine3d camera_wrt_world;
      camera_wrt_world.translation() << tf_camera.transform.translation.x, tf_camera.transform.translation.y, tf_camera.transform.translation.z;
      auto quat = Eigen::Quaterniond(tf_camera.transform.rotation.w, tf_camera.transform.rotation.x, tf_camera.transform.rotation.y, tf_camera.transform.rotation.z);
      camera_wrt_world.linear() = quat.toRotationMatrix();
      target_frame_world = camera_wrt_world * target_frame_camera;
      target_frame_world.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      Eigen::Vector3d displacement = target_frame_world.linear().matrix() * Eigen::Vector3d(0.02, -0.025, 0);
      target_frame_world.translation() += displacement;
      target_frame_world.translation()(2) = 0.025;

      if (target_frame_world.linear().col(2).dot(Eigen::Vector3d(0, 0, 1)) < 0.8)
      {
        res.success = false;
        ROS_WARN("Detection failed, trying again...");
      }
      else
      {
        Eigen::Vector3d x = target_frame_world.linear().col(0);
        Eigen::Vector3d z = Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d x_proj = x - (x.dot(z) / (z.norm() * z.norm())) * z;
        x_proj = x_proj/x_proj.norm();
        Eigen::Vector3d y = z.cross(x_proj);
        target_frame_world.linear().col(0) = x_proj;
        target_frame_world.linear().col(1) = y;
        target_frame_world.linear().col(2) = z;
        res.success = true;
        ROS_INFO("Successful detection");
        break;
      }
    }
  }

  if (!res.success)
  {
    return res.success;
  }

   Eigen::Quaterniond quat2(target_frame_world.linear().matrix());
   std::cout << target_frame_world.linear().matrix() << std::endl;
   std::cout << target_frame_world.translation().transpose() << std::endl;

   // Publish tf
  publishPose(target_frame_world, "world", "target_object");
  res.success = true;
  return true;
}


int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "rosba_object_detector");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("estimate_pose", estimatePose);
  ros::ServiceServer service2 = n.advertiseService("modify_pose", modifyPose);
  pub = n.advertise<sensor_msgs::PointCloud2>("rosba_pc", 0, true);

  ros::spin();

  return 0;
}
