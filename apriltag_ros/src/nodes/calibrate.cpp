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
#include <autharl_core>

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


template<class Matrix>
void writeBinary(const std::string& filename, const Matrix& matrix)
{
    std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
    out.write((char*) (&rows), sizeof(typename Matrix::Index));
    out.write((char*) (&cols), sizeof(typename Matrix::Index));
    out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
    out.close();
}


template<class Matrix>
void readBinary(const std::string& filename, Matrix& matrix)
{
    std::ifstream in(filename, std::ios::in | std::ios::binary);
    typename Matrix::Index rows=0, cols=0;
    in.read((char*) (&rows),sizeof(typename Matrix::Index));
    in.read((char*) (&cols),sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
    in.close();
}

// void publishPointsRviz()
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
        if (rgb_xy(0) > 0 && rgb_xy(1) > 0 && rgb_xy(0) <= camera.size_x && rgb_xy(1) <= camera.size_y)
        {
          depth_translated.at<unsigned short>((int)rgb_xy(1), (int)rgb_xy(0)) = rgb_xy(2);
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
  
  apriltag_ros::PinholeCamera camera(fx, fy, cx, cy, size.at(0), size.at(1));

  // Read topic to read RGB, depth
  std::string camera_rgb_topic;
  n.getParam("apriltag_config/camera/" + camera_name + "/topic_rgb", camera_rgb_topic);

  std::string camera_depth_topic;
  n.getParam("apriltag_config/camera/" + camera_name + "/topic_depth", camera_depth_topic);

  // sensor_msgs::ImageConstPtr rgb = ros::topic::waitForMessage<sensor_msgs::Image>(camera_rgb_topic, ros::Duration(1.0));
  // cv::Mat rgb_cv = fromROS(rgb);

  arl::viz::RVisualizer rviz("world");
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
    rviz.visualizeSphere(robot_points.at(i * 4 + 0), 0.01, rviz_visual_tools::colors::RED);

    n.getParam("apriltag_config/calibration/" + setup + "/id" + std::to_string(i) + "/B", robot_point);
    robot_points.at(i * 4 + 1)[0] = robot_point.at(0);
    robot_points.at(i * 4 + 1)[1] = robot_point.at(1);
    robot_points.at(i * 4 + 1)[2] = robot_point.at(2);
    rviz.visualizeSphere(robot_points.at(i * 4 + 1), 0.01, rviz_visual_tools::colors::GREEN);

    n.getParam("apriltag_config/calibration/" + setup + "/id" + std::to_string(i) + "/C", robot_point);
    robot_points.at(i * 4 + 2)[0] = robot_point.at(0);
    robot_points.at(i * 4 + 2)[1] = robot_point.at(1);
    robot_points.at(i * 4 + 2)[2] = robot_point.at(2);
    rviz.visualizeSphere(robot_points.at(i * 4 + 2), 0.01, rviz_visual_tools::colors::BLUE);

    n.getParam("apriltag_config/calibration/" + setup + "/id" + std::to_string(i) + "/D", robot_point);
    robot_points.at(i * 4 + 3)[0] = robot_point.at(0);
    robot_points.at(i * 4 + 3)[1] = robot_point.at(1);
    robot_points.at(i * 4 + 3)[2] = robot_point.at(2);
    rviz.visualizeSphere(robot_points.at(i * 4 + 3), 0.01, rviz_visual_tools::colors::BLACK);
  }

  std::vector<int> board_size;
  float tile_size, tile_border;
  n.getParam("apriltag_config/pattern/board_size", board_size);
  n.getParam("apriltag_config/pattern/tile_size", tile_size);
  n.getParam("apriltag_config/pattern/tile_border", tile_border);

  bool plot;
  n.getParam("apriltag_config/calibration/plot", plot);

  // Start calibration

  sensor_msgs::ImageConstPtr rgb = ros::topic::waitForMessage<sensor_msgs::Image>(camera_rgb_topic, ros::Duration(1.0));
  cv::Mat rgb_cv = fromROS(rgb);

  sensor_msgs::ImageConstPtr depth = ros::topic::waitForMessage<sensor_msgs::Image>(camera_depth_topic, ros::Duration(1.0));
  cv::Mat depth_cv = fromROS(depth, true);
  cv::Mat depth_aligned = alignDepth(depth_cv, camera);

  apriltag_ros::Calibrator calibrator(cv::Size(board_size.at(0),
                                      board_size.at(1)), tile_size, tile_border, fx, fy, cx, cy, size.at(0),
                                      size.at(1), 3);
  Eigen::Affine3f transformation = calibrator.run(rgb_cv, depth_aligned, robot_points, plot);

  // Hardcoded fixes
  // transformation.translation()[0] += 0.03;
  // transformation.translation()[1] += 0.03;

  // Write calibration to file
  std::string file_path = ros::package::getPath("apriltag_ros") + "/output/calibration_transform.bin";
  writeBinary<Eigen::Matrix4f>(file_path, transformation.matrix());
  file_path = ros::package::getPath("apriltag_ros") + "/output/calibration_transform.txt";
  std::ofstream file(file_path);
  if (file.is_open())
  {
    file << transformation.matrix() << std::endl;
    ROS_INFO_STREAM("Written calibration transform to file: " << file_path);
  }

  // Serialize transformation to store in parameter server
  // std::vector<double> serialized_tran(4 * 4);
  // for (int i = 0; i < )
  // while(ros::ok())
  // publishTF(transformation, "world", "asus_xtion_rgb_optical_frame");
  // std::cout << "Calibration transformation: " << std::endl << transformation << std::endl;

  return 0;
}
