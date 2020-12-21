#ifndef APRILTAG_ROS_DETECTOR_H
#define APRILTAG_ROS_DETECTOR_H


// system
#include <iostream>
#include <vector>

// opencv
#include <opencv2/opencv.hpp>

// eigen
#include <eigen3/Eigen/Geometry>

// apriltag
#include <apriltag.h>
#include <tag36h11.h>

// cv_tools
#include "cv_tools/geometry/pinhole_camera.hpp"

#include <chrono>


namespace apriltag_ros
{
struct Tag
{
  float size;
  float border;
  unsigned int id;
  cv::Point2f vertices[4];
};

/**
 * @brief A model for a Pinhole Camera which receives the intrinsics as input
 * 
 * More info here: https://en.wikipedia.org/wiki/Pinhole_camera_model
 *
 */
class PinholeCamera
{
public:
  PinholeCamera(double fx, double fy, double cx, double cy, double size_x, double size_y);

  /**
   * @brief Projects a point on the image plane to 3-D given the depth value on
   *        the pixel and the intrinsics of the camera
   *
   * @param point A 2-D point on the image plane
   * @param depth_value The depth value for this 2-D point
   * 
   * @returns The 3-D point with respect to the camera's frame
   */
  Eigen::Vector3f backProject(const Eigen::Vector2f& point, float depth_value) const;
  
  Eigen::Vector3f project(const Eigen::Vector3f& point_3d) const;

  /**
   * @brief Returns camera matrix.
   *        
   *
   * @returns The rotation matrix: 
   * 
   * \f[
   *  \begin{bmatrix}
   *   f_x &   0 & c_x \\
   *    0  & f_y & c_y \\
   *    0  &   0 &  1 
   *  \end{bmatrix}
   * \f]
   */
  Eigen::Matrix3f matrix() const;

  /**
   * @brief Focal length in x-axis (millimeters)
   */
  double fx;

  /**
   * @brief Focal length in y-axis (millimeters)
   */
  double fy;
  
  /**
   * @brief The x-dimension (along height) of the principal point (in pixels)
   */
  double cx;

  /**
   * @brief The y-dimension (along width) of the principal point (in pixels)
   */
  double cy;

  /**
   * @brief The height of the image (in pixels)
   */
  double size_x;

  /**
   * @brief The witdh of the image (in pixels)
   */
  double size_y;
};

/**
 * \brief Wrapper class of Apriltag for detecting a tag
 * 
 * \param params The apriltag parameters
 * \param refine_pose Parameter for better accuracy with the cost of
 * performance. Set to 0 for fast detection with low accuracy and to 1 for slow
 * detection with high accuracy
 */
class AprilTagDetector
{
public:
    AprilTagDetector(const cv::Size& board_size, float tag_size, float tag_border, int refine_pose = 0);

    /**
     * \brief Detects the number of tags in an RGB image.
     * 
     * \param img The RGB image
     * \returns A vector of detected tags
     */
    std::vector<Tag> detect(const cv::Mat &img) const;

    /**
     * \brief Detects the number of tags in an RGB image, but it returns the points serialized.
     * 
     * \param img The RGB image
     * \param plot Plot the points on image
     * \returns A vector of the detected vertices.
     */
    std::vector<Eigen::Vector2f> getTagPoints2D(const cv::Mat &rgb, bool plot) const;

    /**
     * \brief Returns the points in 3D.
     * 
     * \param img The RGB image
     * \param plot Plot the points on image
     * \returns A vector of the detected vertices.
     */
    std::vector<Eigen::Vector3f> getTagPoints3D(const cv::Mat &rgb,
                                                const cv::Mat &depth,
                                                const PinholeCamera& camera,
                                                std::vector<Eigen::Vector2f>& points2d,
                                                bool plot=false) const;

    std::vector<Eigen::Affine3f> getPose(const cv::Mat &rgb,
                                         const cv::Mat &depth,
                                         const PinholeCamera& camera,
                                         std::vector<Eigen::Vector2f>& points2d,
                                         std::vector<Eigen::Vector3f>& points3d,
                                         bool plot=false) const;

    bool findPoints(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                    const cv::Mat &img);

    void drawPoints(cv::Mat &img, std::vector<cv::Point2f> &pts);

private:
    apriltag_detector* td_;
    cv::Size board_size; 
    float tag_size;
    float tag_border;
};


class ChessBoardDetector{
public:
    struct ChessBoardParamaters
    {
        cv::Size board_size_;
        double square_size_;

        ChessBoardParamaters()
        {
            board_size_ = cv::Size(9, 6);
            square_size_ = 0.025;   //2.5cm
        }
    };

    ChessBoardDetector(){}

    bool findPoints(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                    const cv::Mat &img, const ChessBoardParamaters &params);
};



class PoseEstimator{

public:
    PoseEstimator(){}

    Eigen::Affine3d estimate(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                             const cv_tools::PinholeCamera &camera);

    void drawAxes(cv::Mat &img, cv_tools::PinholeCamera &camera, cv::Point2f &corner);

private:
    cv::Mat r_vec_, t_vec_;

};

class Timer
{
public:
  Timer(const std::string& name)
    : creation(std::chrono::steady_clock::now())
    , name(name)
  {
  }

  double existenceTime(bool print=true)
  {
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> execution_time = end - creation;  // in ms
    double ms = execution_time.count();
    if (print)
      std::cout << "Timer: " << name << "took: " <<  ms << " ms" << std::endl;
    return ms;
  }

private:
  std::chrono::steady_clock::time_point creation;
  std::string name;
};


}


#endif  // APRILTAG_ROS_DETECTOR_H
