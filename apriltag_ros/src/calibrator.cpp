#include <apriltag_ros/calibrator.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

namespace apriltag_ros
{
Calibrator::Calibrator(const cv::Size& board_size, float tag_size, float tag_border, const PinholeCamera& camera, double nmarkers)
    : tag_detector(board_size, tag_size, tag_border, 1)
    , camera(camera)
    , nmarkers_(nmarkers)
{
}

Calibrator::Calibrator(const cv::Size& board_size, float tag_size, float tag_border, 
                       float fx, float fy, float cx, float cy, unsigned int size_x, unsigned int size_y,
                       double nmarkers)
    : tag_detector(board_size, tag_size, tag_border, 1)
    , camera(fx, fy, cx, cy, size_x, size_y)
    , nmarkers_(nmarkers)
{
}

Eigen::Matrix4f Calibrator::doCalibration(const std::vector<Eigen::Vector3f> &robot_points,
                                          const std::vector<Eigen::Vector3f> &tag_points)
{
    cv::Mat meanRobot = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));

    int n = robot_points.size();

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            meanRobot.at<float>(j, 0) += (robot_points[i](j) / (float)n);
        }
    }    

    cv::Mat xtionPoints(n, 3, CV_32FC1, cv::Scalar::all(0));

    cv::Mat meanXtion = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));

    for (unsigned int i=0;i<n;i++)
    {
        xtionPoints.at<float>(i, 0) = tag_points[i](0) ;
        xtionPoints.at<float>(i, 1) = tag_points[i](1) ;
        xtionPoints.at<float>(i, 2) = tag_points[i](2) ;

        meanXtion.at<float>(0, 0) += tag_points[i](0)/n;
        meanXtion.at<float>(1, 0) += tag_points[i](1)/n;
        meanXtion.at<float>(2, 0) += tag_points[i](2)/n;
    }   


    cv::Mat H = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    cv::Mat Pa = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));
    cv::Mat Pb = cv::Mat(1, 3, CV_32FC1, cv::Scalar::all(0));

    for(int i=0; i<n; ++i)
    {
        for(int j=0; j<3; ++j)
        {
            Pa.at<float>(j, 0) = xtionPoints.at<float>(i, j) - meanXtion.at<float>(j, 0);
            Pb.at<float>(0, j) = robot_points[i](j) - meanRobot.at<float>(j, 0);
        }
        H += Pa * Pb;
    }
    // std::cout << "H: " << H << std::endl;

    cv::SVD svd(H, cv::SVD::FULL_UV);
    cv::Mat tr(4, 4, CV_32FC1, cv::Scalar::all(0)) ;
    cv::Mat V = svd.vt.t();
    double det = cv::determinant(V);

    if(det < 0){
        for(int i=0; i<V.rows; ++i)
            V.at<float>(i,3) *= -1;
    }

    cv::Mat R = V * svd.u.t();
    // std::cout << "R: " << R << std::endl;

    cv::Mat t = (-1)*R*meanXtion + meanRobot;

    Eigen::Matrix4f res;
    for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
            res(i, j) = R.at<float>(i, j);
    for(int i=0; i<3; ++i){
        res(3, i) = 0;
        res(i, 3) = t.at<float>(i);
    }
    res(3,3) = 1;

    return res;
}

Eigen::Affine3f Calibrator::run(const cv::Mat &rgb, const cv::Mat &depth, const std::vector<Eigen::Vector3f> &robot_points, bool plot)
{
    // AprilTags::TagDetector *tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);

    cv::Mat image_gray;
    cv::cvtColor(rgb, image_gray, CV_BGR2GRAY);

    std::vector<Eigen::Vector2f> points2d;
    std::vector<Eigen::Vector3f> tag_points = tag_detector.getTagPoints3D(rgb, depth, camera, points2d, plot);
    Eigen::Matrix4f trans = doCalibration(robot_points, tag_points);
    if(trans(2,3) < 0)
        for(int i=0; i<4; ++i)
            trans(2, i) *= -1;
    
    Eigen::Affine3f result;
    result.linear() = trans.block(0, 0, 3, 3);
    result.translation() = trans.block(0, 3, 3, 1);
    if (plot)
    {
        std::cout << "Transformation found by calibration:" << std::endl << result.matrix() << std::endl;
    }
    return result;

}
}  // namespace apriltag_detector