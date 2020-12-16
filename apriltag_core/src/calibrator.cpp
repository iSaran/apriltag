#include <apriltag_core/calibrator.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

namespace apriltag_core
{
Calibrator::Calibrator(const AprilTagParameters& tag_params, double fx, double fy, double cx, double cy, double nmarkers)
    : tag_detector(tag_params)
    , fx_(fx)
    , fy_(fy)
    , cx_(cx)
    , cy_(cy)
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

Eigen::Vector3f Calibrator::backproject(const Eigen::Vector2f& img_point, const cv::Mat& depth)
{
    Eigen::Vector3f point_3d;
    point_3d[2] = (float)depth.at<unsigned short>(img_point[1], img_point[0]) / 1000.0f;
    point_3d[0] = (img_point[0] - cx_) / fx_ * point_3d[2];
    point_3d[1] = (img_point[1] - cy_) / fy_ * point_3d[2];
    return point_3d;
}

std::vector<Eigen::Vector2f> Calibrator::getTagPoints2D(const cv::Mat &rgb, bool plot)
{

    std::vector<AprilTagDetector::Result> results;
    tag_detector.detect(rgb, results);

    std::vector<Eigen::Vector2f> tag_points(results.size() * 4);   
    std::cout << "Results size: " << results.size() << std::endl;

    for (int i = 0; i < results.size(); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            tag_points.at(i * 4 + j)[0] = results.at(i).pts[j].x;
            tag_points.at(i * 4 + j)[1] = results.at(i).pts[j].y;
        }
    }



    if (plot)
    {
        cv::Mat img = rgb;
        cv::Point p;
        for (int i = 0; i < results.size(); i++)
        {
            std::cout << "get 2ddddddddddd" << std::endl << tag_points.at(i).transpose() << std::endl;
            p.x = tag_points.at(i * 4 + 0)[0]; p.y = tag_points.at(i * 4 + 0)[1];
            cv::circle(img, p, 5, cv::Scalar(0, 0, 255), 3);
            p.x = tag_points.at(i * 4 + 1)[0]; p.y = tag_points.at(i * 4 + 1)[1];
            cv::circle(img, p, 5, cv::Scalar(0, 255, 0), 3);
            p.x = tag_points.at(i * 4 + 2)[0]; p.y = tag_points.at(i * 4 + 2)[1];
            cv::circle(img, p, 5, cv::Scalar(255, 0, 0), 3);
            p.x = tag_points.at(i * 4 + 3)[0]; p.y = tag_points.at(i * 4 + 3)[1];
            cv::circle(img, p, 5, cv::Scalar(0, 0, 0), 3);
        }
        cv::imshow("rgb", img);
        cv::waitKey();
    }

    return tag_points;
}

std::vector<Eigen::Vector3f> Calibrator::getTagPoints(const cv::Mat &rgb, const cv::Mat &depth, bool plot)
{
    auto img_points = getTagPoints2D(rgb, plot);

    std::vector<Eigen::Vector3f> tag_points(nmarkers_ * 4);
    for(int i=0; i<nmarkers_ * 4; i++)
    {
        tag_points.at(i) = this->backproject(img_points.at(i), depth);
    }

    return tag_points;
}

Eigen::Affine3f Calibrator::run(const cv::Mat &rgb, const cv::Mat &depth, const std::vector<Eigen::Vector3f> &robot_points, bool plot)
{
    // AprilTags::TagDetector *tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);

    cv::Mat image_gray;
    cv::cvtColor(rgb, image_gray, CV_BGR2GRAY);


    cv::imshow("rgb", image_gray);
    cv::waitKey();






    // for(int i = 0; i < results.size(); i++)
    // {
    //     AprilTagDetector::Result &res = results[i];

    //     //!<store 2d points
    //     for(int j = 0; j < 4; j++)
    //     {
    //         std::cout << "res::::::::::" << res.pts[0] << std::endl;
    //         pts.push_back(res.pts[j]);
    //     }

    //     //!< store 3d points
    //     int row = res.id / this->params.board_size_.width;
    //     int col = res.id % this->params.board_size_.width;

    //     float x = col * (this->params.tag_size_ + this->params.tag_border_);
    //     float y = row * (this->params.tag_size_ + this->params.tag_border_);

    //     //!< counter-clockwise winding order
    //     objs.push_back(cv::Point3f(x, y, 0.0));
    //     objs.push_back(cv::Point3f(x + this->params.tag_size_, y, 0.0));
    //     objs.push_back(cv::Point3f(x + this->params.tag_size_ , y + this->params.tag_size_, 0.0));
    //     objs.push_back(cv::Point3f(x , y + this->params.tag_size_, 0.0));
    // }

    // vector<AprilTags::TagDetection> detections = tagDetector->extractTags(image_gray);

    // cv::Mat rgb_tags;
    // rgb.copyTo(rgb_tags);
    // if(detections.size() > 0){
    //     for(int i=0; i<detections.size(); ++i){
    //         for(int j=0; j<4; ++j)
    //             cv::circle(rgb_tags, cv::Point(detections[i].p[j].first, detections[i].p[j].second), 4, cv::Scalar(0,0,255), 2);
    //         detections[i].draw(rgb_tags);
    //     }
    // }

//     if(detections.size() < nmarkers_) {
//         std::cout << "Only " << detections.size() << " of " << nmarkers_ << " markers detected" << std::endl;
//         return;
//     }

//     std::cout << "Press s to stop or any key to continue" << std::endl;
//     while(true){
		
//         cv::imshow("tags", rgb_tags);
//         cv::imwrite("tags.png", rgb_tags);
//         cv::imwrite("rgb.png", rgb);
//         int k = cv::waitKey(30);
//         if( (char)k == 's')
//             return;

//         if( k > 0)
//             break;
//     }
//     std::sort(detections.begin(), detections.end(), comparator);


//     std::vector<Eigen::Vector3f> tag_points(nmarkers_ * 4);
//     for(int i=0; i<nmarkers_; ++i){
// //        if(detections[i].id != i){
// //            std::cout << "Wrong id recognized..." << std::endl;
// //            return;
// //        }
//         for(int j=0; j<4; ++j){
//             int imgx = detections[i].p[j].first;
//             int imgy = detections[i].p[j].second;
//             float Z = (float)depth.at<unsigned short>(imgy, imgx) / 1000.0f;
//             float X = (imgx - cx_) / fx_ * Z;
//             float Y = (imgy - cy_) / fy_ * Z;
//             tag_points[i*4 + j] << X, Y, Z;
//         }
//     }


    std::vector<Eigen::Vector3f> tag_points = getTagPoints(rgb, depth, plot);
    Eigen::Matrix4f trans = doCalibration(robot_points, tag_points);
    std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
    std::cout << trans << std::endl;
    std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
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