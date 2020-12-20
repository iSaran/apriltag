#include <apriltag_ros/detector.h>
#include <chrono>


namespace apriltag_ros
{
AprilTagParameters::AprilTagParameters(const std::vector<int>& size, float tag_size, float tag_border)
{
    board_size_ = cv::Size(size.at(0), size.at(1));
    tag_size_ = tag_size;
    tag_border_ = tag_border;
}

AprilTagDetector::AprilTagDetector(const AprilTagParameters& params, int refine_pose)
    : params(params)
{
    apriltag_family_t* tf = tag36h11_create();

    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf);

    td_->debug = 0;
    td_->refine_pose = refine_pose;
    td_->nthreads = 4;
}


void AprilTagDetector::detect(const cv::Mat &img, std::vector<AprilTagDetector::Result> &results)
{

    using clock = std::chrono::steady_clock;
    clock::time_point start = clock::now();

    cv::Mat gray;
    //!< convert to gray scale
    if(img.type() == CV_8UC1) gray = img;
    else cv::cvtColor(img, gray, CV_BGR2GRAY);




    //!< convert to uint8
    //!< https://msdn.microsoft.com/en-us/library/windows/desktop/aa473780%28v=vs.85%29.aspx
    image_u8_t *im_u8 = image_u8_create(img.cols, img.rows);
    uint8_t *dst = im_u8->buf;

    for(int y = 0; y < im_u8->height; y++)
    {
        memcpy(dst, gray.ptr(y), im_u8->width);
        dst = dst + im_u8->stride;
    }

    //!< detect tags from an image
    zarray_t* detections = apriltag_detector_detect(td_, im_u8);
    
    clock::time_point end = clock::now();
    // std::chrono::duration<double> execution_time = end - start;  // in seconds
    std::chrono::duration<double, std::milli> execution_time = end - start;  // in ms
    double k = execution_time.count();
    std::cout << "find points took " << k << " ms" << std::endl;


    for(int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        Result res;
        res.id = det->id;
        for(int j = 0; j < 4; j++)
        {
            res.pts[j].x = det->p[j][0];
            res.pts[j].y = det->p[j][1];
        }
        results.push_back(res);

        apriltag_detection_destroy(det);
    }

    zarray_destroy(detections);
    image_u8_destroy(im_u8);
}


bool AprilTagDetector::findPoints(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                                  const cv::Mat &img)
{
    std::vector<AprilTagDetector::Result> results;
    detect(img,results);

    if(results.size() == 0)
    {
        return false;
    }

    for(int i = 0; i < results.size(); i++)
    {
        Result &res = results[i];

        //!<store 2d points
        for(int j = 0; j < 4; j++)
        {
            pts.push_back(res.pts[j]);
        }

        //!< store 3d points
        int row = res.id / this->params.board_size_.width;
        int col = res.id % this->params.board_size_.width;

        float x = col * (this->params.tag_size_ + this->params.tag_border_);
        float y = row * (this->params.tag_size_ + this->params.tag_border_);

        //!< counter-clockwise winding order
        objs.push_back(cv::Point3f(x, y, 0.0));
        objs.push_back(cv::Point3f(x + this->params.tag_size_, y, 0.0));
        objs.push_back(cv::Point3f(x + this->params.tag_size_ , y + this->params.tag_size_, 0.0));
        objs.push_back(cv::Point3f(x , y + this->params.tag_size_, 0.0));
    }

    // drawPoints(img, pts);

    return true;
}



void AprilTagDetector::drawPoints(cv::Mat &img, std::vector<cv::Point2f> &pts)
{
  cv::circle(img, pts[0], 2, cv::Scalar(0, 0, 255), 3);
  cv::circle(img, pts[1], 2, cv::Scalar(0, 0, 255), 3);
  cv::circle(img, pts[2], 2, cv::Scalar(0, 0, 255), 3);
  cv::circle(img, pts[3], 2, cv::Scalar(0, 0, 255), 3);

  cv::imshow("img", img);
  cv::waitKey();
}


bool ChessBoardDetector::findPoints(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                                    const cv::Mat &img, const ChessBoardParamaters &params)
{
    cv::Mat gray, tmpMat;
    tmpMat = img;

    cv::cvtColor(tmpMat, gray, CV_BGR2GRAY);

    bool found = cv::findChessboardCorners(gray, params.board_size_, pts,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

    if (found == 0 && (pts.size() != 0))
    {
        cv::cornerSubPix(gray, pts, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }

    //cv::drawChessboardCorners(tmpMat, params.board_size_, cv::Mat(corners_),found);

    for(int i = 0; i < params.board_size_.height; i++)
    {
        for(int j = 0; j < params.board_size_.width; j++)
        {
            objs.push_back(cv::Point3f(float(i * params.square_size_), float(j * params.square_size_), 0));
        }
    }

    return found;
}


Eigen::Affine3d PoseEstimator::estimate(std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs,
                                        const cv_tools::PinholeCamera &camera)
{
    r_vec_ = cv::Mat(cv::Size(3,1), CV_64F);
    t_vec_ = cv::Mat(cv::Size(3,1), CV_64F);

    cv::solvePnPRansac(cv::Mat(objs), cv::Mat(pts), camera.getMatrix(),
                       camera.getDistortionCoeffs(), r_vec_, t_vec_);

    cv::Mat R = cv::Mat(cv::Size(3,3), CV_64F);
    cv::Rodrigues(r_vec_, R);

    Eigen::Matrix3d r;
    Eigen::Vector3d t;
    for(int i = 0; i < 3 ; i++)
    {
        for(int j = 0; j < 3 ; j++)
        {
            r(i,j) = R.at<double>(i,j);
        }
        t(i) = t_vec_.at<double>(0,i);
    }
    Eigen::Affine3d tr = Eigen::Translation3d(t) * r;

    return tr;
}


void PoseEstimator::drawAxes(cv::Mat &img, cv_tools::PinholeCamera &camera, cv::Point2f &corner)
{
  std::vector<cv::Point3f> axes;
  // axes.push_back(cv::Point3f(.0, 0.05, 0.0));
  // axes.push_back(cv::Point3f(0.05, .0, 0.0));
  // axes.push_back(cv::Point3f(0.0, 0.0, -.05));

  axes.push_back(cv::Point3f(.05, 0.0, 0.0));
  axes.push_back(cv::Point3f(0.0, .05, 0.0));
  axes.push_back(cv::Point3f(0.0, 0.0, .05));

  std::vector<cv::Point2f> projected_points;
  cv::projectPoints(axes, r_vec_, t_vec_, camera.getMatrix(), camera.getDistortionCoeffs(), projected_points);
  cv::arrowedLine(img, corner, projected_points[0], cv::Scalar(0,0,255), 8);
  cv::arrowedLine(img, corner, projected_points[1], cv::Scalar(0,255,0), 8);
  cv::arrowedLine(img, corner, projected_points[2], cv::Scalar(255,0,0), 8);
  cv::imshow("rgb", img);
  cv::waitKey();
}
}
