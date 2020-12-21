#include <apriltag_ros/detector.h>
#include <chrono>


namespace apriltag_ros
{
AprilTagDetector::AprilTagDetector(const cv::Size& board_size, float tag_size, float tag_border, int refine_pose)
    : board_size(board_size)
    , tag_size(tag_size)
    , tag_border(tag_border)
{
    apriltag_family_t* tf = tag36h11_create();

    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf);

    td_->debug = 0;
    td_->refine_pose = refine_pose;
    td_->nthreads = 4;
}


std::vector<Tag> AprilTagDetector::detect(const cv::Mat &img) const
{

    using clock = std::chrono::steady_clock;
    clock::time_point start = clock::now();

    // Conver to grayscale
    cv::Mat gray;
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
    std::chrono::duration<double, std::milli> execution_time = end - start;  // in ms
    // std::cout << "find points took " << execution_time.count() << " ms" << std::endl;


    std::vector<Tag> results;
    for(int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        Tag result;
        result.id = det->id;
        result.size = this->tag_size;
        result.border = this->tag_border;
        for(int j = 0; j < 4; j++)
        {
            result.vertices[j].x = det->p[j][0];
            result.vertices[j].y = det->p[j][1];
        }
        results.push_back(result);

        apriltag_detection_destroy(det);
    }

    zarray_destroy(detections);
    image_u8_destroy(im_u8);

    return results;
}


std::vector<Eigen::Vector2f> AprilTagDetector::getTagPoints2D(const cv::Mat &rgb, bool plot) const
{
  std::vector<Tag> results = this->detect(rgb);

  if (results.size() == 0)
  {
    throw std::runtime_error("Apriltag: Cannot detect tags!");
  }

  std::vector<Eigen::Vector2f> tag_points(results.size() * 4);   

  for (int i = 0; i < results.size(); i++)
  {
    for (int j = 0; j < 4; j++)
    {
      tag_points.at(i * 4 + j)[0] = results.at(i).vertices[j].x;
      tag_points.at(i * 4 + j)[1] = results.at(i).vertices[j].y;
    }
  }

  if (plot)
  {
    cv::Mat img = rgb;
    cv::Point p;
    for (int i = 0; i < results.size(); i++)
    {
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

std::vector<Eigen::Vector3f> AprilTagDetector::getTagPoints3D(const cv::Mat &rgb,
                                                              const cv::Mat &depth,
                                                              const PinholeCamera& camera,
                                                              std::vector<Eigen::Vector2f>& points2d,
                                                              bool plot) const
{
    points2d = getTagPoints2D(rgb, plot);

    std::vector<Eigen::Vector3f> tag_points(points2d.size());
    for(int i = 0; i < points2d.size(); i++)
    {
      tag_points.at(i) = camera.backProject(points2d.at(i), (float)depth.at<unsigned short>(points2d.at(i)[1], points2d.at(i)[0]));
    }
    return tag_points;
}

std::vector<Eigen::Affine3f> AprilTagDetector::getPose(const cv::Mat &rgb,
                                                       const cv::Mat &depth,
                                                       const PinholeCamera& camera,
                                                       std::vector<Eigen::Vector2f>& points2d,
                                                       std::vector<Eigen::Vector3f>& points3d,
                                                       bool plot) const
{
  points3d = getTagPoints3D(rgb, depth, camera, points2d, plot);
  int n_tags = points3d.size() / 4;
  std::vector<Eigen::Affine3f> poses;
  for (int i = 0; i < n_tags; i++)
  {
    Eigen::Affine3f pose;
    int index = i * n_tags;
    pose.translation() = points3d.at(index);
    pose.linear().col(0) = (points3d.at(index + 1) - points3d.at(index)) /
                           (points3d.at(index + 1) - points3d.at(index)).norm();
    pose.linear().col(1) = (points3d.at(index + 3) - points3d.at(index)) /
                           (points3d.at(index + 3) - points3d.at(index)).norm();
    pose.linear().col(2) = pose.linear().col(0).cross(pose.linear().col(1));
    poses.push_back(pose);
  }
  return poses;
}


bool AprilTagDetector::findPoints(std::vector<cv::Point2f> &pts,
                                  std::vector<cv::Point3f> &objs,
                                  const cv::Mat &img)
{
    std::vector<Tag> tags = this->detect(img);

    if(tags.size() == 0)
    {
        return false;
    }

    for(int i = 0; i < tags.size(); i++)
    {
        //!<store 2d points
        for(int j = 0; j < 4; j++)
        {
            pts.push_back(tags.at(i).vertices[j]);
        }

        //!< store 3d points
        int row = tags.at(i).id / this->board_size.width;
        int col = tags.at(i).id % this->board_size.width;

        float x = col * (this->tag_size + this->tag_border);
        float y = row * (this->tag_size + this->tag_border);

        //!< counter-clockwise winding order
        objs.push_back(cv::Point3f(x, y, 0.0));
        objs.push_back(cv::Point3f(x + this->tag_size, y, 0.0));
        objs.push_back(cv::Point3f(x + this->tag_size, y + this->tag_size, 0.0));
        objs.push_back(cv::Point3f(x , y + this->tag_size, 0.0));
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


PinholeCamera::PinholeCamera(double fx, double fy, double cx, double cy, double size_x, double size_y)
  : fx(fx)
  , fy(fy)
  , cx(cx)
  , cy(cy)
  , size_x(size_x)
  , size_y(size_y)
{
}

Eigen::Vector3f PinholeCamera::project(const Eigen::Vector3f& point_3d) const
{
  Eigen::Vector3f point;
  point[0] = point_3d[0] * this->fx / point_3d[2] + this->cx;
  point[1] = point_3d[1] * this->fy / point_3d[2] + this->cy;
  point[2] = point_3d[2] * 1000.0f;
  return point;
}

Eigen::Vector3f PinholeCamera::backProject(const Eigen::Vector2f& point, float depth_value) const
{
  Eigen::Vector3f point_3d;
  point_3d[2] = depth_value / 1000.0f;
  point_3d[0] = (point[0] - this->cx) * point_3d[2] / this->fx ;
  point_3d[1] = (point[1] - this->cy) * point_3d[2] / this->fy ;
  return point_3d;
}

Eigen::Matrix3f PinholeCamera::matrix() const
{
  Eigen::Matrix3f cam_matrix;
  cam_matrix.setZero();
  cam_matrix(0, 0) = this->fx;
  cam_matrix(1, 1) = this->fy;
  cam_matrix(0, 2) = this->cx;
  cam_matrix(1, 2) = this->cy;
  cam_matrix(2, 2) = 1.0;
  return cam_matrix;
}
}
