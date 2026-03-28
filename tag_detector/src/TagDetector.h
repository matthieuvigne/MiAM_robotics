#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "VisionSharedMem.h"

typedef std::vector<Marker> MarkerList;


struct Tag
{
    int markerId;
    Eigen::Affine3d TRM;
};


/// @brief Detect Aruco tags in image
class TagDetector
{
  public:
    TagDetector(int width, int height, double fx, double fy, double cx, double cy);

  public:

    // Getters
    inline double fx() const { return fx_; }
    inline double fy() const { return fy_; }
    inline double cx() const { return cx_; }
    inline double cy() const { return cy_; }
    inline int width() const { return width_; }
    inline int height() const { return height_; }
    inline Eigen::Affine3d TRC() const { return TRC_; }

    // Setters
    inline double& fx() { return fx_; }
    inline double& fy() { return fy_; }
    inline double& cx() { return cx_; }
    inline double& cy() { return cy_; }
    inline Eigen::Affine3d& TRC() { return TRC_; }

    // Detect and return the markers of interest found in the image.
    MarkerList find_markers(cv::Mat const& img);

  protected:

    // Get the Camera2 matrix
    cv::Mat get_camera_matrix() const;

    // Detect the markers within the image
    bool take_picture(
      cv::Mat* image,
      double timeout = 1.0);
    int detect_markers(
      cv::Mat const& img,
      std::vector<Tag>* TRM_ptr) const;

  protected:

    // Extrinsic parameters
    Eigen::Affine3d TRC_ = Eigen::Affine3d::Identity();

    // Intrinsic parameters
    double fx_ = NAN;     // x axis focal length [px/m]
    double fy_ = NAN;     // y axis focal length [px/m]
    double cx_ = NAN;     // x axis optical center coordinate [px
    double cy_ = NAN;     // y axis optical center coordinate [px]

    // Image resolution
    int width_;           // image width [px]
    int height_;          // image height [px]

    // Marker detection
    typedef cv::Ptr<cv::aruco::DetectorParameters> CvParamsPtr;
    typedef cv::Ptr<cv::aruco::Dictionary> CvDicoPtr;
    CvParamsPtr cv_params_ptr_;
    CvDicoPtr cv_dico_ptr_;
};

