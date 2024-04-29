#include "common/solar_panel_camera.hpp"

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <unistd.h>
#include <thread>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------------------------------

SolarPanelCamera::SolarPanelCamera(
  std::string const& camera_name)
: camera_(camera_name),
  dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250)),
  detector_params_(cv::aruco::DetectorParameters::create()),
  distortion_coeffs_(cv::Mat(4,1,CV_64FC1, cv::Scalar::all(0)))
{
    std::thread cameraThread(&SolarPanelCamera::cameraThread, this);
    cameraThread.detach();
}

//--------------------------------------------------------------------------------------------------
// Public functions
//--------------------------------------------------------------------------------------------------
double SolarPanelCamera::getSolarPanelOrientation(bool const& isPlayingRightSide, int const& timeoutMs)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        isPlayingRightSide_ = isPlayingRightSide;
        isPictureReady_ = false;
    }
    cameraTrigger_.notify_all();

    int i = 0;
    while (!isPictureReady_ && i < timeoutMs)
    {
        usleep(1000);
        if (isPictureReady_ && std::isnan(viewedOrientation_))
        {
            isPictureReady_ = false;
            cameraTrigger_.notify_all();
        }
        i++;
    }
    return viewedOrientation_;
}

//--------------------------------------------------------------------------------------------------

void SolarPanelCamera::cameraThread()
{
    pthread_setname_np(pthread_self(), "vision");
    while (true)
    {
        bool isPlayingRightSide = false;
        // Wait for user to ask for picture
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cameraTrigger_.wait(lock);
            isPlayingRightSide = isPlayingRightSide_;
            lock.unlock();
        }
        // Actually take the picture
        viewedOrientation_ = getSolarPanelOrientation_impl(isPlayingRightSide);
        isPictureReady_ = true;
    }
}


double SolarPanelCamera::getSolarPanelOrientation_impl(bool const& isPlayingRightSide)
{
    // Check if camera is working
    double angle_deg = std::nan("");
    if(!this->camera_.isOpened())
    {
        std::cerr << "ERROR: Could not open camera." << std::endl;
        return angle_deg;
    }

    // Get image
    cv::Mat image;
    this->camera_ >> image;

    if (image.empty())
    {
        std::cout << "Warning: empty image" << std::endl;
        return angle_deg;
    }

    // Create the camera matrix
    uint32_t image_width = image.cols;
    uint32_t image_height = image.rows;
    cv::Mat camera_matrix = cv::Mat::eye(3,3,CV_64FC1);
    camera_matrix.at<double>(0,0) = 542;
    camera_matrix.at<double>(1,1) = 476;
    camera_matrix.at<double>(0,2) = double(image_width)/2.0;
    camera_matrix.at<double>(1,2) = double(image_height)/2.0;

    // Detect all the markers
    std::vector<int> detected_marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;
    cv::aruco::detectMarkers(image, this->dictionary_, marker_corners, detected_marker_ids,
    this->detector_params_, rejected_candidates);

    // Return NaN if none or multiple markers are detected
    size_t const num_detected_markers = detected_marker_ids.size();
    if(num_detected_markers != 1)
    {
        std::cerr << "ERROR: Detected none or multiple markers." << std::endl;
        return angle_deg;
    }

    // Skip if not the right marker
    if(detected_marker_ids[0] != 47u)
    return angle_deg;;

    // Estimate and draw the marker's pose
    std::vector<cv::Vec3d> rvecs, tvecs;
    double const marker_length = 5e-2; // [m]
    std::vector<std::vector<cv::Point2f>> markers_corners{marker_corners[0]};
    cv::aruco::estimatePoseSingleMarkers(markers_corners, marker_length, camera_matrix,
    this->distortion_coeffs_, rvecs, tvecs);

    // Get the rotation matrix
    Eigen::Map<Eigen::Vector3d> const rvec((double*) rvecs[0].val);
    double const angle = rvec.norm();
    Eigen::Vector3d const axis = rvec.normalized();
    Eigen::Matrix3d const RCM(Eigen::AngleAxisd(angle,axis));

#if 0
    // Project the camera axis vector into the marker's plane (angle around camera's z axis)
    double const rad2deg = 180. / M_PI;
    Eigen::Vector3d C_u_Mref = isPlayingRightSide ? -1.*RCM.col(1) : 1.*RCM.col(1);
    Eigen::Vector3d C_u_Cz = Eigen::Vector3d::UnitZ();
    C_u_Mref -= (C_u_Mref.dot(C_u_Cz))*C_u_Cz;
    C_u_Mref /= C_u_Mref.norm();
    Eigen::Vector3d C_u_Cx = Eigen::Vector3d::UnitX();
    angle_deg = std::acos(C_u_Cx.dot(C_u_Mref))*rad2deg;
#else
    // Project the camera axis vector into the marker's plane (angle around marker's z axis)
    double const rad2deg = 180. / M_PI;
    Eigen::Vector3d C_u_Mref = isPlayingRightSide ? 1.0*RCM.col(1) : -1.0*RCM.col(1);
    Eigen::Vector3d C_u_Mz = RCM.col(2);
    Eigen::Vector3d C_u_Zref = Eigen::Vector3d::UnitZ();
    C_u_Zref -= (C_u_Zref.dot(C_u_Mz))*C_u_Mz;
    C_u_Zref /= C_u_Zref.norm();
    Eigen::Vector3d C_u_Xref = Eigen::Vector3d::UnitX();
    C_u_Xref = -C_u_Mz.cross(C_u_Zref).normalized();
    double cos_theta = C_u_Mref.dot(C_u_Xref);
    double sin_theta = C_u_Mref.dot(C_u_Zref);
    angle_deg = - std::atan2(sin_theta,cos_theta)*rad2deg;

    // cv::Mat clone = image.clone();
    // cv::drawFrameAxes(clone, camera_matrix, distortion_coeffs_, rvecs[0], tvecs[0], 5e-2);
    // cv::imwrite("test.jpg", clone);

#endif
    return angle_deg;
}

} // namespace vision
