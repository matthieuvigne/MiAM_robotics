#ifndef VISION_COMMON_GET_SOLAR_PANEL_ORIENTATION_HPP
#define VISION_COMMON_GET_SOLAR_PANEL_ORIENTATION_HPP

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace vision {

class SolarPanelCamera {

  public:
    SolarPanelCamera() = delete;
    SolarPanelCamera(
      std::string const& camera_name);

  public:
    double getSolarPanelOrientation(bool const& isPlayingRightSide);

  private:
    cv::VideoCapture camera_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat distortion_coeffs_;

}; // class SolarPanelCamera

} // namespace vision

#endif
