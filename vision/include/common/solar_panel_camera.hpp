#ifndef VISION_COMMON_GET_SOLAR_PANEL_ORIENTATION_HPP
#define VISION_COMMON_GET_SOLAR_PANEL_ORIENTATION_HPP

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <mutex>
#include <condition_variable>

namespace vision {

class SolarPanelCamera {

    public:
        SolarPanelCamera(std::string const& camera_name);

        double getSolarPanelOrientation(bool const& isPlayingRightSide, int const& timeoutMs = 100);

    private:

        // Actual function taking picture and computing solar panel angle.
        double getSolarPanelOrientation_impl(bool const& isPlayingRightSide);

        // Background thread actually talking to the camera.
        void cameraThread();

        std::mutex mutex_;
        std::condition_variable cameraTrigger_;

        double viewedOrientation_;
        bool isPlayingRightSide_;
        bool isPictureReady_;


        cv::VideoCapture camera_;
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
        cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
        cv::Mat distortion_coeffs_;

}; // class SolarPanelCamera

} // namespace vision

#endif
