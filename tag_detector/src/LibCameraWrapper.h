
#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>

#include <condition_variable>
#include <mutex>
#include <map>

/// @brief Wrapping of LibCamera logic. Exposes the last image as OpenCV image
class LibCameraWrapper {
    public:
        LibCameraWrapper();
        ~LibCameraWrapper();

        void start(int image_width, int image_height);

        cv::Mat waitForNextFrame();
    private:
        void requestCallback(libcamera::Request *request);

        int width_ = 640;
        int height_ = 480;
        cv::Mat lastImage_;
        std::unique_ptr<libcamera::CameraManager> cameraManager_;
        std::shared_ptr<libcamera::Camera> camera_;
        std::shared_ptr<libcamera::FrameBufferAllocator> allocator_;
        std::vector<std::unique_ptr<libcamera::Request>> requests_;

        std::map<int, void *> mappedBuffers_;

        std::mutex mutex_;
        std::condition_variable cv_;
        bool userWantsData_ = false;
};
