#ifndef CAMERA_MESSAGES_HPP
#define CAMERA_MESSAGES_HPP

#include <opencv2/opencv.hpp>

namespace vision_mgs {

struct Image {
  POINTER_TYPEDEF(Image);
  size_t camera_idx;
  int64_t timestamp_ns;
  cv::Ptr<cv::Mat> image_ptr;
}; // struct Image

} // namespace camera_mgs

#endif // CAMERA_MESSAGES_HPP
