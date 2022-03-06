#include <vision/module.hpp>

namespace vision {
  
//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

Module::Module(std::string const& parameters_filename)
{
  // Build the board from the parameters
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void Module::boardThread()
{
  // TODO
}

//--------------------------------------------------------------------------------------------------

void Module::cameraThread(size_t camera_idx)
{
  // Get the associated camera loop
  if(camera_idx >= this->camera_threads_.size())
    throw std::out_of_range("Inconsistent camera index.");
  CameraThread& camera_thread = this->camera_threads_[camera_idx];

  // Main loop
  while(true)
  {
    // Wait condition (unbusy waiting)
    std::unique_lock<std::mutex> camera_locker(camera_thread.mtx);
    camera_thread.con.wait(camera_locker,
      [&](){ return camera_thread.image != nullptr; });
    vision_mgs::Image::UniquePtr image_msg = std::move(camera_thread.image);
    camera_thread.mtx.unlock();

    // Process the image
    // TODO
  }
}

//--------------------------------------------------------------------------------------------------

} // namespace vision
