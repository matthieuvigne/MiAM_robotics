#include "LibCameraWrapper.h"

#include <sys/mman.h>
#include <unistd.h>


LibCameraWrapper::LibCameraWrapper():
    cameraManager_(std::make_unique<libcamera::CameraManager>())
{
}

LibCameraWrapper::~LibCameraWrapper()
{
    camera_->stop();
    allocator_.reset();
    camera_->release();
    camera_.reset();
    cameraManager_->stop();
}

void LibCameraWrapper::start(int image_width, int image_height)
{
    width_ = image_width;
    height_ = image_height;
    cameraManager_->start();

    if (cameraManager_->cameras().empty())
    {
        throw std::runtime_error("No camera detected");
    }

    camera_ = cameraManager_->get(cameraManager_->cameras()[0]->id());
    camera_->acquire();

    // Configure camera
    std::unique_ptr<libcamera::CameraConfiguration> config = camera_->generateConfiguration({ libcamera::StreamRole::Viewfinder });
    libcamera::StreamConfiguration &streamConfig = config->at(0);
    streamConfig.pixelFormat = libcamera::formats::RGB888;
    streamConfig.size.width = image_width;
    streamConfig.size.height = image_height;
    streamConfig.bufferCount = 1;
    config->validate();
    camera_->configure(config.get());

    // // Allocate buffers
    allocator_ = std::make_shared<libcamera::FrameBufferAllocator>(camera_);
    allocator_->allocate(streamConfig.stream());

    const auto &buffers = allocator_->buffers(streamConfig.stream());
    requests_.clear();
    for (const auto &buffer : buffers)
    {
        auto request = camera_->createRequest();
        request->addBuffer(streamConfig.stream(), buffer.get());

        int64_t frame_time = 33333;
        request->controls().set(
            libcamera::controls::FrameDurationLimits,
            libcamera::Span<const int64_t, 2>({frame_time, frame_time}));

        for (const libcamera::FrameBuffer::Plane &plane : buffer->planes())
        {
            void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
                        plane.fd.get(), 0);
            mappedBuffers_[plane.fd.get()] = memory;
        }
        requests_.push_back(std::move(request));
    }

    // Callback
    camera_->requestCompleted.connect(this, &LibCameraWrapper::requestCallback);

    camera_->start();
    for (auto &req : requests_)
        camera_->queueRequest(req.get());
}


cv::Mat LibCameraWrapper::waitForNextFrame()
{
    userWantsData_ = true;

    std::unique_lock lk(mutex_);
    cv_.wait(lk);

    return lastImage_;
}

void LibCameraWrapper::requestCallback(libcamera::Request *request)
{
    if (request->status() == libcamera::Request::RequestCancelled)
        return;

    if (userWantsData_)
    {
        for (auto &pair : request->buffers())
        {
            std::unique_lock lk(mutex_);
            libcamera::FrameBuffer *buffer = pair.second;
            lastImage_ = cv::Mat(height_, width_, CV_8UC3, mappedBuffers_[buffer->planes()[0].fd.get()]).clone();
            userWantsData_ = false;
            cv_.notify_one();
            break;
        }
    }
    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
}