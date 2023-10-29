#include "common/ThreadHandler.h"
#include <iostream>

/**
 * Static methods should be defined outside the class.
 */

ThreadHandler* ThreadHandler::pinstance_{nullptr};
std::mutex ThreadHandler::mutex_;

/**
 * The first time we call GetInstance we will lock the storage location
 *      and then we make sure again that the variable is null and then we
 *      set the value. RU:
 */
ThreadHandler *ThreadHandler::GetInstance()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (pinstance_ == nullptr)
    {
        pinstance_ = new ThreadHandler();
    }
    return pinstance_;
}


void ThreadHandler::addThread(std::thread& newThread, bool detach)
{
    pthread_t handle = newThread.native_handle();
    ThreadHandler::GetInstance()->createdThreads_.push_back(handle);

    if (detach)
        newThread.detach();
}



void ThreadHandler::removeAllThreads()
{
    for (auto t : ThreadHandler::GetInstance()->createdThreads_)
    {
        std::cout << "Cancelling: " << t << std::endl;
        pthread_cancel(t);
    }
    ThreadHandler::GetInstance()->createdThreads_.clear();
}