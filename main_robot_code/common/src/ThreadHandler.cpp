#include "common/ThreadHandler.h"
#include <iostream>


ThreadHandler& ThreadHandler::GetInstance()
{
    static ThreadHandler handler;
    return handler;
}


pthread_t ThreadHandler::addThread(std::thread& newThread, bool detach)
{
    pthread_t handle = newThread.native_handle();
    ThreadHandler::GetInstance().createdThreads_.push_back(handle);

    std::cout << "ThreadHandler: added thread, count:" << ThreadHandler::GetInstance().createdThreads_.size() << std::endl;
    if (detach)
        newThread.detach();
    return handle;
}



void ThreadHandler::removeAllThreads()
{
    for (auto t : ThreadHandler::GetInstance().createdThreads_)
    {
        std::cout << "Cancelling: " << t << std::endl;
        pthread_cancel(t);
    }
    ThreadHandler::GetInstance().createdThreads_.clear();
}