#ifndef NETWORK_SERVER_THREAD_HPP
#define NETWORK_SERVER_THREAD_HPP

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <assert.h>

#include <miam_utils/network/macros.hpp>
#include <miam_utils/network/server_socket.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class definition
//--------------------------------------------------------------------------------------------------

class ServerThread : public ServerSocket {

public:

  typedef std::unique_ptr<char*> Message;
  POINTER_TYPEDEF(ServerThread);
  ServerThread(int port);
  virtual ~ServerThread();

public:

  inline void join();
  // inline bool setCameraThread(camera::CameraThread const* camera_thread_ptr);

private:

  void serverThread();

public:

  // Threading
  mutable std::mutex mtx_;
  std::unique_ptr<std::thread> thread_ptr_;
  // camera::CameraThread const* camera_thread_ptr_;

}; // class ServerThread

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

void ServerThread::join()
{
  thread_ptr_->join();
}

// //--------------------------------------------------------------------------------------------------

// bool ServerThread::setCameraThread(camera::CameraThread const* camera_thread_ptr)
// {
//   if(camera_thread_ptr == nullptr) return false;
//   camera_thread_ptr_ = camera_thread_ptr;
//   return true;
// }

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_SERVER_THREAD_HPP
