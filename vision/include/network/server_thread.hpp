#ifndef NETWORK_SERVER_THREAD_HPP
#define NETWORK_SERVER_THREAD_HPP

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <assert.h>

#include <common/macros.hpp>
#include <network/server_socket.hpp>

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

private:

  void serverThread();

public:

  // Threading
  mutable std::mutex mtx_;
  std::unique_ptr<std::thread> thread_ptr_;

}; // class ServerThread

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

void ServerThread::join()
{
  this->thread_ptr_->join();
}

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_SERVER_THREAD_HPP