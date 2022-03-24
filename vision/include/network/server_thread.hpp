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

public:

  POINTER_TYPEDEF(ServerThread);

public:

  ServerThread(int port);
  virtual ~ServerThread();

private:

  void serverThread();

public:

  // Threading
  mutable std::mutex mtx_;
  std::unique_ptr<std::thread> thread_ptr_;

}; // class ServerThread

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_SERVER_THREAD_HPP
