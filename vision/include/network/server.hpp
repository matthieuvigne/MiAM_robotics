#ifndef NETWORK_SERVER_HPP
#define NETWORK_SERVER_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <assert.h>

#include <common/macros.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class definition
//--------------------------------------------------------------------------------------------------

class Server {

public:

  typedef std::unique_ptr<char*> Message;

public:

  POINTER_TYPEDEF(Server);

public:

  Server();
  virtual ~Server();

private:

  void serverThread();

public:

  std::mutex server_mtx_;
  std::condition_variable server_con_;
  std::queue<Message> server_buffer_;

private:

  bool shut_down_ = false;

}; // class Server

} // namespace network

#endif // NETWORK_SERVER_HPP
