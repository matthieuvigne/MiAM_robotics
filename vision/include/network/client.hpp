#ifndef NETWORK_CLIENT_HPP
#define NETWORK_CLIENT_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <common/macros.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class definition
//--------------------------------------------------------------------------------------------------

class Client {

public:

  typedef std::unique_ptr<char*> Message;

public:

  POINTER_TYPEDEF(Client);

public:

  Client(
    std::mutex* mtx_ptr,
    std::condition_variable* con_ptr,
    std::queue<Message>* buffer_ptr);
  virtual ~Client();

public:

  void clientThread();

public:

    std::mutex* const client_mtx_;
    std::condition_variable* const client_con_;
    std::queue<Message>* const client_buffer_;

}; // class Client

} // namespace network

#endif // NETWORK_CLIENT_HPP
