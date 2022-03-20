#ifndef NETWORK_SERVER_HPP
#define NETWORK_SERVER_HPP

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

class Server : public ServerSocket {

public:

  typedef std::unique_ptr<char*> Message;

public:

  POINTER_TYPEDEF(Server);

public:

  Server(int port);
  virtual ~Server();

public:

  inline void launchThread();
  inline void join() const;
  inline void abortThread();

private:

  void serverThread();

public:

  // Threading
  mutable std::mutex thread_mtx_;
  std::condition_variable thread_con_;
  std::unique_ptr<std::thread> thread_ptr_;
  std::queue<Message> server_buffer_;
  bool abort_thread_ = false;

}; // class Server

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

void Server::launchThread()
{
  this->thread_ptr_.reset(new std::thread([=](){this->serverThread();}));
}

//--------------------------------------------------------------------------------------------------

void Server::join() const
{
  this->thread_ptr_->join();
}

//--------------------------------------------------------------------------------------------------

void Server::abortThread()
{
  if(this->thread_ptr_ != nullptr)
  {
    this->abort_thread_ = true;
    this->thread_con_.notify_all();
    this->thread_ptr_->join();
    this->thread_ptr_ = nullptr;
    this->abort_thread_ = false;
  }
}

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_SERVER_HPP
