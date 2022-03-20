#ifndef NETWORK_CLIENT_HPP
#define NETWORK_CLIENT_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <common/macros.hpp>
#include <network/client_socket.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class definition
//--------------------------------------------------------------------------------------------------

class Client : public ClientSocket {

public:

  typedef std::unique_ptr<char*> Message;

public:

  POINTER_TYPEDEF(Client);

public:

  Client(std::string host, int port);
  virtual ~Client();

public:

  void clientThread();

public:

    std::mutex client_mtx_;
    std::condition_variable client_con_;
    std::queue<Message> client_buffer_;

}; // class Client

} // namespace network

#endif // NETWORK_CLIENT_HPP
