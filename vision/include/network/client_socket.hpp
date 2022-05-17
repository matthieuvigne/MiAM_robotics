#ifndef NETWORK_CLIENT_SOCKET_HPP
#define NETWORK_CLIENT_SOCKET_HPP

#include <network/socket.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class ClientSocket : private Socket
{
  public:
    ClientSocket();

    void connect(std::string host, int port);
    virtual ~ClientSocket(){};

    ClientSocket const& operator << (std::string const&) const;
    ClientSocket const& operator >> (std::string&) const;
};

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_CLIENT_SOCKET_HPP
