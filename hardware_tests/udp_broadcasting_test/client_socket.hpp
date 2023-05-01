#ifndef NETWORK_CLIENT_SOCKET_HPP
#define NETWORK_CLIENT_SOCKET_HPP

#include <socket.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class ClientSocket : public Socket
{
  public:
    ClientSocket();
    virtual ~ClientSocket(){};

    void connect(std::string host, int port, bool isUDP=false);
    ClientSocket const& operator << (std::string const&) const;
    ClientSocket const& operator >> (std::string&) const;
};

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_CLIENT_SOCKET_HPP
