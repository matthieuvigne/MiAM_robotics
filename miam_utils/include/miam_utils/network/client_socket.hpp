#ifndef NETWORK_CLIENT_SOCKET_HPP
#define NETWORK_CLIENT_SOCKET_HPP

#include <miam_utils/network/socket.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class ClientSocket : public Socket
{
  public:
    ClientSocket();
    virtual ~ClientSocket(){};

    /// @brief connects to socket
    /// @param host id address
    /// @param port port 
    /// @param isUDP if true, set udp broadcast
    void connect(std::string host, int port, bool isUDP=false);
    ClientSocket const& operator << (std::string const&) const;
    ClientSocket const& operator >> (std::string&) const;
};

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_CLIENT_SOCKET_HPP
