#include <iostream>

#include <network/client_socket.hpp>
#include <network/socket_exception.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructors
//--------------------------------------------------------------------------------------------------

ClientSocket::ClientSocket()
: Socket()
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void ClientSocket::connect(std::string host, int port, bool isUDP)
{
  if(!Socket::create(isUDP))
    throw SocketException("Could not create client socket.");

  if(!Socket::connect(host, port))
    throw SocketException("Could not bind to port.");
}
//--------------------------------------------------------------------------------------------------

ClientSocket const& ClientSocket::operator << (std::string const& s) const
{
  if(!Socket::send(s))
    throw SocketException("Could not write to socket.");
  return *this;
}

//--------------------------------------------------------------------------------------------------

ClientSocket const& ClientSocket::operator >> (std::string& s) const
{
  if(!Socket::recv(s))
    throw SocketException("Could not read from socket.");
  return *this;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
