#include <iostream>

#include <network/server_socket.hpp>
#include <network/socket_exception.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

ServerSocket::ServerSocket (int port)
{
  if (!Socket::create())
    throw SocketException ("Could not create server socket.");

  if (!Socket::bind(port))
    throw SocketException ("Could not bind to port.");

  if (!Socket::listen())
    throw SocketException ("Could not listen to socket.");
}

//--------------------------------------------------------------------------------------------------

ServerSocket::~ServerSocket()
{
  std::cout << "Shutting down the server" << std::endl;
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

ServerSocket const& ServerSocket::operator << (std::string const& s) const
{
  if(!Socket::send(s))
    throw SocketException ("Could not write to socket.");
  return *this;
}

//--------------------------------------------------------------------------------------------------

ServerSocket const& ServerSocket::operator >> (std::string& s) const
{
  if (!Socket::recv(s))
    throw SocketException ("Could not read from socket.");
  return *this;
}

//--------------------------------------------------------------------------------------------------

void ServerSocket::accept(ServerSocket& sock)
{
  if(!Socket::accept(sock))
    throw SocketException("Could not accept socket.");
}

//--------------------------------------------------------------------------------------------------

} // namespace network
