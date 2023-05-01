#include <miam_utils/network/socket.hpp>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>

#include <miam_utils/network/macros.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

Socket::Socket() :
  m_sock ( -1 )
{
  memset ( &m_addr, 0, sizeof ( m_addr ) );
}

//--------------------------------------------------------------------------------------------------

Socket::~Socket()
{
  if ( is_valid() )
    ::close ( m_sock );
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

bool Socket::create(bool isUDP)
{
  if (isUDP)
  {
    m_sock = socket ( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
  }
  else
  {
    m_sock = socket ( AF_INET, SOCK_STREAM, 0 );
  }

  if ( ! is_valid() )
    return false;

  // TIME_WAIT - argh
  int on = 1;
  if (isUDP)
  {
    if ( setsockopt ( m_sock, SOL_SOCKET, SO_BROADCAST, ( char const* ) &on, sizeof ( on ) ) == -1 )
      return false;
  }
  else
  {
    if ( setsockopt ( m_sock, SOL_SOCKET, SO_REUSEADDR, ( char const* ) &on, sizeof ( on ) ) == -1 )
      return false;
  }

  return true;
}

//--------------------------------------------------------------------------------------------------

bool Socket::bind ( int const port )
{
  if(!this->is_valid())
    return false;

  m_addr.sin_family = AF_INET;
  m_addr.sin_addr.s_addr = INADDR_ANY;
  m_addr.sin_port = htons ( port );

  int bind_return = ::bind ( m_sock, (sockaddr*) &m_addr, sizeof(m_addr) );
  if ( bind_return == -1 )
    return false;

  return true;
}

//--------------------------------------------------------------------------------------------------

bool Socket::listen() const
{
  if(!this->is_valid())
    return false;
  int listen_return = ::listen ( m_sock, MAXCONNECTIONS );
  if ( listen_return == -1 )
    return false;
  return true;
}

//--------------------------------------------------------------------------------------------------

bool Socket::accept(Socket& new_socket) const
{
  int addr_length = sizeof(m_addr);
  new_socket.m_sock = ::accept( m_sock, (sockaddr*) &m_addr, (socklen_t*) &addr_length );
  if (new_socket.m_sock <= 0)
    return false;
  return true;
}

//--------------------------------------------------------------------------------------------------

bool Socket::send(std::string const s) const
{
  int status = ::send( m_sock, s.c_str(), s.size(), MSG_NOSIGNAL );
  if(status == -1) return false;
  return true;
}

//--------------------------------------------------------------------------------------------------

int Socket::recv(std::string& s) const
{
  char buf[MAXRECV+1];
  memset(buf, '\0', MAXRECV+1);
  int status = ::recv(m_sock, buf, MAXRECV, 0);
  s = std::string(buf,MAXRECV);
  return status;
}

//--------------------------------------------------------------------------------------------------

bool Socket::connect(std::string const host, int const port)
{
  if(!this->is_valid())
    return false;
  m_addr.sin_family = AF_INET;
  m_addr.sin_port = htons ( port );
  int status = inet_pton ( AF_INET, host.c_str(), &m_addr.sin_addr );
  if ( errno == EAFNOSUPPORT )
    return false;
  status = ::connect ( m_sock, ( sockaddr * ) &m_addr, sizeof ( m_addr ) );
  if ( status == 0 )
    return true;
  else
    return false;
}

//--------------------------------------------------------------------------------------------------

void Socket::set_non_blocking(bool const b)
{
  int opts;
  opts = fcntl ( m_sock, F_GETFL );

  if ( opts < 0 )
    return;

  if ( b )
    opts = ( opts | O_NONBLOCK );
  else
    opts = ( opts & ~O_NONBLOCK );

  fcntl ( m_sock, F_SETFL,opts );
}

//--------------------------------------------------------------------------------------------------

} // namespace network
