#ifndef NETWORK_SOCKET_EXCEPTION_HPP
#define NETWORK_SOCKET_EXCEPTION_HPP

#include <string>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class SocketException
{
  public:
    SocketException ( std::string s ) : m_s ( s ) {};
    ~SocketException (){};
    std::string const& description() const { return m_s; }

  private:
    std::string m_s;

}; // SocketException

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_SOCKET_EXCEPTION_HPP
