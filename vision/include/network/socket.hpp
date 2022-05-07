#ifndef NETWORK_SOCKET_HPP
#define NETWORK_SOCKET_HPP

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>

namespace network {

//--------------------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------------------

int const MAXHOSTNAME = 200;
int const MAXCONNECTIONS = 5;
int const MAXRECV = 1000; // Max message size = 1kB

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class Socket
{
  public:

    // Constructor and destructor
    Socket();
    virtual ~Socket();

    // Server initialization
    bool create();
    bool bind(int const port);
    bool listen() const;
    bool accept(Socket&) const;

    // Client initialization
    bool connect(std::string const host, int const port);

    // Data Transimission
    bool send(std::string const) const;
    int recv(std::string&) const;

    // Checks
    void set_non_blocking(bool const);
    inline bool is_valid() const { return m_sock != -1; }

 private:

    int m_sock;
    sockaddr_in m_addr;
};

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_SOCKET_HPP
