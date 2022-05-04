#ifndef NETWORK_CLIENT_REQUEST_HPP
#define NETWORK_CLIENT_REQUEST_HPP

#include <common/macros.hpp>
#include <network/message.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class ClientRequest : public Message {

public:
  POINTER_TYPEDEF(ClientRequest);
  DISALLOW_EVIL_CONSTRUCTORS(ClientRequest);
  enum class Initialization { UNKNOWN, PURPLE_TEAM, YELLOW_TEAM };
  ClientRequest(MessageType type, void* params = NULL);
  ClientRequest(std::string const& message);
  virtual ~ClientRequest(){}

private:
  bool serializeParams(std::vector<char>* params) const;
  bool deserializeParams(std::vector<char> const& params);

}; // class ClientRequest

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_CLIENT_REQUEST_HPP
