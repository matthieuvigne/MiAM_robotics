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

public:
  ClientRequest();
  ClientRequest(MessageType type);
  ClientRequest(std::string const& message);
  virtual ~ClientRequest(){}

private:
  bool serializeParams(std::vector<char>* params) const;
  bool deserializeParams(std::vector<char> const& params);

}; // class ClientRequest

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_CLIENT_REQUEST_HPP
