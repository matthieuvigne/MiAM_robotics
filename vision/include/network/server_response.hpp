#ifndef NETWORK_SERVER_RESPONSE_HPP
#define NETWORK_SERVER_RESPONSE_HPP

#include <common/macros.hpp>
#include <network/message.hpp>

namespace network {

class ServerResponse : public Message {

public:
  POINTER_TYPEDEF(ServerResponse);
  DISALLOW_EVIL_CONSTRUCTORS(ServerResponse);
  ServerResponse(MessageType type, std::shared_ptr<void> params = nullptr);
  ServerResponse(std::string const& message);
  virtual ~ServerResponse(){}

private:
  bool serializeParams(std::vector<char>* params) const;
  bool deserializeParams(std::vector<char> const& params);

}; // class ClientRequest

} // namespace network

#endif // NETWORK_SERVER_RESPONSE_HPP
