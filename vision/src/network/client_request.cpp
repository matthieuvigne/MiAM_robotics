#include <cstring>
#include <vector>

#include <network/client_request.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest(MessageType type, void* params)
: Message(type, params)
{}

//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest(std::string const& message)
: Message(message)
{}

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

bool ClientRequest::serializeParams(std::vector<char>* params_ptr) const
{
  if(params_ptr == NULL) return false;
  std::vector<char> params = *params_ptr;
  switch(this->type_)
  {
    case MessageType::UNKNOWN:
    case MessageType::GET_MEASUREMENTS:
    case MessageType::SHUT_DOWN:
      break;
    default:
      return false;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

bool ClientRequest::deserializeParams(std::vector<char> const& params)
{
  switch(this->type_)
  {
    case MessageType::GET_MEASUREMENTS:
    case MessageType::SHUT_DOWN:
    case MessageType::UNKNOWN:
      break;
    default:
      return false;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
