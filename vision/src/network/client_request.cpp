#include <cstring>
#include <vector>

#include <network/client_request.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest(MessageType type, void* params)
: Message(type)
{
  switch(type_)
  {
    case MessageType::INITIALIZATION:
    {
      Initialization const initialization = params ? 
        *static_cast<Initialization*>(params)
        : Initialization::UNKNOWN;
      params_.reset(new Initialization(initialization));
    }
    case MessageType::GET_MEASUREMENTS:
    case MessageType::SHUT_DOWN:
    case MessageType::UNKNOWN:
      break;
    default:
      throw std::runtime_error("Unknown message type");
  }
}

//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest(std::string const& message)
: Message(message)
{}

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

bool ClientRequest::serializeParams(std::vector<char>* params_ptr) const
{
  CHECK_NOTNULL(params_ptr);
  switch(type_)
  {
    case MessageType::INITIALIZATION:
    {
      Initialization const initialization = *static_cast<Initialization const*>(params_.get());
      std::memcpy(params_ptr, &initialization, sizeof(MessageType));
      break;
    }
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
  switch(type_)
  {
    case MessageType::INITIALIZATION:
    {
      Initialization initialization;
      std::memcpy(params_.get(), params.data(), sizeof(Initialization));
      break;
    }
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
