#include <cstring>
#include <vector>

#include <common/common.hpp>
#include <common/logger.hpp>
#include <network/client_request.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest()
: Message(MessageType::UNKNOWN)
{}

//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest(MessageType type)
: Message(type)
{}

//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest(std::string const& message)
: Message(MessageType::UNKNOWN)
{
  deserialize(message);
}

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

bool ClientRequest::serializeParams(std::vector<char>* params_ptr) const
{
  CHECK_NOTNULL(params_ptr);
  std::vector<char>& params = *params_ptr;
  switch(type_)
  {
    case MessageType::INITIALIZATION:
    {
      common::Team const& team = getParamsAs<common::Team>();
      params.resize(sizeof(common::Team));
      std::memcpy(params.data(), &team, sizeof(common::Team));
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
      params_.reset(new common::Team);
      std::memcpy(params_.get(), params.data(), sizeof(common::Team));
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
