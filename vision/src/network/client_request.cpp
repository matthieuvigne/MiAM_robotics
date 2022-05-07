#include <cstring>
#include <vector>

#include <common/common.hpp>
#include <common/logger.hpp>
#include <network/client_request.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest(MessageType type, std::shared_ptr<void> params)
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
  CHECK_NOTNULL(params_ptr);
  switch(type_)
  {
    case MessageType::INITIALIZATION:
    {
      CONSOLE << "OK";
      common::Team const& team = getParamsAs<common::Team>();
      CONSOLE << "OK";
      std::memcpy(params_ptr, &team, sizeof(common::Team));
      CONSOLE << "OK";
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
