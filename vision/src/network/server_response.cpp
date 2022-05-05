#include <network/server_response.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

ServerResponse::ServerResponse(MessageType type, std::shared_ptr<void> params)
: Message(type, params)
{}

//--------------------------------------------------------------------------------------------------

ServerResponse::ServerResponse(std::string const& message)
: Message(message)
{}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

bool ServerResponse::serializeParams(std::vector<char>* serialized_params) const
{
  switch(type_)
  {
    case MessageType::GET_MEASUREMENTS:
    {
      CHECK_NOTNULL(serialized_params);
      common::MarkerIdToEstimate const& markers = getParamsAs<common::MarkerIdToEstimate>();
      common::Marker::serialize(markers, serialized_params);
    }
    case MessageType::INITIALIZATION:
    case MessageType::SHUT_DOWN:
    case MessageType::UNKNOWN:
    default:
      break;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

bool ServerResponse::deserializeParams(std::vector<char> const& serialized_params)
{
  switch(this->type_)
  {
    case MessageType::GET_MEASUREMENTS:
    {
      params_.reset(new common::MarkerIdToEstimate);
      common::MarkerIdToEstimate& markers = getParamsAs<common::MarkerIdToEstimate>();
      common::Marker::deserialize(serialized_params, &markers);
    }
    case MessageType::SHUT_DOWN:
    case MessageType::INITIALIZATION:
    case MessageType::UNKNOWN:
    default:
      break;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
