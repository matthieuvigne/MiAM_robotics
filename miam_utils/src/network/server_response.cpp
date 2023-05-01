// #include <common/logger.hpp>
#include <miam_utils/network/server_response.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

ServerResponse::ServerResponse()
: Message(MessageType::UNKNOWN)
{}

//--------------------------------------------------------------------------------------------------

ServerResponse::ServerResponse(MessageType type)
: Message(type)
{}

//--------------------------------------------------------------------------------------------------

ServerResponse::ServerResponse(std::string const& message)
: Message(MessageType::UNKNOWN)
{
  deserialize(message);
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

bool ServerResponse::serializeParams(std::vector<char>* serialized_params) const
{
  switch(type_)
  {
    // case MessageType::GET_MEASUREMENTS:
    // {
    //   CHECK_NOTNULL(serialized_params);
    //   common::MarkerList const& markers = getParamsAs<common::MarkerList>();
    //   common::Marker::serialize(markers, serialized_params);
    // }
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
    // case MessageType::GET_MEASUREMENTS:
    // {
    //   params_.reset(new common::MarkerList);
    //   common::MarkerList& markers = getParamsAs<common::MarkerList>();
    //   common::Marker::deserialize(serialized_params, &markers);
    // }
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
