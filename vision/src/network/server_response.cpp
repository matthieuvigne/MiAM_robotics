#include <network/server_response.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

ServerResponse::ServerResponse(MessageType type, void* params)
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
  switch(this->type_)
  {
    case MessageType::GET_MEASUREMENTS:
    {
      CHECK_NOTNULL(serialized_params);
      common::MarkerIdToEstimate const* markers_ptr =
        static_cast<common::MarkerIdToEstimate const*>(this->params_);
      CHECK_NOTNULL(markers_ptr);
      common::Marker::serialize(*markers_ptr, serialized_params);
    }
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
      common::MarkerIdToEstimate& markers =
        *static_cast<common::MarkerIdToEstimate*>(this->params_);
      common::Marker::deserialize(serialized_params, &markers);
    }
    case MessageType::SHUT_DOWN:
    case MessageType::UNKNOWN:
    default:
      break;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
