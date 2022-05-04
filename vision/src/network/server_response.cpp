#include <network/server_response.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

ServerResponse::ServerResponse(MessageType type, void const* params)
: Message(type)
{
  switch(type_)
  {
    case MessageType::GET_MEASUREMENTS:
    {
      params_.reset(new common::MarkerIdToEstimate);
      common::MarkerIdToEstimate const* markers_ptr =
        static_cast<common::MarkerIdToEstimate const*>(params);
      *static_cast<common::MarkerIdToEstimate*>(params_.get()) = *markers_ptr;
      break;
    }
    case MessageType::INITIALIZATION:
    case MessageType::SHUT_DOWN:
    case MessageType::UNKNOWN:
      break;
    default:
      throw std::runtime_error("Unknown message type");
    
  }
}

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
      common::MarkerIdToEstimate const& markers =
        *static_cast<common::MarkerIdToEstimate const*>(params_.get());
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
      common::MarkerIdToEstimate& markers =
        *static_cast<common::MarkerIdToEstimate*>(params_.get());
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
