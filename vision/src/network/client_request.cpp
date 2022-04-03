#include <cstring>
#include <vector>

#include <network/client_request.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest(RequestType type, RequestParams const* params_ptr)
: type_   (type)
{
  if(params_ptr)
  {
    this->params_.reset(new RequestParams);
    *(this->params_) = *params_ptr;
  }
}

//--------------------------------------------------------------------------------------------------

ClientRequest::ClientRequest(std::string const& request)
{
  this->deserialize(request, &(this->type_), this->params_.get());
}

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

bool ClientRequest::serialize(std::string* request_ptr) const
{
  if(request_ptr == NULL) return false;
  std::string& request = *request_ptr;
  
  // Append the type of the request
  size_t const n = sizeof(RequestType);
  std::vector<char> type(n+1);
  std::memcpy(&type, &(this->type_), sizeof(RequestType));
  type[n] = '\0';
  
  // Append the supplementary data
  std::vector<char> params;
  switch(this->type_)
  {
    case RequestType::GET_MEASUREMENTS:
    case RequestType::SHUT_DOWN:
    default:
      break;
  }
  params.push_back('\0');
  
  
  // Build the message
  request = std::string(type.data(),type.size()) + std::string(params.data(),params.size());
  request.shrink_to_fit();
  return true;
}

//--------------------------------------------------------------------------------------------------

bool ClientRequest::deserialize(
  std::string const& request,
  RequestType* type,
  RequestParams* params)
{
  // [TODO]
  return true;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
