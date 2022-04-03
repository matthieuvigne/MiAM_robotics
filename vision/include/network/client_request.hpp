#ifndef NETWORK_CLIENT_REQUEST_HPP
#define NETWORK_CLIENT_REQUEST_HPP

#include <common/macros.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Struct declaration
//--------------------------------------------------------------------------------------------------

enum class RequestType {GET_MEASUREMENTS, SHUT_DOWN};

//--------------------------------------------------------------------------------------------------

struct RequestParams {
  POINTER_TYPEDEF(RequestParams);
}; ///< Structure to inherit from for specific request types

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class ClientRequest {

public:
  POINTER_TYPEDEF(ClientRequest);
  DISALLOW_EVIL_CONSTRUCTORS(ClientRequest);
  ClientRequest(RequestType type, RequestParams const* params = NULL);
  ClientRequest(std::string const& request);
  virtual ~ClientRequest(){}

public:
  bool serialize(std::string* request) const;
  bool deserialize(std::string const& request, RequestType* type, RequestParams* params);

private:
  RequestType type_ = RequestType::GET_MEASUREMENTS;
  RequestParams::UniquePtr params_ = nullptr;

}; // class ClientRequest

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_CLIENT_REQUEST_HPP
