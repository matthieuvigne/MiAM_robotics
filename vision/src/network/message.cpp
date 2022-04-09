#include <cstring>
#include <vector>

#include <network/message.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

Message::Message(MessageType type, void* params)
: type_   (type),
  params_ (params)
{}

//--------------------------------------------------------------------------------------------------

Message::Message(std::string const& message)
: Message(MessageType::UNKNOWN, NULL)
{
  this->deserialize(message);
}

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

bool Message::serialize(std::string* message_ptr) const
{
  if(message_ptr == NULL) return false;
  std::string& message = *message_ptr;
  
  // Append the type of the message
  size_t const n = sizeof(MessageType);
  std::vector<char> type(n+1);
  std::memcpy(&type, &(this->type_), sizeof(MessageType));
  type[n] = '\0';
  
  // Append the supplementary data
  std::vector<char> params;
  this->serializeParams(&params);
  params.push_back('\0');
  
  // Build the message
  message = std::string(type.data(),type.size()) + std::string(params.data(),params.size());
  message.shrink_to_fit();
  return true;
}

//--------------------------------------------------------------------------------------------------

bool Message::deserialize(std::string const& message)
{
  // Get the type of the client's message
  std::size_t const pos1 = message.find('\0');
  std::memcpy(&(this->type_), message.data(), pos1);
  
  // Get the associated parameters of the client's message according the message's type
  std::size_t const pos2 = message.find('\0', pos1 + 1);
  std::string::const_iterator params_begin = message.cbegin() + pos1 + 1;
  std::string::const_iterator params_end = message.cbegin() + pos2 - 1;
  std::vector<char> const params_bytes(params_begin, params_end); 
  bool const success = this->deserializeParams(params_bytes);
  return success;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
