#include <cstring>
#include <vector>
#include <iostream>

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
  std::vector<char> type(n);
  std::memcpy(type.data(), &(this->type_), sizeof(MessageType));

  // Append the supplementary data
  std::vector<char> params;
  this->serializeParams(&params);

  // Build the message
  message = std::string(type.data(),type.size()) + std::string(params.data(),params.size());
  message.shrink_to_fit();
  return true;
}

//--------------------------------------------------------------------------------------------------

bool Message::deserialize(std::string const& message)
{
  // Get the type of the client's message
  size_t constexpr type_size_bytes = sizeof(MessageType);
  std::memcpy(&(this->type_), message.data(), type_size_bytes);

  // Get the associated parameters of the client's message according the message's type
  std::string::const_iterator params_begin = message.cbegin() + type_size_bytes;
  std::string::const_iterator params_end = message.cend();
  try
  {

    std::vector<char> const params_bytes(params_begin, params_end);
    return this->deserializeParams(params_bytes);
  }
  catch (std::length_error &)
  {
    // Failed to deseralize, just drop the message.
  }
    return false;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
