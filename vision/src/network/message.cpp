#include <cstring>
#include <vector>
#include <iostream>

#include <network/message.hpp>
//~ #include <network/socket.hpp>

/* À modifier:
 * -----------
 * - ajouter la taille au début du message
 * - ne pas communiquer la covariance
 */

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
  // Get the reference
  CHECK_NOTNULL(message_ptr);
  std::string& message = *message_ptr;

  // Append the type of the message
  std::vector<char> type_str(sizeof(MessageType));
  std::memcpy(type_str.data(), &type_, sizeof(MessageType));

  // Append the supplementary data
  std::vector<char> params_str;
  serializeParams(&params_str);

  // Get the message size
  std::vector<char> size_str(sizeof(uint16_t));
  uint16_t const msg_size = sizeof(uint16_t) + type_str.size() + params_str.size();
  std::memcpy(size_str.data(), &msg_size, sizeof(uint16_t));
  CHECK(size_str.size() == sizeof(uint16_t));

  // Build the message
  message  = std::string(size_str.data(), size_str.size());
  message += std::string(type_str.data(), type_str.size());
  message += std::string(params_str.size(), params_str.size());
  message.shrink_to_fit();
  return true;
}

//--------------------------------------------------------------------------------------------------

bool Message::deserialize(std::string const& message)
{
  // Get the size of the message
  size_t current_byte = 0u;
  uint16_t msg_size_bytes = 0u;
  std::memcpy(&msg_size_bytes, &message[current_byte], sizeof(uint16_t));
  current_byte += sizeof(uint16_t);

  // Get the type of the message
  std::memcpy(&type_, &message[current_byte], sizeof(MessageType));
  current_byte += sizeof(MessageType);

  // Get the associated parameters of the client's message according the message's type
  if(current_byte == msg_size_bytes) return true;
  std::string::const_iterator params_begin = message.cbegin() + current_byte;
  std::string::const_iterator params_end = message.cbegin() + msg_size_bytes;
  try
  {
    std::vector<char> const params_bytes(params_begin, params_end);
    return deserializeParams(params_bytes);
  }
  catch(std::length_error&)
  {
    // Failed to deseralize, just drop the message.
  }
  return false;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
