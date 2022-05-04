#ifndef NETWORK_MESSAGE_HPP
#define NETWORK_MESSAGE_HPP

#include <iostream>

#include <common/macros.hpp>
#include <common/marker.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Struct declaration
//--------------------------------------------------------------------------------------------------

enum class MessageType {UNKNOWN, INITIALIZATION, GET_MEASUREMENTS, SHUT_DOWN};

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class Message {

public:
  POINTER_TYPEDEF(Message);
  DISALLOW_EVIL_CONSTRUCTORS(Message);

public:
  Message(MessageType type);
  Message(std::string const& message);
  virtual ~Message(){}

public:
  bool serialize(std::string* message) const;
  bool deserialize(std::string const& message);
  inline MessageType getType() const;
  inline void const* getParams() const;
  inline static MessageType deserializeType(std::string const& message);

protected:
  virtual bool serializeParams(std::vector<char>* params) const = 0;
  virtual bool deserializeParams(std::vector<char> const& params) = 0;

protected:
  MessageType type_ = MessageType::UNKNOWN;
  std::shared_ptr<void> params_ = nullptr;

}; // class Message

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

MessageType Message::getType() const
{
  return type_;
}

//--------------------------------------------------------------------------------------------------

void const* Message::getParams() const
{
  return params_.get();
}

//--------------------------------------------------------------------------------------------------

MessageType Message::deserializeType(std::string const& message)
{
  MessageType message_type = MessageType::UNKNOWN;
  size_t constexpr type_byte = sizeof(uint16_t);
  size_t constexpr type_size = sizeof(MessageType);
  std::memcpy(&message_type, &message[type_byte], type_size);
  return message_type;
}

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_MESSAGE_HPP
