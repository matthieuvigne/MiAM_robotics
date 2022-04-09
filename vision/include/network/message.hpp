#ifndef NETWORK_MESSAGE_HPP
#define NETWORK_MESSAGE_HPP

#include <common/macros.hpp>
#include <common/marker.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Struct declaration
//--------------------------------------------------------------------------------------------------

enum class MessageType {UNKNOWN, GET_MEASUREMENTS, SHUT_DOWN};

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class Message {

public:
  POINTER_TYPEDEF(Message);
  DISALLOW_EVIL_CONSTRUCTORS(Message);

public:
  Message(MessageType type, void* params = NULL);
  Message(std::string const& message);
  virtual ~Message(){}

public:
  inline void setParams(void* params);
  bool serialize(std::string* message) const;
  bool deserialize(std::string const& message);
  inline MessageType getType() const;
  inline void const* getParams() const;

protected:
  virtual bool serializeParams(std::vector<char>* params) const = 0;
  virtual bool deserializeParams(std::vector<char> const& params) = 0;

protected:
  MessageType type_ = MessageType::UNKNOWN;
  void* params_ = NULL;

}; // class Message

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

MessageType Message::getType() const
{
  return this->type_;
}

//--------------------------------------------------------------------------------------------------

void Message::setParams(void* params)
{
  this->params_ = params;
}

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_MESSAGE_HPP
