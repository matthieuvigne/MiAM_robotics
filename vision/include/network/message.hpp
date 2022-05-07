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
  Message(MessageType type, std::shared_ptr<void> params = nullptr);
  //~ Message(std::string const& message);
  virtual ~Message(){}

public:
  bool serialize(std::string* message) const;
  bool deserialize(std::string const& message);
  inline MessageType getType() const;
  inline int64_t getTimestampNanoseconds() const;
  template<typename T> T const& getParamsAs() const;
  template<typename T> T const* getParamsPtrAs() const;
  inline static MessageType deserializeType(std::string const& message);

protected:
  template<typename T> T& getParamsAs();
  template<typename T> T* getParamsPtrAs();
  virtual bool serializeParams(std::vector<char>* params) const = 0;
  virtual bool deserializeParams(std::vector<char> const& params) = 0;

protected:
  MessageType type_ = MessageType::UNKNOWN;
  int64_t timestamp_ns_ = 0;
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

int64_t Message::getTimestampNanoseconds() const
{
  return timestamp_ns_;
}

//--------------------------------------------------------------------------------------------------

template<typename T>
T* Message::getParamsPtrAs()
{
  return static_cast<T*>(params_.get());
}

//--------------------------------------------------------------------------------------------------

template<typename T>
T const* Message::getParamsPtrAs() const
{
  return static_cast<T const*>(params_.get());
}

//--------------------------------------------------------------------------------------------------

template<typename T>
T& Message::getParamsAs()
{
  CHECK_NOTNULL(params_.get());
  T* result = getParamsPtrAs<T>();
  CHECK_NOTNULL(result);
  return *result;
}

//--------------------------------------------------------------------------------------------------

template<typename T>
T const& Message::getParamsAs() const
{
  CHECK_NOTNULL(params_.get());
  T const* result = getParamsPtrAs<T>();
  CHECK_NOTNULL(result);
  return *result;
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
