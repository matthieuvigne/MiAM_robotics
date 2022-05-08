#ifndef NETWORK_MESSAGE_HPP
#define NETWORK_MESSAGE_HPP

#include <iostream>

#include <common/common.hpp>
#include <common/macros.hpp>
#include <common/marker.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Struct declaration
//--------------------------------------------------------------------------------------------------

enum class MessageType {UNKNOWN, INITIALIZATION, GET_MEASUREMENTS, SHUT_DOWN};
std::string print(MessageType type);

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class Message {

public:
  POINTER_TYPEDEF(Message);

public:
  DISALLOW_EVIL_CONSTRUCTORS(Message);
  Message(MessageType type);
  virtual ~Message(){}

public:
  void setType(MessageType type);
  inline void setTimestamp(int64_t timestamp_ns);

  inline MessageType getType() const;
  inline int64_t getTimestampNanoseconds() const;

  template<typename T> T const& getParamsAs() const;
  template<typename T> T const* getParamsPtrAs() const;
  template<typename T> T& getParamsAs();
  template<typename T> T* getParamsPtrAs();

  bool serialize(std::string* message) const;
  bool deserialize(std::string const& message);
  inline static MessageType deserializeType(std::string const& message);

protected:
  virtual bool serializeParams(std::vector<char>* params) const = 0;
  virtual bool deserializeParams(std::vector<char> const& params) = 0;

private:
  void initializeParams();

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

void Message::setTimestamp(int64_t timestamp_ns)
{
  timestamp_ns_ = timestamp_ns;
}

//--------------------------------------------------------------------------------------------------

template<typename T>
T* Message::getParamsPtrAs()
{
  // Check the consistency with the type of the message
  bool is_consistent = false;
  switch(type_)
  {
    case MessageType::INITIALIZATION:
      is_consistent = std::is_same<T,common::Team>::value;
      break;
    case MessageType::GET_MEASUREMENTS:
      is_consistent = std::is_same<T,common::MarkerEstimates>::value;
      break;
    case MessageType::SHUT_DOWN:
    case MessageType::UNKNOWN:
    default:
      break;
  }
  if(!is_consistent)
    throw std::runtime_error("You asked the wrong type");
  return static_cast<T*>(params_.get());
}

//--------------------------------------------------------------------------------------------------

template<typename T>
T const* Message::getParamsPtrAs() const
{
  // Check the consistency with the type of the message
  bool is_consistent = false;
  switch(type_)
  {
    case MessageType::INITIALIZATION:
      is_consistent = std::is_same<T,common::Team>::value;
      break;
    case MessageType::GET_MEASUREMENTS:
      is_consistent = std::is_same<T,common::MarkerEstimates>::value;
      break;
    case MessageType::SHUT_DOWN:
    case MessageType::UNKNOWN:
    default:
      break;
  }
  if(!is_consistent)
    throw std::runtime_error("You asked the wrong type");
  return static_cast<T const*>(params_.get());
}

//--------------------------------------------------------------------------------------------------

template<typename T>
T& Message::getParamsAs()
{
  // Return the pointer
  CHECK_NOTNULL(params_.get());
  T* result = getParamsPtrAs<T>();
  CHECK_NOTNULL(result);
  return *result;
}

//--------------------------------------------------------------------------------------------------

template<typename T>
T const& Message::getParamsAs() const
{ 
  // Return the pointer
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
