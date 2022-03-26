#ifndef NETWORK_MESSAGES_HPP
#define NETWORK_MESSAGES_HPP

#include <common/macros.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class Message {
  public:
    POINTER_TYPEDEF(Message);
    enum class Type { GET_LATEST_MEASUREMENTS, LOOK_AT_BOARD_COORDINATES };
    
  public:
    Message(Type type);
  
  public:
    std::string serialize() const;
    bool deserialize(std::string const& message);
    
  private:
    // TODO
  
}; // class tMessage

//--------------------------------------------------------------------------------------------------

} // namespace network

#endif // NETWORK_MESSAGES_HPP
