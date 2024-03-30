#ifndef _MESSAGE_HPP
#define _MESSAGE_HPP

#include <vector>
#include <miam_utils/trajectory/Utilities.h>
#include <memory>

/*
Messages from SCD to PAMI can be of categories:
- configuration message: set side (right side, left side)
- immediate new trajectory: pami should execute traj immediatly (i.e. setup trajectory)
- match state message: whether match started and currrent match time
- alternative travel trajectory: replanified trajectory to objective 

Messages from PAMI to SCD can be of categories:
- pami report message
*/

using namespace miam::trajectory;
typedef std::vector<float > VecFloat;

enum MessageType
{
    CONFIGURATION = 0,
    NEW_TRAJECTORY = 1,
    MATCH_STATE = 2,
    PAMI_REPORT = 10,
    ERROR = 99
};

enum PlayingSide
{
    BLUE_SIDE = 0,
    YELLOW_SIDE = 1
};

class Message
{
public:
    MessageType get_message_type() { return messageType_; }
    uint8_t get_sender_id() { return senderId_; }

    static std::shared_ptr<Message > parse(VecFloat message, uint8_t senderId = 255);
    virtual VecFloat serialize() { };

private:
    MessageType messageType_;
    uint8_t senderId_;
    Message(MessageType mt, uint8_t senderId = 255) : messageType_(mt), senderId_(senderId) {};

    friend class ConfigurationMessage;
    friend class MatchStateMessage;
    friend class NewTrajectoryMessage;
    friend class ErrorMessage;
    friend class PamiReportMessage;
};

/* Specialized messages */

// CONFIGURATION
class ConfigurationMessage : public Message
{
public:
    ConfigurationMessage(PlayingSide playingSide, uint8_t senderId = 255) :
        Message(MessageType::CONFIGURATION, senderId),
        playingSide_(playingSide) {};
    
    VecFloat serialize();

    PlayingSide playingSide_;
};

// NEW TRAJECTORY
class NewTrajectoryMessage : public Message
{
public:
    NewTrajectoryMessage(TrajectoryVector trajectory, uint8_t senderId = 255) :
        Message(MessageType::NEW_TRAJECTORY, senderId),
        newTrajectory_(trajectory) {};

    VecFloat serialize();
    
    TrajectoryVector newTrajectory_;
};

// MATCH_STATE
class MatchStateMessage : public Message
{
public:
    MatchStateMessage(bool matchStarted, float matchTime, uint8_t senderId = 255) :
        Message(MessageType::MATCH_STATE, senderId),
        matchStarted_(matchStarted),
        matchTime_(matchTime) {};
    
    VecFloat serialize();

    bool matchStarted_;
    float matchTime_;
};

// ERROR
class ErrorMessage : public Message
{
public:
    ErrorMessage(uint8_t senderId = 255) : Message(MessageType::ERROR, senderId) {};  

    VecFloat serialize(); 
};

// PAMI_REPORT
class PamiReportMessage : public Message
{
public:
    PamiReportMessage(
        bool matchStarted, 
        float matchTime, 
        PlayingSide playingSide, 
        uint8_t senderId = 255
    ) : Message(MessageType::PAMI_REPORT, senderId),
        matchStarted_(matchStarted),
        matchTime_(matchTime),
        playingSide_(playingSide) {};
    
    VecFloat serialize();

    bool matchStarted_;
    float matchTime_;
    PlayingSide playingSide_;
};

#endif