#ifndef MAIN_ROBOT_ABSTRACT_ACTION_H
#define MAIN_ROBOT_ABSTRACT_ACTION_H

#include "common/RobotInterface.h"

#define FRONT_CLAW_XOFFSET 182.0
#define BACK_CLAW_XOFFSET 135.0
#define BACK_DIFF_XOFFSET FRONT_CLAW_XOFFSET - BACK_CLAW_XOFFSET


class AbstractAction
{
public:
    AbstractAction(std::string const& actionName, RobotInterface *robot):
        name_(actionName),
        robot_(robot)
    {}

    /// @brief This function is called before choosing the action in the list,
    ///        giving the opportunity for an action to update its start position and priority.
    virtual void updateStartCondition() = 0;

    /// @brief This function is called when the action is selected, and can be used to trigger
    ///        events that can happen while the robot is going to the start position.
    ///        This function should be non-blocking.
    virtual void actionStartTrigger()
    {}

    /// @brief Perform the action.
    /// @return True if the action can be removed from the action list, false otherwise.
    virtual bool performAction()
    {
        return true;
    }


    friend std::ostream& operator<<(std::ostream& os, const AbstractAction& dt);
    RobotPosition startPosition_;
    bool isStartMotionBackward_ = false; ///< Go to start should be done backward
    int priority_ = -1; ///< Action priority: higher priority actions will be attempted first. Action with negative priorities will not be performed.
    bool wasFailed = false;

    std::string getName()
    {
        return name_;
    }
protected:
    std::string name_;
    RobotInterface *robot_;
};

inline std::ostream& operator<<(std::ostream& os, const AbstractAction& a)
{
    os << a.name_ << ": failed " << a.wasFailed << " priority " << a.priority_ << ", (" << a.startPosition_ << ")";
    return os;
}

#endif
