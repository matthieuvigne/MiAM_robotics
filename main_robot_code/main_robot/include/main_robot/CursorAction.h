#ifndef MAIN_ROBOT_CURSOR_ACTION_H
#define MAIN_ROBOT_CURSOR_ACTION_H

#include <string>
#include "main_robot/AbstractAction.h"
#include "main_robot/ServoManager.h"

class CursorAction: public AbstractAction
{
public:
    CursorAction(RobotInterface *robot, ServoManager *servoManager):
        AbstractAction("Cursor", robot),
        servoManager_(servoManager)
    {}

    /// @brief This function is called before choosing the action in the list,
    ///        giving the opportunity for an action to update its start position and priority.
    void updateStartCondition() override;

    void actionStartTrigger() override;

    bool performAction() override;

private:
    ServoManager *servoManager_;
};


#endif
