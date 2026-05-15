#ifndef MAIN_ROBOT_CENTRAL_DROP_ACTION_H
#define MAIN_ROBOT_CENTRAL_DROP_ACTION_H

#include <string>
#include "main_robot/AbstractAction.h"
#include "main_robot/ServoManager.h"

class CentralMultiDrop: public AbstractAction
{
public:
    CentralMultiDrop(RobotInterface *robot, ServoManager *servoManager):
        AbstractAction("Central drop ", robot),
        servoManager_(servoManager)
    {

    }

    /// @brief This function is called before choosing the action in the list,
    ///        giving the opportunity for an action to update its start position and priority.
    void updateStartCondition() override;

    void actionStartTrigger() override;

    bool performAction() override;

private:
    ServoManager *servoManager_;
};


#endif
