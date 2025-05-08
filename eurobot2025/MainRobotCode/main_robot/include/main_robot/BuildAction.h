#ifndef MAIN_ROBOT_BUILD_ACTION_H
#define MAIN_ROBOT_BUILD_ACTION_H

#include <string>
#include "main_robot/AbstractAction.h"
#include "main_robot/ServoManager.h"

class BuildAction: public AbstractAction
{
public:
    BuildAction(RobotInterface *robot, ServoManager *servoManager, int const& zoneId):
        AbstractAction("Building " + std::to_string(zoneId), robot),
        servoManager_(servoManager),
        zoneId_(zoneId)
    {
        largeZone_ = zoneId < 2;
        nDrop_ = 0;
    }

    /// @brief This function is called before choosing the action in the list,
    ///        giving the opportunity for an action to update its start position and priority.
    void updateStartCondition() override;

    void actionStartTrigger() override;

    bool performAction() override;

private:
    ServoManager *servoManager_;
    int zoneId_;

    bool largeZone_;
    int nDrop_;
};


#endif
