#ifndef MAIN_ROBOT_DROP_PLANTS_ACTION_H
#define MAIN_ROBOT_DROP_PLANTS_ACTION_H

#include <string>
#include "main_robot/AbstractAction.h"
#include "main_robot/ServoManager.h"

class DropPlantsAction: public AbstractAction
{
public:
    /// @brief Action to pick up plants from a given zone.
    /// @param zoneCenter Center of the zone from which to pick-up the plants
    DropPlantsAction(RobotInterface *robot, ServoManager *servoManager, int const& zoneId):
        AbstractAction("Drop plants " + std::to_string(zoneId), robot),
        servoManager_(servoManager),
        zoneId_(zoneId)
    {}

    /// @brief This function is called before choosing the action in the list,
    ///        giving the opportunity for an action to update its start position and priority.
    void updateStartCondition() override;

    void actionStartTrigger() override;

    bool performAction() override;

private:
     ServoManager *servoManager_;
    int zoneId_;
};


#endif