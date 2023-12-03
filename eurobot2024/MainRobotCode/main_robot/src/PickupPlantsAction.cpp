#include "main_robot/PickupPlantsAction.h"


// Radius of the circle for action start.
double const RADIUS = 200;

void PickupPlantsAction::updateStartCondition()
{
    // Start depends on robot current pose.
    RobotPosition currentPose = robot_->getMotionController()->getCurrentPosition();

    startPosition_ = PLANT_COLLECT_COORD[zoneId_];

    double const angle = std::atan2(startPosition_.y - currentPose.y, startPosition_.x - currentPose.x);
    startPosition_.x -= RADIUS * std::cos(angle);
    startPosition_.y -= RADIUS * std::sin(angle);
    startPosition_.theta = angle;
    priority_ = 1;

    // Can't do action if robot is full
    if (robot_->gameState_.nPlantsInRobot > 4)
        priority_ =-1;
}


void PickupPlantsAction::actionStartTrigger()
{
    // TODO: we should move turret in advance
}

bool PickupPlantsAction::performAction()
{
    // TODO: move turret

    // Grab three plants
    robot_->getMotionController()->goToStraightLine(PLANT_COLLECT_COORD[zoneId_], 0.5);

    robot_->gameState_.nPlantsPerZone[zoneId_] -= 3;
    robot_->gameState_.nPlantsInRobot += 3;


    // Action should not be done again
    return true;
}


