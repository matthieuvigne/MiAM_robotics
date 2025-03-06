#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#include <miam_utils/raspberry_pi/RaspberryPi.h>


#define BANNER_ID 10


void ServoManager::init(RobotInterface *robot, bool const& isTurretAlreadyCalibrated)
{
    robot_ = robot;
    servos_ = robot->getServos();

    // TODO
}


void ServoManager::foldBanner()
{
    servos_->setTargetPosition(BANNER_ID, 1600);
}


void ServoManager::dropBanner()
{
    servos_->setTargetPosition(BANNER_ID, 2048);
}
