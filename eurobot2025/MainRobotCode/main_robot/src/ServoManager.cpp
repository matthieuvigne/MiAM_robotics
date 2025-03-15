#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#include <miam_utils/raspberry_pi/RaspberryPi.h>


#define BANNER_ID 31


void ServoManager::init(RobotInterface *robot, bool const& isTurretAlreadyCalibrated)
{
    robot_ = robot;
    servos_ = robot->getServos();

    foldBanner();
}


void ServoManager::foldBanner()
{
    servos_->setTargetPosition(BANNER_ID, 2048);
}


void ServoManager::dropBanner()
{
    servos_->setTargetPosition(BANNER_ID, 3000);
}
