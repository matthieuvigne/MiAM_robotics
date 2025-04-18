#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#include <miam_utils/raspberry_pi/RaspberryPi.h>


#define BANNER_ID 31

ServoManager::ServoManager():
    frontRightClaw_(RailServo(13, 23, 9500, true), 14, 15, 575, false),
    frontLeftClaw_(RailServo(10, 24, 9500, false), 12, 11, 825, true),
    backRail_(21, 22, 9500, false),
    frontPlankRail_(6, 21, 6000, false),
    frontCanRail_(5, 20, 6000, false)
{
}

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();

    std::vector<RailServo*> rails({
        &frontRightClaw_.rail_,
        &frontLeftClaw_.rail_,
        &backRail_,
        &frontPlankRail_,
        &frontCanRail_});
    railManager_.start(rails);
    foldBanner();
    foldPlank();
}


void ServoManager::foldBanner()
{
    servos_->setTargetPosition(BANNER_ID, 2048);
}


void ServoManager::dropBanner()
{
    servos_->setTargetPosition(BANNER_ID, 3000);
}


void ServoManager::grabPlank()
{
    // TODO
}


void ServoManager::releasePlank()
{
    // TODO
}


void ServoManager::foldPlank()
{
    // TODO
}


