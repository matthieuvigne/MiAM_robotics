#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#include <miam_utils/raspberry_pi/RaspberryPi.h>

// Servo map:
// 5 : front rail, can (claws)
// 6 : front rail, plank
// 10: front left rail
// 11: front left claw
// 12: front left wrist
// 13: front right rail
// 14: front right claw
// 15: front right wrist
// 20: back rail
// 31: banner

// ??: front central claw, left
// ??: front central claw, right
// ??: back claw, left
// ??: back claw, right
// ??: plank, wrist
// ??: plank, claw

#define BANNER_ID 31

ServoManager::ServoManager():
    frontRightClaw_(RailServo(13, 23, 9500, true), 14, 15, 575, false),
    frontLeftClaw_(RailServo(10, 24, 9500, false), 12, 11, 825, true),
    backRail_(20, 20, 9500, false),
    frontPlankRail_(6, 21, 6000, false),
    frontCanRail_(5, 22, 6000, false)
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


