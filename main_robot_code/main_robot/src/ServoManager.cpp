#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#include <miam_utils/raspberry_pi/RaspberryPi.h>

// Servo map:
#define ID_RAIL_X 10
#define ID_RAIL_Y 11
#define ID_ARM_1 12
#define ID_ARM_2 13
#define ID_ARM_3 14
#define ID_ARM_4 15
#define ID_CURSOR 20

ServoManager::ServoManager():
    frontRightClaw_(RailServo(13, 23, 9500, true), 14, 15, 195, false), //0.9/* TODO check this value*/
    frontLeftClaw_(RailServo(10, 24, 9500, false), 12, 11, 735, true), //1.1
    backRail_(20, 20, 9600, true),
    frontPlankRail_(6, 21, 8000, false),
    frontCanRail_(5, 22, 7800, true, true)
{
}

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();
    servos_->setMode(0xFE, STS::Mode::POSITION);
    robot_->wait(0.002);
    servos_->setMode(0xFE, STS::Mode::POSITION);
    robot_->wait(0.002);

    // Setup rails
    backRail_ = servos_->createRail(20, 20, 9600, true);
    frontPlankRail_ = servos_->createRail(6, 21, 8000, false);
    frontCanRail_ = servos_->createRail(5, 22, 7800, true, true);


}

void ServoManager::setRailsToInitPosition()
{
}

void ServoManager::prepareGrab(bool const& front)
{
}

void ServoManager::clawsToMoveConfiguration(bool const& front)
{
}

bool ServoManager::areBothFrontSideClawsFull()
{
}

bool ServoManager::grab(bool const& front, bool const& frontFullGrab)
{
    return true;
}

int ServoManager::countGrab(bool const& front)
{
    return 0;
}

bool ServoManager::checkGrab(bool const& front)
{
    return countGrab(front) == 2;
}

void ServoManager::dropBackCans(bool ground)
{
}
void ServoManager::raiseFrontSideClaws()
{
}


bool ServoManager::buildFrontTower()
{
    return true;
}

void ServoManager::frontClawOpen()
{
}

void ServoManager::frontClawClose()
{
}


void ServoManager::backClawOpen()
{
}

void ServoManager::backClawClose()
{
}

void ServoManager::foldClaws(bool setup)
{
}

void ServoManager::foldBanner()
{
}

void ServoManager::dropBanner()
{
}

void ServoManager::grabPlank()
{
}

void ServoManager::releasePlank()
{
}

void ServoManager::foldPlank()
{
}

void ServoManager::grabBackTwoPlanks()
{
}

void ServoManager::grabBackOnePlank()
{
}

void ServoManager::releaseBackPlank()
{
}

void ServoManager::foldBackPlank(bool init)
{
}


