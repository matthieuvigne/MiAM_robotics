#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#include <miam_utils/raspberry_pi/RaspberryPi.h>

// Servo map:
#define ID_RAIL_X 10
#define ID_RAIL_Y 11
#define ID_ARM_1 12
#define ID_ARM_2 13
#define ID_ARM_3 14
#define ID_HAND_ROT 15
#define ID_HAND_TRIGHT 16
#define ID_HAND_TLEFT 17
#define ID_BED 18
#define ID_FINGER_R 19
#define ID_FINGER_L 20
#define ID_CURSOR 21

ServoManager::ServoManager()
{
}

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();

    servos_->setMode(ID_ARM_1,      STS::Mode::POSITION);
    servos_->setMode(ID_ARM_2,      STS::Mode::POSITION);
    servos_->setMode(ID_ARM_3,      STS::Mode::POSITION);
    servos_->setMode(ID_HAND_ROT,   STS::Mode::POSITION);
    servos_->setMode(ID_HAND_TRIGHT,STS::Mode::POSITION);
    servos_->setMode(ID_HAND_TLEFT, STS::Mode::POSITION);
    servos_->setMode(ID_BED,        STS::Mode::POSITION);
    servos_->setMode(ID_FINGER_R,   STS::Mode::POSITION);
    servos_->setMode(ID_FINGER_L,   STS::Mode::POSITION);
    servos_->setMode(ID_CURSOR,     STS::Mode::POSITION);

    // Setup rails
    railX_ = servos_->createRail(ID_RAIL_X, 6, 5500, true);
    railY_ = servos_->createRail(ID_RAIL_Y, 25, 4400, false);


    cursorFold();
    bedFold();
    moveArm(ArmPosition::CALIBRATE);
    // Start calib
    servos_->startRailCalibration();
}

void ServoManager::moveRails(RailPosition const& position)
{
    switch(position)
    {
        case RailPosition::STARTUP:
            railX_->move(0.0);
            railY_->move(0.0);
            break;
        default: break;
    }
}

bool ServoManager::areRailsMoving()
{
    return railX_->isMoving() || railY_->isMoving();
}

void ServoManager::cursorFold()
{
    servos_->setTargetPosition(ID_CURSOR, 2048);
}
void ServoManager::cursorUnfold()
{
    servos_->setTargetPosition(ID_CURSOR, 3200);
}


void ServoManager::bedFold()
{
    servos_->setTargetPosition(ID_BED, 2048);
}
void ServoManager::bedUnfold()
{
    servos_->setTargetPosition(ID_BED, 4080);
}

void ServoManager::moveArm(ArmPosition const& position)
{
    switch(position)
    {
        case ArmPosition::CALIBRATE:
            servos_->setTargetPosition(ID_ARM_1, 2048);
            servos_->setTargetPosition(ID_ARM_2, 1600);
            servos_->setTargetPosition(ID_ARM_3, 2400);
            break;
        default: break;
    }
}
