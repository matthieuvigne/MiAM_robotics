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

    // Setup rails
    railX_ = servos_->createRail(ID_RAIL_X, 6, 9600, true);
    railX_ = servos_->createRail(ID_RAIL_Y, 5, 9600, true);

    // Start calib
    servos_->startRailCalibration();

    cursorFold();
}

void ServoManager::cursorFold()
{
    servos_->setTargetPosition(ID_CURSOR, 2048);
}
void ServoManager::cursorUnfold()
{
    servos_->setTargetPosition(ID_CURSOR, 3200);
}
