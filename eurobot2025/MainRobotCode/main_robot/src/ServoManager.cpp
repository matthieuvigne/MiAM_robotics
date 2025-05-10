#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#include <miam_utils/raspberry_pi/RaspberryPi.h>

// Servo map:
// 5 : front rail, can (claws)
// 6 : front rail, plank
// 7: front central claw, right
// 8: front central claw, left
// 10: front left rail
// 11: front left claw
// 12: front left wrist
// 13: front right rail
// 14: front right claw
// 15: front right wrist
// 20: back rail
// 21: back claw, left
// 22: back claw, right
// 31: banner
// 32: plank, wrist
// 33: plank, claw
// 34: back plank, claw


#define BANNER_ID 31

#define FRONT_CLAW_R 7
#define FRONT_CLAW_L 8
#define BACK_CLAW_L 21
#define BACK_CLAW_R 22
#define PLANK_WRIST 32
#define PLANK_CLAW 33
#define BACK_PLANK_CLAW 34


ServoManager::ServoManager():
    frontRightClaw_(RailServo(13, 23, 9500, true), 14, 15, 190, false), //0.9/* TODO check this value*/
    frontLeftClaw_(RailServo(10, 24, 9500, false), 12, 11, 730, true), //1.1
    backRail_(20, 20, 9000, true),
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
    frontRightClaw_.init(servos_);
    frontRightClaw_.rail_.init(servos_);
    frontLeftClaw_.init(servos_);
    frontLeftClaw_.rail_.init(servos_);
    backRail_.init(servos_);
    frontPlankRail_.init(servos_);
    frontCanRail_.init(servos_);

    frontRightClaw_.move(ClawPosition::FORWARD);
    frontRightClaw_.closeClaw();
    frontLeftClaw_.move(ClawPosition::FORWARD);
    frontLeftClaw_.closeClaw();

    foldClaws();
    releasePlank();
    releaseBackPlank();

    std::vector<RailServo*> rails({
        &frontRightClaw_.rail_,
        &frontLeftClaw_.rail_,
        &backRail_,
        &frontPlankRail_,
        &frontCanRail_});
    railManager_.start(rails);

    // Move banner servo to hold position
    servos_->setTargetPosition(BANNER_ID, 1850);
}

void ServoManager::setRailsToInitPosition()
{
    frontRightClaw_.rail_.move(0.6);
    frontLeftClaw_.rail_.move(0.6);
    frontCanRail_.move(0.0);
    frontPlankRail_.move(0.0);
    backRail_.move(0.25);
}

void ServoManager::prepareGrab(bool const& front)
{
    if (front)
    {

        frontRightClaw_.rail_.move(0.05);
        frontLeftClaw_.rail_.move(0.05);
        frontCanRail_.move(0.0);
        frontPlankRail_.move(0.0);

        frontRightClaw_.move(ClawPosition::FORWARD);
        frontRightClaw_.openClaw();
        frontLeftClaw_.move(ClawPosition::FORWARD);
        frontLeftClaw_.openClaw();

        releasePlank();
        frontClawClose();
    }
    else
    {
        backClawOpen();
        releaseBackPlank();
        backRail_.move(0.0);
    }
}

void ServoManager::grab(bool const& front)
{
    if (front)
    {
        frontRightClaw_.closeClaw();
        frontLeftClaw_.closeClaw();

        grabPlank();
        frontClawClose();
        robot_->wait(0.5);

        frontPlankRail_.move(0.1);
        frontCanRail_.move(0.15);
        frontRightClaw_.rail_.move(0.1);
        frontLeftClaw_.rail_.move(0.1);
    }
    else
    {
        backClawClose();
        robot_->wait(0.5);
        backRail_.move(0.1);
        grabBackPlank();
    }
    while (railManager_.areAnyMoving())
        robot_->wait(0.010);
}

void ServoManager::dropBackCans(bool ground)
{
    backRail_.move((ground ? 0.0: 0.8));
    while (backRail_.isMoving())
        robot_->wait(0.050);
    backClawOpen();
    releaseBackPlank();
    robot_->wait(0.3);
}

void ServoManager::buildFrontTower()
{
    // Position rails
    frontPlankRail_.move(1.0);
    frontCanRail_.move(0.05);
    frontLeftClaw_.rail_.move(0.02);
    frontRightClaw_.rail_.move(0.02);
    while (frontRightClaw_.rail_.isMoving())
        robot_->wait(0.05);

    // Move side cans
    frontRightClaw_.move(ClawPosition::SIDE);
    frontLeftClaw_.move(ClawPosition::SIDE);
    robot_->wait(0.5);
    frontRightClaw_.rail_.move(0.95);
    frontLeftClaw_.rail_.move(0.95);
    while (frontRightClaw_.rail_.isMoving() || frontLeftClaw_.rail_.isMoving())
        robot_->wait(0.05);

    frontRightClaw_.move(ClawPosition::FORWARD);
    frontLeftClaw_.move(ClawPosition::FORWARD);
    robot_->wait(0.7);
    frontRightClaw_.openClaw();
    frontLeftClaw_.openClaw();

    // Drop plank
    frontPlankRail_.move(0.85);
    frontCanRail_.move(0.0);
    while (frontPlankRail_.isMoving() || frontCanRail_.isMoving())
        robot_->wait(0.05);
    frontClawOpen();
    releasePlank();
}

#define FRONT_CLAW_RANGE_OPEN 230
#define FRONT_CLAW_RANGE_CLOSE 150
#define FC_R_FOLD 1020
#define FC_L_FOLD 240

#define BACK_CLAW_RANGE_OPEN 230
#define BACK_CLAW_RANGE_CLOSE 120
#define BC_L_FOLD 540
#define BC_R_FOLD 250

void ServoManager::frontClawOpen()
{
    servos_->setTargetPosition(FRONT_CLAW_R, FC_R_FOLD - FRONT_CLAW_RANGE_OPEN);
    servos_->setTargetPosition(FRONT_CLAW_L, FC_L_FOLD + FRONT_CLAW_RANGE_OPEN);
}

void ServoManager::frontClawClose()
{
    servos_->setTargetPosition(FRONT_CLAW_R, FC_R_FOLD - FRONT_CLAW_RANGE_CLOSE);
    servos_->setTargetPosition(FRONT_CLAW_L, FC_L_FOLD + FRONT_CLAW_RANGE_CLOSE);
}


void ServoManager::backClawOpen()
{
    servos_->setTargetPosition(BACK_CLAW_L, BC_L_FOLD - BACK_CLAW_RANGE_OPEN);
    servos_->setTargetPosition(BACK_CLAW_R, BC_R_FOLD + BACK_CLAW_RANGE_OPEN);
}

void ServoManager::backClawClose()
{
    servos_->setTargetPosition(BACK_CLAW_L, BC_L_FOLD - BACK_CLAW_RANGE_CLOSE);
    servos_->setTargetPosition(BACK_CLAW_R, BC_R_FOLD + BACK_CLAW_RANGE_CLOSE);
}

void ServoManager::foldClaws()
{
    servos_->setTargetPosition(FRONT_CLAW_R, FC_R_FOLD);
    servos_->setTargetPosition(FRONT_CLAW_L, FC_L_FOLD);
    servos_->setTargetPosition(BACK_CLAW_L, BC_L_FOLD);
    servos_->setTargetPosition(BACK_CLAW_R, BC_R_FOLD);
}

void ServoManager::foldBanner()
{
    servos_->setTargetPosition(BANNER_ID, 2048);
}

void ServoManager::dropBanner()
{
    servos_->setTargetPosition(BANNER_ID, 1000);
}

void ServoManager::grabPlank()
{
    servos_->setTargetPosition(PLANK_WRIST, 2048);
    servos_->setTargetPosition(PLANK_CLAW, 1900);
}

void ServoManager::releasePlank()
{
    servos_->setTargetPosition(PLANK_WRIST, 2200);
    servos_->setTargetPosition(PLANK_CLAW, 2300);
}

void ServoManager::foldPlank()
{
    servos_->setTargetPosition(PLANK_WRIST, 3000);
    servos_->setTargetPosition(PLANK_CLAW, 2048);
}

void ServoManager::grabBackPlank()
{
    // TODO
    // servos_->setTargetPosition(BACK_PLANK_CLAW, 500);
}

void ServoManager::releaseBackPlank()
{
    // TODO
    // servos_->setTargetPosition(BACK_PLANK_CLAW, 500);
}


