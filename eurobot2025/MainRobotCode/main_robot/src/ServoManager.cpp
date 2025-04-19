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


#define BANNER_ID 31

#define FRONT_CLAW_R 7
#define FRONT_CLAW_L 8
#define PLANK_WRIST 32
#define PLANK_CLAW 33


ServoManager::ServoManager():
    frontRightClaw_(RailServo(13, 23, 9500, true), 14, 15, 575, false),
    frontLeftClaw_(RailServo(10, 24, 9500, false), 12, 11, 825, true),
    backRail_(20, 20, 5500, true),
    frontPlankRail_(6, 21, 7100, false),
    frontCanRail_(5, 22, 7000, true, true)
{
}

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();
    servos_->setMode(0xFE, STS::Mode::POSITION);
    robot_->wait(0.001);
    servos_->setMode(0xFE, STS::Mode::POSITION);
    robot_->wait(0.001);
    frontRightClaw_.init(servos_);
    frontRightClaw_.rail_.init(servos_);
    frontLeftClaw_.init(servos_);
    frontLeftClaw_.rail_.init(servos_);
    backRail_.init(servos_);
    frontPlankRail_.init(servos_);
    frontCanRail_.init(servos_);

    frontRightClaw_.move(ClawPosition::FORWARD);
    frontRightClaw_.openClaw();
    frontLeftClaw_.move(ClawPosition::FORWARD);
    frontLeftClaw_.openClaw();

    servos_->setTargetPosition(FRONT_CLAW_R, 820);
    servos_->setTargetPosition(FRONT_CLAW_L, 520);

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
        servos_->setTargetPosition(FRONT_CLAW_R, 820);
        servos_->setTargetPosition(FRONT_CLAW_L, 520);
    }
}

void ServoManager::grab(bool const& front)
{
    if (front)
    {
        frontRightClaw_.closeClaw();
        frontLeftClaw_.closeClaw();

        grabPlank();
        servos_->setTargetPosition(FRONT_CLAW_R, 900);
        servos_->setTargetPosition(FRONT_CLAW_L, 440);
    }
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
    servos_->setTargetPosition(PLANK_WRIST, 1800);
    servos_->setTargetPosition(PLANK_CLAW, 2048);
}


