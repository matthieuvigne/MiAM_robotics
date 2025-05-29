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
// 35: back plank, finger


#define BANNER_ID 31

#define FRONT_CLAW_R 7
#define FRONT_CLAW_L 8
#define FRONT_SIDE_CLAW_L 11
#define FRONT_SIDE_CLAW_R 15
#define BACK_CLAW_L 21
#define BACK_CLAW_R 22
#define PLANK_WRIST 32
#define PLANK_CLAW 33
#define BACK_PLANK_CLAW 34
#define BACK_PLANK_FINGER 35

#define FRONT_CLAW_RANGE_OPEN 230
#define FRONT_CLAW_RANGE_CLOSE 170
#define FC_R_FOLD 730
#define FC_L_FOLD (245 - 5)

#define BACK_CLAW_RANGE_OPEN 240
#define BACK_CLAW_RANGE_CLOSE 170
#define BC_L_FOLD 540
#define BC_R_FOLD 140

#define OFFSET_STEP 10

#define FC_R_DEFAULT_CLOSE FC_R_FOLD - FRONT_CLAW_RANGE_CLOSE
#define FC_L_DEFAULT_CLOSE FC_L_FOLD + FRONT_CLAW_RANGE_CLOSE

#define BC_L_DEFAULT_CLOSE BC_L_FOLD - BACK_CLAW_RANGE_CLOSE
#define BC_R_DEFAULT_CLOSE BC_R_FOLD + BACK_CLAW_RANGE_CLOSE

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

    foldClaws(true);
    releasePlank();
    foldBackPlank(true);
    // Disable finger
    servos_->writeRegister(BACK_PLANK_FINGER, STS::registers::TORQUE_SWITCH, 0);

    std::vector<RailServo*> rails({
        &frontRightClaw_.rail_,
        &frontLeftClaw_.rail_,
        &backRail_,
        &frontPlankRail_,
        &frontCanRail_});
    railManager_.start(rails);

    foldBanner();
}

void ServoManager::setRailsToInitPosition()
{
    frontRightClaw_.rail_.move(0.4);
    frontLeftClaw_.rail_.move(0.4);
    frontCanRail_.move(0.0);
    frontPlankRail_.move(0.0);
    backRail_.move(0.30);
    servos_->setTargetPosition(BANNER_ID, 1750);
}

void ServoManager::prepareGrab(bool const& front)
{
    if (front)
    {

        frontRightClaw_.move(ClawPosition::FORWARD);
        frontRightClaw_.openClaw();
        frontLeftClaw_.move(ClawPosition::FORWARD);
        frontLeftClaw_.openClaw();

        frontRightClaw_.rail_.move(0.05);
        frontLeftClaw_.rail_.move(0.05);
        frontCanRail_.move(0.0);
        frontPlankRail_.move(0.0);

        while (railManager_.areAnyMoving())
            robot_->wait(0.010);

        releasePlank();
        frontClawOpen();
    }
    else
    {
        backRail_.move(0.0);
        while (backRail_.isMoving())
            robot_->wait(0.010);
        backClawOpen();
        releaseBackPlank();
    }
}

void ServoManager::clawsToMoveConfiguration(bool const& front)
{
    if (front)
    {
        frontRightClaw_.rail_.move(0.40);
        frontLeftClaw_.rail_.move(0.40);
        frontCanRail_.move(0.0);
        frontPlankRail_.move(0.0);
        releasePlank();
        servos_->setTargetPosition(PLANK_WRIST, 2600);
        while (railManager_.areAnyMoving())
            robot_->wait(0.010);
        servos_->setTargetPosition(FRONT_CLAW_R, FC_R_FOLD);
        servos_->setTargetPosition(FRONT_CLAW_L, FC_L_FOLD);

        frontRightClaw_.move(ClawPosition::FOLDED);
        frontRightClaw_.foldClaw();
        frontLeftClaw_.move(ClawPosition::FOLDED);
        frontLeftClaw_.foldClaw();
    }
    else
    {
        servos_->setTargetPosition(BACK_CLAW_L, BC_L_FOLD);
        servos_->setTargetPosition(BACK_CLAW_R, BC_R_FOLD);
        releaseBackPlank();
    }
}

bool ServoManager::areBothFrontSideClawsFull()
{
    int rErr, lErr;
    bool const sR= frontRightClaw_.isClawFull(rErr);
    bool const sL= frontLeftClaw_.isClawFull(lErr);
    robot_->logger_  << "[areBothFrontSideClawsFull] Claw front side check" <<  rErr << " " << lErr << " success " << (sR && sL) <<  std::endl;
    return sR && sL;
}

bool ServoManager::grab(bool const& front, bool const& frontFullGrab)
{
    if (front)
    {
        // Grab the front towers
        frontRightClaw_.closeClaw();
        frontLeftClaw_.closeClaw();
        frontClawClose();
        // Grab the plank
        grabPlank();
        robot_->wait(0.1);
        if(!checkGrab(front) || (frontFullGrab && !areBothFrontSideClawsFull()))
        {
            releasePlank();
            return false;
        }
        robot_->wait(0.2);
        // Move the rail up
        frontPlankRail_.move(0.1);
        frontCanRail_.move(0.15);
        frontRightClaw_.rail_.move(0.1);
        frontLeftClaw_.rail_.move(0.1);
    }
    else
    {
        // Grab the back towers
        grabBackTwoPlanks();
        backClawClose();
        robot_->wait(0.3);
        if(!checkGrab(front))
        {
          releaseBackPlank();
          robot_->wait(0.2);
          return false;
        }
        robot_->wait(0.2);
        backRail_.move(0.1);
    }
    while (railManager_.areAnyMoving())
        robot_->wait(0.010);

    return true;
}

int ServoManager::countGrab(bool const& front)
{
#ifdef SIMULATION
    return true;
#endif
    int const MIN_TH = 7;
    int const MAX_TH = 75;

    int servoIds[2] = {BACK_CLAW_R, BACK_CLAW_L};
    int targetPosition[2] = {lastCloseTarget_back_R, lastCloseTarget_back_L};
    int defaultTargetPositions[2] = {BC_R_DEFAULT_CLOSE, BC_L_DEFAULT_CLOSE};
    if (front)
    {
        servoIds[0] = FRONT_CLAW_R;
        servoIds[1] = FRONT_CLAW_L;
        targetPosition[0] = lastCloseTarget_front_R;
        targetPosition[1] = lastCloseTarget_front_L;
        defaultTargetPositions[0] = FC_R_DEFAULT_CLOSE;
        defaultTargetPositions[1] = FC_L_DEFAULT_CLOSE;
    }

    int errors[2];
    int nGrab = 0;
    for (int i = 0; i < 2; i++)
    {
        int error = std::abs(servos_->getCurrentPosition(servoIds[i]) - targetPosition[i]);
        bool current_claw_is_success = error < MAX_TH && error > MIN_TH;

        int const MAX_ATTEMPT = 2;

        // If last target is not default, then do not attempt to compensate
        if (targetPosition[i] == defaultTargetPositions[i])
        {
            for (uint currentAttempt=0; currentAttempt < MAX_ATTEMPT; currentAttempt++)
            {
                if (current_claw_is_success)
                {
                    break;
                }

                std::cout << "[Servo manager] Grab check iter id=" << i << ", " << currentAttempt << std::endl;

                if (i == 0)
                {
                    targetPosition[i] += (front ? OFFSET_STEP : -OFFSET_STEP);
                }
                else if (i == 1)
                {
                    targetPosition[i] += (front ? -OFFSET_STEP : OFFSET_STEP);
                }

                robot_->logger_  << "[Grab check] " << (front ? "front " : "back ") << " servo nb " << i << " grabbing more current target" << targetPosition[i] << std::endl;
                servos_->setTargetPosition(servoIds[i], targetPosition[i]);
                robot_->wait(0.100);
                error = std::abs(servos_->getCurrentPosition(servoIds[i]) - targetPosition[i]);
                // TODO here we are grabbing even more if error > MAX_TH! we should avoid that
                current_claw_is_success = error < MAX_TH && error > MIN_TH;
            }
        }

        if (current_claw_is_success)
            nGrab ++;
        errors[i] = error;
    }

    if (front)
    {
        lastCloseTarget_front_R = targetPosition[0];
        lastCloseTarget_front_L = targetPosition[1];
    }
    else
    {
        lastCloseTarget_back_R = targetPosition[0];
        lastCloseTarget_back_L = targetPosition[1];
    }

    robot_->logger_  << "[Grab check] " << (front ? "front " : "back ") <<  errors[0] << " " << errors[1] << " number grabbed: " << nGrab << std::endl;
    return nGrab;
}

bool ServoManager::checkGrab(bool const& front)
{
    return countGrab(front) == 2;
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
void ServoManager::raiseFrontSideClaws()
{
    frontRightClaw_.rail_.move(0.95);
    frontLeftClaw_.rail_.move(0.95);
    robot_->wait(0.5);
    return;
}


bool ServoManager::buildFrontTower()
{
    if(!areBothFrontSideClawsFull() || !checkGrab(true))
    {
        frontRightClaw_.openClaw();
        frontLeftClaw_.openClaw();
        frontClawOpen();
        releasePlank();
        return false;
    }

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
    robot_->wait(0.4);
    frontRightClaw_.rail_.move(0.91);
    frontLeftClaw_.rail_.move(0.86);
    frontCanRail_.move(0.0);
    while (frontRightClaw_.rail_.isMoving() || frontLeftClaw_.rail_.isMoving())
        robot_->wait(0.05);

    frontRightClaw_.move(ClawPosition::FORWARD);
    frontLeftClaw_.move(ClawPosition::FORWARD);
    robot_->wait(0.8);

    frontRightClaw_.openClaw();
    frontLeftClaw_.openClaw();

    // Drop plank
    frontPlankRail_.move(0.85);
    while (frontPlankRail_.isMoving())
        robot_->wait(0.05);
    frontClawOpen();
    releasePlank();
    robot_->wait(0.3);
    return true;
}

void ServoManager::frontClawOpen()
{
    servos_->setTargetPosition(FRONT_CLAW_R, FC_R_FOLD - FRONT_CLAW_RANGE_OPEN);
    servos_->setTargetPosition(FRONT_CLAW_L, FC_L_FOLD + FRONT_CLAW_RANGE_OPEN);
}

void ServoManager::frontClawClose()
{
    servos_->setTargetPosition(FRONT_CLAW_R, FC_R_DEFAULT_CLOSE);
    servos_->setTargetPosition(FRONT_CLAW_L, FC_L_DEFAULT_CLOSE);
    lastCloseTarget_front_R = FC_R_DEFAULT_CLOSE;
    lastCloseTarget_front_L = FC_L_DEFAULT_CLOSE;
}


void ServoManager::backClawOpen()
{
    servos_->setTargetPosition(BACK_CLAW_L, BC_L_FOLD - BACK_CLAW_RANGE_OPEN);
    servos_->setTargetPosition(BACK_CLAW_R, BC_R_FOLD + BACK_CLAW_RANGE_OPEN);
}

void ServoManager::backClawClose()
{
    servos_->setTargetPosition(BACK_CLAW_L, BC_L_DEFAULT_CLOSE);
    servos_->setTargetPosition(BACK_CLAW_R, BC_R_DEFAULT_CLOSE);
    lastCloseTarget_back_L = BC_L_DEFAULT_CLOSE;
    lastCloseTarget_back_R = BC_R_DEFAULT_CLOSE;
}

void ServoManager::foldClaws(bool setup)
{
    servos_->setTargetPosition(FRONT_CLAW_R, FC_R_FOLD);
    servos_->setTargetPosition(FRONT_CLAW_L, FC_L_FOLD);
    // Banner
    if (setup)
        servos_->setTargetPosition(BACK_CLAW_L, BC_L_FOLD - 50);
    else
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
    robot_->wait(0.3);
    servos_->setTargetPosition(PLANK_CLAW, 1900);
}

void ServoManager::releasePlank()
{
    servos_->setTargetPosition(PLANK_WRIST, 2200);
    servos_->setTargetPosition(PLANK_CLAW, 2300);
}

void ServoManager::foldPlank()
{
    servos_->setTargetPosition(PLANK_WRIST, 2800);
    servos_->setTargetPosition(PLANK_CLAW, 2048);
}

#define BACK_PLANK_STRAIGHT 280
#define BACK_FINGER_CLOSE 800

void ServoManager::grabBackTwoPlanks()
{
    servos_->setTargetPosition(BACK_PLANK_CLAW, BACK_PLANK_STRAIGHT); // 300
    servos_->setTargetPosition(BACK_PLANK_FINGER, BACK_FINGER_CLOSE - 270); // 300
}

void ServoManager::grabBackOnePlank()
{
    grabBackTwoPlanks();
    // servos_->setTargetPosition(BACK_PLANK_CLAW, 292);
}

void ServoManager::releaseBackPlank()
{
    servos_->setTargetPosition(BACK_PLANK_CLAW, BACK_PLANK_STRAIGHT - 25);
    servos_->setTargetPosition(BACK_PLANK_FINGER, BACK_FINGER_CLOSE - 400);
}

void ServoManager::foldBackPlank(bool init)
{
    servos_->setTargetPosition(BACK_PLANK_CLAW, (init ? 100 : 0) + BACK_PLANK_STRAIGHT);
    servos_->setTargetPosition(BACK_PLANK_FINGER, BACK_FINGER_CLOSE);
}


