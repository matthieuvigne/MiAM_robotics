#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#define TURRET_ID = 42;

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();

    std::thread th(&ServoManager::turretMotionThread, this);
    ThreadHandler::addThread(th);
}


void ServoManager::openClaw(int const& clawId)
{
    servos_->setTargetPosition(clawId, 2048);
}


void ServoManager::closeClaw(int const& clawId)
{
    servos_->setTargetPosition(clawId, 1000);
}

void ServoManager::openClaws(bool const& front)
{
    openClaw(0);
    openClaw(1);
    openClaw(2);
}


void ServoManager::closeClaws(bool const& front)
{
    openClaw(3);
    openClaw(4);
    openClaw(5);
}

void ServoManager::updateClawContent(bool const& front, GameState & gameState)
{
    int offset = (front ? 0 : 3);

    // TODO: for now, grab is considered perfect
    for (int j = 0; j < 3; j++)
        gameState.robotClawContent[offset + j] = ClawContent::UNKNOWN_PLANT;
}


void ServoManager::moveTurret(double const& targetPosition)
{
    currentPosition_ = targetPosition;
}

void ServoManager::waitForTurret()
{
    while (turretState_ != turret::state::IDLE)
        robot_->wait(0.001);
}


void ServoManager::turretMotionThread()
{
    turretState_ = turret::state::IDLE;
    while (true)
    {
        robot_->wait(0.020);
        // TODO

    }

}
