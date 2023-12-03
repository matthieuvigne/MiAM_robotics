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

void ServoManager::moveTurret(double const&)
{

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
