#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#define TURRET_ID 42

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();

    turretState_ = turret::state::CALIBRATING;

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
    // for (int j = 0; j < 3; j++)
    //     gameState.robotClawContent[offset + j] = ClawContent::UNKNOWN_PLANT;

    for (int j = 0; j < 3; j++)
    {
        // if (((double) rand() / (RAND_MAX)) > 0.5)
            gameState.robotClawContent[offset + j] = ClawContent::UNKNOWN_PLANT;
    }
}


void ServoManager::moveTurret(double const& targetPosition)
{
#ifdef SIMULATION
    currentTurretPosition_ = targetTurretPosition_;
    return;
#endif
    targetTurretPosition_ = targetPosition;
    if (targetTurretPosition_ > currentTurretPosition_)
        turretState_ = turret::state::MOVING_CLOCKWISE;
    else
        turretState_ = turret::state::MOVING_COUNTER_CLOCKWISE;
}

void ServoManager::waitForTurret()
{
    while (turretState_ != turret::state::IDLE)
        robot_->wait(0.001);
}


void ServoManager::turretMotionThread()
{
#ifdef SIMULATION
    turretState_ = turret::state::IDLE;
    return;
#endif

    // FIXME
    while(true)
    {
        turretState_ = turret::state::IDLE;
        usleep(50000);
    }
    return;
    // Perform calibration

    while (true)
    {
        robot_->wait(0.020);

        if (turretState_ == turret::state::MOVING_CLOCKWISE || turretState_ == turret::state::MOVING_COUNTER_CLOCKWISE)
        {
            int const turretMotionSign = (turretState_ == turret::state::MOVING_CLOCKWISE ? 1 : -1);

            if (turretMotionSign * (currentTurretPosition_ - targetTurretPosition_) > 0)
                servos_->setTargetVelocity(TURRET_ID, turretMotionSign * 2000);
            else
            {
                servos_->setTargetVelocity(TURRET_ID, 0);
                turretState_ = turret::state::IDLE;
            }
        }
        else
            servos_->setTargetVelocity(TURRET_ID, 0);

        updateTurretPosition();
    }
}

void ServoManager::updateTurretPosition()
{
    int pos = servos_->getCurrentPosition(TURRET_ID);

    double increment = (pos - lastTurretPosition_) / 4096 * 2 * M_PI;
    if (increment > M_PI)
        increment = increment - 2 * M_PI;
    if (increment < -M_PI)
        increment = increment + 2 * M_PI;

    currentTurretPosition_ += increment;

    lastTurretPosition_ = pos;
}
