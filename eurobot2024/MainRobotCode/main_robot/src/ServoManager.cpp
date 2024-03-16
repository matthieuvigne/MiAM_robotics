#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"
#include <miam_utils/raspberry_pi/RaspberryPi.h>

#define TURRET_ID 10
#define TURRET_START_SWITCH_ID 23

#define SOLAR_PANEL_WHEEL 21

double const REDUCTION_RATIO = 25 / 65.0;

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();

    RPi_setupGPIO(TURRET_START_SWITCH_ID, PI_GPIO_INPUT_PULLUP);

    turretState_ = turret::state::CALIBRATING;

    // Small servos need to be init - and this needs to be done severa times.
    for (int i = 0; i < 3; i++)
    {
      servos_->setMode(0xFE, STS::Mode::POSITION);
      robot_->wait(0.1);
    }
    openClaws(true);
    setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
    setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);
    raiseSolarPanelArm();
    servos_->setMode(SOLAR_PANEL_WHEEL, STS::Mode::VELOCITY);

    // Change P gain of the turret servo to prevent vibrations
    servos_->setPIDGains(TURRET_ID, 15, 5, 0);

    std::thread th(&ServoManager::turretMotionThread, this);
    ThreadHandler::addThread(th);

}


void ServoManager::openClaw(int const& clawId)
{
    switch(clawId)
    {
        case 2:
            servos_->setTargetPosition(clawId, 435);
            break;
        case 3:
            servos_->setTargetPosition(clawId, 445);
            break;
        case 4:
            servos_->setTargetPosition(clawId, 570);
            break;
        default:
            std::cout << "Failed to open claw." << std::endl;
    }
}


void ServoManager::closeClaw(int const& clawId)
{
    switch(clawId)
    {
        case 2:
            servos_->setTargetPosition(clawId, 365);
            break;
        case 3:
            servos_->setTargetPosition(clawId, 515);
            break;
        case 4:
            servos_->setTargetPosition(clawId, 640);
            break;
        default:
            std::cout << "Failed to close claw." << std::endl;
    }
}

void ServoManager::openClaws(bool const& front)
{
    std::cout << "openClaws" << std::endl;
    openClaw(2);
    robot_->wait(0.050);
    openClaw(3);
    robot_->wait(0.050);
    openClaw(4);
}


void ServoManager::closeClaws(bool const& front)
{
    closeClaw(2);
    robot_->wait(0.050);
    closeClaw(3);
    robot_->wait(0.050);
    closeClaw(4);
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

double ServoManager::getTurretPosition() const
{
  return currentTurretPosition_;
}

void ServoManager::setClawPosition(ClawSide const& side, ClawPosition const& claw_position)
{
    int servoId = (side == ClawSide::FRONT ? 11 : 13);
    switch(claw_position)
    {
        case ClawPosition::LOW_POSITION:
            servos_->setTargetPosition(servoId, 1625);
            robot_->wait(0.030);
            servos_->setTargetPosition(servoId + 1, 3432);
            robot_->wait(0.030);
            break;
        case ClawPosition::MEDIUM_POSITION:
            servos_->setTargetPosition(servoId, 2335);
            robot_->wait(0.030);
            servos_->setTargetPosition(servoId + 1, 2550);
            robot_->wait(0.030);
            break;
        case ClawPosition::HIGH_POSITION:
            servos_->setTargetPosition(servoId, 2755);
            robot_->wait(0.030);
            servos_->setTargetPosition(servoId + 1, 2352);
            robot_->wait(0.030);
            break;
        default:
            std::cout << "Unknown target claw position." << std::endl;
    }
}

void ServoManager::moveTurret(double const& targetPosition)
{
#ifdef SIMULATION
    currentTurretPosition_ = targetTurretPosition_;
    return;
#endif
    // Don't move turret before it's finish moving.
    waitForTurret();
    raiseSolarPanelArm();
    setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
    setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);

    double const deltaAngle = targetPosition - currentTurretPosition_;
    int const nStep = static_cast<int>(-deltaAngle / REDUCTION_RATIO * 4096 / 2 / M_PI);
    servos_->setTargetPosition(TURRET_ID, nStep);

    currentTurretPosition_ = targetPosition;
}

void ServoManager::waitForTurret()
{
    while (turretState_ == turret::state::CALIBRATING || servos_->isMoving(TURRET_ID))
        robot_->wait(0.050);
    // std::cout << "waitForTurret" << std::endl;

    // while (turretState_ != turret::state::IDLE)
    //     robot_->wait(0.010);
}


void ServoManager::turretMotionThread()
{
#ifdef SIMULATION
    turretState_ = turret::state::IDLE;
    return;
#endif

    setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
    setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);
    robot_->wait(0.3);

    // Perform calibration
    turretState_ = turret::state::CALIBRATING;
    servos_->setMode(TURRET_ID, STS::Mode::VELOCITY);

    // Hit fast, then slow
    while (RPi_readGPIO(TURRET_START_SWITCH_ID) == 1)
        servos_->setTargetVelocity(TURRET_ID, -1023);
    for (int i = 0; i <5; i++)
    {
        servos_->setTargetVelocity(TURRET_ID, 1000);
        robot_->wait(0.1);
    }
    robot_->wait(0.5);
    while (RPi_readGPIO(TURRET_START_SWITCH_ID) == 1)
        servos_->setTargetVelocity(TURRET_ID, -300);

    servos_->setTargetVelocity(TURRET_ID, 0);
    robot_->wait(0.050);
    servos_->setMode(TURRET_ID, STS::Mode::STEP);
    robot_->wait(0.050);

    double const deltaAngle = -3.47;
    int const nStep = static_cast<int>(-deltaAngle / REDUCTION_RATIO * 4096 / 2 / M_PI);
    servos_->setTargetPosition(TURRET_ID, nStep);
    robot_->wait(1.0);
    while (servos_->isMoving(TURRET_ID))
        robot_->wait(0.050);
    currentTurretPosition_ = 0.0;

    turretState_ = turret::state::IDLE;
    while (true)
    {
        // robot_->wait(0.020);
        // int pos = servos_->getCurrentPosition(TURRET_ID);

        // std::cout << "pos" << pos << std::endl;

        // updateTurretPosition();
        // std::cout << "isMoving" << servos_->isMoving(TURRET_ID) << std::endl;
        // robot_->log("ServoManager.turretAngle", currentTurretPosition_);
        // robot_->log("ServoManager.turretState", static_cast<int>(turretState_));
        // robot_->log("TurrentSwitch", RPi_readGPIO(TURRET_START_SWITCH_ID));
    }
    while (true)
    {
        // robot_->wait(0.020);

        // if (turretState_ == turret::state::MOVING_CLOCKWISE || turretState_ == turret::state::MOVING_COUNTER_CLOCKWISE)
        // {
        //     int const turretMotionSign = (turretState_ == turret::state::MOVING_CLOCKWISE ? 1 : -1);

        //     if (turretMotionSign * (currentTurretPosition_ - targetTurretPosition_) > 0)
        //         servos_->setTargetVelocity(TURRET_ID, turretMotionSign * 2000);
        //     else
        //     {
        //         servos_->setTargetVelocity(TURRET_ID, 0);
        //         turretState_ = turret::state::IDLE;
        //     }
        // }
        // else
        //     servos_->setTargetVelocity(TURRET_ID, 0);

        // updateTurretPosition();
    }
}

void ServoManager::updateTurretPosition()
{
    int pos = servos_->getCurrentPosition(TURRET_ID);

    std::cout << "pos" << pos << std::endl;
    double increment = -(pos - lastTurretPosition_) / 4096.0 * 2 * M_PI;
    if (increment > M_PI)
        increment = increment - 2 * M_PI;
    if (increment < -M_PI)
        increment = increment + 2 * M_PI;

    std::cout << currentTurretPosition_ << std::endl;


    currentTurretPosition_ += increment * REDUCTION_RATIO;

    lastTurretPosition_ = pos;
}


void ServoManager::raiseSolarPanelArm()
{
    servos_->setTargetPosition(20, 1200);
}

void ServoManager::lowerSolarPanelArm()
{
    servos_->setTargetPosition(20, 2048);
}

void ServoManager::spinSolarPanel(bool const& spin)
{
    servos_->setTargetVelocity(SOLAR_PANEL_WHEEL, (spin ? -1023 : 0));
}