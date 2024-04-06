#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <vision/common/get_solar_panel_orientation.hpp>

#define TURRET_ID 10
#define TURRET_START_SWITCH_ID 23

#define SOLAR_PANEL_WHEEL 21

double const REDUCTION_RATIO = 23 / 60.0;

void ServoManager::init(RobotInterface *robot, bool const& isTurretAlreadyCalibrated)
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
    openClaws(false);
    setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
    setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);
    raiseSolarPanelArm();
    openElectromagnetArms();
    turnOffMagnets();
    servos_->setMode(SOLAR_PANEL_WHEEL, STS::Mode::VELOCITY);

    // Change P gain of the turret servo to prevent vibrations
    servos_->setPIDGains(TURRET_ID, 15, 5, 0);

    std::thread th(&ServoManager::turretMotionThread, this, isTurretAlreadyCalibrated);
    ThreadHandler::addThread(th);

}


int clawOpen[6] = {435, 445, 570, 665, 370, 405};
int clawDirection[6] = {-1, 1, 1, -1, 1, 1};

int const CLAW_MOTION = 90;

void ServoManager::openClaw(int const& clawId, bool const& halfOpen)
{
    int const idx = clawId - 2;
    if (idx < 6 && idx >= 0)
    {
        int const offset = (halfOpen ? 0.3 * CLAW_MOTION * clawDirection[idx] : 0);
        servos_->setTargetPosition(clawId, clawOpen[idx] + offset);
        isClawServoClosed_[idx] = false;
    }
}


void ServoManager::closeClaw(int const& clawId)
{
    int const idx = clawId - 2;

    if (idx < 6 && idx >= 0)
    {
        isClawServoClosed_[idx] = true;
        servos_->setTargetPosition(clawId, clawOpen[idx] + clawDirection[idx] * CLAW_MOTION);
    }
}

void ServoManager::openClaws(bool const& front, bool const& halfOpen)
{
    int offset = (front ? 2 : 5);
    for (int i = 0; i < 3; i++)
    {
        robot_->wait(0.005);
        openClaw(offset + i, halfOpen);
    }
}


void ServoManager::closeClaws(bool const& front)
{
    int offset = (front ? 2 : 5);
    for (int i = 0; i < 3; i++)
    {
        robot_->wait(0.005);
        closeClaw(offset + i);
    }
}

int ServoManager::updateClawContent(bool const& front, GameState & gameState)
{
    int offset = (front ? 0 : 3);


    int oldPlantNumber = 0;
    for (int j = 0; j < 3; j++)
        if (gameState.robotClawContent[offset + j] != ClawContent::EMPTY)
            oldPlantNumber++;


    for (int j = 0; j < 3; j++)
    {
        int const idx = offset + j;
        if (!isClawServoClosed_[idx])
        {
            gameState.robotClawContent[idx] = ClawContent::EMPTY;
        }
        else
        {
#ifdef SIMULATION
            gameState.robotClawContent[offset + j] = ClawContent::UNKNOWN_PLANT;
#else
            int const currentPosition = servos_->getCurrentPosition(2 + idx);
            int const targetClosePosition = clawOpen[idx] + clawDirection[idx] * CLAW_MOTION;
            if (std::abs(currentPosition - targetClosePosition) > 10)
            {
                gameState.robotClawContent[offset + j] = ClawContent::UNKNOWN_PLANT;
            }
            else
            {
                gameState.robotClawContent[offset + j] = ClawContent::EMPTY;
            }
#endif
        }
    }

    int newPlantNumber = 0;
    for (int j = 0; j < 3; j++)
        if (gameState.robotClawContent[offset + j] != ClawContent::EMPTY)
            newPlantNumber++;
    return newPlantNumber - oldPlantNumber;
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
            servos_->setTargetPosition(servoId + 1, 2432);
            robot_->wait(0.030);
            break;
        case ClawPosition::MEDIUM_POSITION:
            servos_->setTargetPosition(servoId, 2230);
            robot_->wait(0.030);
            servos_->setTargetPosition(servoId + 1, 1850);
            robot_->wait(0.030);
            break;
        case ClawPosition::HIGH_POSITION:
            servos_->setTargetPosition(servoId, 2755);
            robot_->wait(0.030);
            servos_->setTargetPosition(servoId + 1, 1352);
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


void ServoManager::turretMotionThread(bool const& isTurretAlreadyCalibrated)
{
#ifdef SIMULATION
    turretState_ = turret::state::IDLE;
    return;
#endif

    setClawPosition(ClawSide::FRONT, ClawPosition::HIGH_POSITION);
    setClawPosition(ClawSide::BACK, ClawPosition::HIGH_POSITION);
    robot_->wait(0.3);

    // Perform calibration
    if (isTurretAlreadyCalibrated)
    {
        servos_->setMode(TURRET_ID, STS::Mode::STEP);
        currentTurretPosition_ = 0.0;
        turretState_ = turret::state::IDLE;
    }
    else
    {
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

        double const deltaAngle = -3.5;
        int const nStep = static_cast<int>(-deltaAngle / REDUCTION_RATIO * 4096 / 2 / M_PI);
        servos_->setTargetPosition(TURRET_ID, nStep);
        robot_->wait(1.0);
        while (servos_->isMoving(TURRET_ID))
            robot_->wait(0.050);
        currentTurretPosition_ = 0.0;

        turretState_ = turret::state::IDLE;
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
    servos_->setTargetPosition(20, 800);
}

void ServoManager::lowerSolarPanelArm()
{
    servos_->setTargetPosition(20, 2048);
}

void ServoManager::spinSolarPanel(bool const& spin)
{
    servos_->setTargetVelocity(SOLAR_PANEL_WHEEL, (spin ? -1500 : 0));
    
    // [TODO] Add with camera
}

void ServoManager::openElectromagnetArms()
{
    servos_->setTargetPosition(30, 2048);
    servos_->setTargetPosition(31, 2048);
}

void ServoManager::closeElectromagnetArms()
{
    int const increment = 1300;
    servos_->setTargetPosition(30, 2048 + increment);
    servos_->setTargetPosition(31, 2048 - increment);
}

void ServoManager::turnOnMagnets()
{
    robot_->getMPC23008()->setOutputs(0xFF);
}

void ServoManager::turnOffMagnets()
{
    robot_->getMPC23008()->setOutputs(0x00);
}
