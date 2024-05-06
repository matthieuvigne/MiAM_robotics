#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#include <miam_utils/raspberry_pi/RaspberryPi.h>

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


// Servo: front, back ; right to left when in back position.
int const clawToServoId[6] = {2, 4, 3, 5, 7, 6};
int const clawOpen[6] = {435, 570, 455, 665, 405, 370};
int const clawDirection[6] = {-1, 1, 1, -1, 1, 1};

int const CLAW_MOTION = 100;

void ServoManager::openClaw(int const& idx, bool const& halfOpen)
{
    if (idx < 6 && idx >= 0)
    {
        int const offset = (halfOpen ? 0.4 * CLAW_MOTION * clawDirection[idx] : 0);
        servos_->setTargetPosition(clawToServoId[idx], clawOpen[idx] + offset);
        isClawServoClosed_[idx] = false;
    }
}


void ServoManager::closeClaw(int const& idx)
{
    if (idx < 6 && idx >= 0)
    {
        isClawServoClosed_[idx] = true;
        servos_->setTargetPosition(clawToServoId[idx], clawOpen[idx] + clawDirection[idx] * CLAW_MOTION);
    }
}

void ServoManager::openClaws(bool const& front, bool const& halfOpen)
{
    int const offset = (front ? 0 : 3);
    for (int i = 0; i < 3; i++)
    {
        robot_->wait(0.005);
        openClaw(offset + i, halfOpen);
    }
}

void ServoManager::openAvailableClaws(bool const& front, GameState & gameState)
{
    int const offset = (front ? 0 : 3);
    for (int i = 0; i < 3; i++)
    {
        if (gameState.robotClawContent[offset + i] == ClawContent::EMPTY)
        {
            openClaw(offset + i, false);
            robot_->wait(0.005);
        }
    }
}


void ServoManager::closeClaws(bool const& front)
{
    int const offset = (front ? 0 : 3);
    for (int i = 0; i < 3; i++)
    {
        robot_->wait(0.005);
        closeClaw(offset + i);
    }
}

int ServoManager::updateClawContent(bool const& front, GameState & gameState)
{
    int const offset = (front ? 0 : 3);

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
            int const currentPosition = servos_->getCurrentPosition(clawToServoId[idx]);
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

bool ServoManager::setClawPosition(ClawSide const& side, ClawPosition const& claw_position)
{
    int servoId = (side == ClawSide::FRONT ? 11 : 13);
    switch(claw_position)
    {
        case ClawPosition::LOW_POSITION:
        {
            int const firstTarget = 1725;
            int const secondTarget = 2332;

            servos_->setTargetPosition(servoId, firstTarget);
            robot_->wait(0.100);
            servos_->setTargetPosition(servoId + 1, secondTarget);
            robot_->wait(0.150);
            servos_->disable(servoId);
            robot_->wait(0.650);
            // Re-enable motor is no plant is present
            if (servos_->getCurrentPosition(servoId) < firstTarget + 50)
                servos_->setTargetPosition(servoId, firstTarget);
            else
                return false;
            break;
        }
        case ClawPosition::MEDIUM_POSITION:
            // Unfold this arm first to avoid hitting bound
            servos_->setTargetPosition(servoId + 1, 1850);
            robot_->wait(0.010);
            servos_->setTargetPosition(servoId, 2230);
            robot_->wait(0.010);
            break;
        case ClawPosition::MEDIUM_POSITION_PLUS:
            servos_->setTargetPosition(servoId, 2280);
            robot_->wait(0.010);
            servos_->setTargetPosition(servoId + 1, 1840);
            robot_->wait(0.010);
            break;
        case ClawPosition::HIGH_POSITION:
            servos_->setTargetPosition(servoId, 2755);
            robot_->wait(0.010);
            //~ servos_->setTargetPosition(servoId + 1, 1352);
            servos_->setTargetPosition(servoId + 1, 1422);
            robot_->wait(0.010);
            break;
        case ClawPosition::TURN_POSITION:
            servos_->setTargetPosition(servoId, 2770);
            robot_->wait(0.010);
            servos_->setTargetPosition(servoId + 1, 1400);
            robot_->wait(0.010);
            break;
        case ClawPosition::START_POSITION:
            servos_->setTargetPosition(servoId, 2825);
            robot_->wait(0.010);
            servos_->setTargetPosition(servoId + 1, 1225);
            robot_->wait(0.010);
            break;
        default:
            std::cout << "Unknown target claw position." << std::endl;
    }
    return true;
}

// Position de dépose dans les pots
//~ 13: 2680
//~ 14: 1400

void ServoManager::moveTurret(double const& targetPosition)
{
#ifdef SIMULATION
    currentTurretPosition_ = targetTurretPosition_;
    return;
#endif
    // Don't move turret before it's finish moving.
    waitForTurret();
    raiseSolarPanelArm();
    setClawPosition(ClawSide::FRONT, ClawPosition::TURN_POSITION);
    setClawPosition(ClawSide::BACK, ClawPosition::TURN_POSITION);
    //~ setClawPosition(ClawSide::FRONT, ClawPosition::MEDIUM_POSITION_PLUS);
    //~ setClawPosition(ClawSide::BACK, ClawPosition::MEDIUM_POSITION_PLUS);

    double const deltaAngle = targetPosition - currentTurretPosition_;
    int const nStep = static_cast<int>(-deltaAngle / REDUCTION_RATIO * 4096 / 2 / M_PI);
    servos_->setTargetPosition(TURRET_ID, nStep);

    currentTurretPosition_ = targetPosition;
}

void ServoManager::waitForTurret()
{
    while (turretState_ == turret::state::CALIBRATING || servos_->isMoving(TURRET_ID))
        robot_->wait(0.050);
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


void ServoManager::raiseSolarPanelArm(bool const& mediumPosition)
{
    if (mediumPosition)
        servos_->setTargetPosition(20, 1700);
    else
        servos_->setTargetPosition(20, 800);
}

void ServoManager::lowerSolarPanelArm()
{
    servos_->setTargetPosition(20, 2048);
}

void ServoManager::spinSolarPanel(bool const& isPlayingRightSide)
{
    servos_->setMode(SOLAR_PANEL_WHEEL, STS::Mode::VELOCITY);
    robot_->wait(0.005);
    servos_->setTargetVelocity(SOLAR_PANEL_WHEEL, (isPlayingRightSide ? 1500 : -1500));
}

void ServoManager::stopSolarPanel()
{
    servos_->setTargetVelocity(SOLAR_PANEL_WHEEL, 0);
}


void ServoManager::openElectromagnetArms()
{
    servos_->setTargetPosition(30, 2048);
    servos_->setTargetPosition(31, 2048);
}

void ServoManager::halfOpenElectromagnetArms()
{
    int const increment = 300;
    servos_->setTargetPosition(30, 2048 + increment);
    servos_->setTargetPosition(31, 2048 - increment);
}

void ServoManager::closeElectromagnetArms()
{
    int const increment = 1300;
    servos_->setTargetPosition(30, 2048 + increment);
    servos_->setTargetPosition(31, 2048 - increment);
}

void ServoManager::turnOnMagnets()
{
    robot_->logger_ << "Turning on magnets" << std::endl;
    robot_->getMPC23008()->setOutputs(0xFF);
}

void ServoManager::turnOffMagnets()
{
    robot_->logger_ << "Turning off magnets" << std::endl;
    robot_->getMPC23008()->setOutputs(0x00);
}

void ServoManager::turnOffFrontMagnets()
{
    robot_->logger_ << "Turning off front" << std::endl;
    robot_->getMPC23008()->setOutputs(0b11101101);
}
