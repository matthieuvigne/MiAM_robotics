#include "main_robot/ServoManager.h"
#include "common/ThreadHandler.h"

#define TURRET_ID 10

void ServoManager::init(RobotInterface *robot)
{
    robot_ = robot;
    servos_ = robot->getServos();

    turretState_ = turret::state::CALIBRATING;

    std::thread th(&ServoManager::turretMotionThread, this);
    ThreadHandler::addThread(th);

    // Small servos need to be init - and this needs to be done severa times.
    for (int i = 0; i < 3; i++)
    {
      servos_->setMode(0xFE, STS::Mode::POSITION);
      robot_->wait(0.1);
    }
    openClaws(true);
    setClawPosition(ClawPosition::HIGH_POSITION);
}

void ServoManager::set_servos(STSServoDriver *servos)
{
  servos_ = servos;
  if(servos_ == NULL)
    std::cout << "Problem" << std::endl;
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

void ServoManager::setClawPosition(ClawPosition claw_position)
{
  switch(claw_position)
  {
    case ClawPosition::LOW_POSITION:
      servos_->setTargetPosition(11, 1625);
      robot_->wait(0.030);
      servos_->setTargetPosition(12, 3432);
      robot_->wait(0.030);
      break;
    case ClawPosition::MEDIUM_POSITION:
      servos_->setTargetPosition(11, 2335);
      robot_->wait(0.030);
      servos_->setTargetPosition(12, 2550);
      robot_->wait(0.030);
      break;
    case ClawPosition::HIGH_POSITION:
      servos_->setTargetPosition(11, 2695);
      robot_->wait(0.030);
      servos_->setTargetPosition(12, 2352);
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

    updateTurretPosition();
    targetTurretPosition_ = targetPosition;
    if (targetTurretPosition_ > currentTurretPosition_)
        turretState_ = turret::state::MOVING_CLOCKWISE;
    else
        turretState_ = turret::state::MOVING_COUNTER_CLOCKWISE;
        
    // Get current turret position
    std::cout << "TARGET POSITION " << targetPosition << std::endl;
    std::cout << "Current turret position = " << currentTurretPosition_ << std::endl;
    std::cout << "Target turret position = " << targetTurretPosition_ << std::endl;
    std::cout << "Current true position = " << servos_->getCurrentPosition(TURRET_ID) << std::endl;
        
    // Move the turrent to the target angle
    double turret_pos = 841 + (2.70*4096)*(targetPosition/360.);
    std::cout << "Target position " << int(turret_pos) << std::endl;
    servos_->setTargetPosition(TURRET_ID, int(turret_pos));
}

void ServoManager::waitForTurret()
{
    while (turretState_ != turret::state::IDLE)
        robot_->wait(0.010);
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


void ServoManager::raiseSolarPannelArm()
{
    servos_->setTargetPosition(20, 1200);
}
void ServoManager::lowerSolarPannelArm()
{
    servos_->setTargetPosition(20, 2048);
}