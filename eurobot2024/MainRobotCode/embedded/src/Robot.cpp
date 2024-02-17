/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3
#include <unistd.h>
#include <thread>
#include <iomanip>

#include "Robot.h"
#include "common/RobotGUI.h"
#include "common/AbstractStrategy.h"


/// @brief GPIO with the start switch.
int const START_SWITCH = 17;

Robot::Robot(RobotParameters const& parameters, AbstractStrategy *strategy, RobotGUI *gui, bool const& testMode, bool const& disableLidar):
    RobotInterface(parameters, gui, strategy, testMode),
    lidar_(parameters.lidarOffset),
    disableLidar_(disableLidar),
    rightMotor_(RPI_SPI_01),
    leftMotor_(RPI_SPI_00)
{
    guiState_.state = robotstate::INIT;
    guiState_.score = 0;
}


void Robot::stopMotors()
{
    rightMotor_.stop();
    leftMotor_.stop();
}


bool Robot::initSystem()
{
    RPi_setupGPIO(START_SWITCH, PI_GPIO_INPUT_PULLUP); // Starting cable.
    guiState_.debugStatus = "";

    // Motor init
    if (!isMotorsInit_ || !isEncodersInit_)
    {
        bool rightEncoderInit = false;
        bool leftEncoderInit = false;
        isMotorsInit_ = true;
        if (!rightMotor_.init(rightEncoderInit))
        {
            guiState_.debugStatus += "Right motor init failed\n";
            isMotorsInit_ = false;
        }
        if (!rightEncoderInit)
            guiState_.debugStatus += "Right encoder init failed\n";

        if (!leftMotor_.init(leftEncoderInit))
        {
            guiState_.debugStatus += "Left motor init failed\n";
            isMotorsInit_ = false;
        }
        if (!leftEncoderInit)
            guiState_.debugStatus += "Left encoder init failed\n";
        isEncodersInit_ = rightEncoderInit && leftEncoderInit;
    }

    // TODO
    if (!isINAInit_)
        isINAInit_ = ina226_.init(&RPI_I2C);

    if (!isServoInit_)
    {
        isServoInit_ = servos_.init("/dev/ttyAMA0", -1);
        if (!isServoInit_)
            guiState_.debugStatus += "Servo init failed\n";
    }

    if (!isLidarInit_ && !disableLidar_)
    {
        isLidarInit_ = lidar_.init("/dev/RPLIDAR", motionController_.robotParams_.lidarNPointsPerTurn);
        if (!isLidarInit_)
            guiState_.debugStatus += "Lidar init failed\n";
    }

    return isMotorsInit_ & isEncodersInit_ & isServoInit_ & (isLidarInit_ || disableLidar_);
}


void Robot::wait(double const& waitTimeS)
{
    usleep(static_cast<int>(1e6 * waitTimeS));
}

void Robot::updateSensorData()
{

    NautilusMeasurements rightMeasurements = rightMotor_.updateMeasurements();
    NautilusMeasurements leftMeasurements = leftMotor_.updateMeasurements();

    rightMeasurements.encoderPosition *= motionController_.robotParams_.rightEncoderDirection;
    leftMeasurements.encoderPosition *= motionController_.robotParams_.leftEncoderDirection;

    // Wheel asymmetry handling
    rightMeasurements.encoderPosition *= 1.0;
    leftMeasurements.encoderPosition *= 0.988;



    measurements_.drivetrainMeasurements.encoderSpeed.right = rightMeasurements.encoderPosition - measurements_.drivetrainMeasurements.encoderPosition[side::RIGHT];
    measurements_.drivetrainMeasurements.encoderSpeed.left = leftMeasurements.encoderPosition - measurements_.drivetrainMeasurements.encoderPosition[side::LEFT];

    measurements_.drivetrainMeasurements.motorSpeed[side::RIGHT] = motionController_.robotParams_.rightMotorDirection * rightMeasurements.motorVelocity;
    measurements_.drivetrainMeasurements.motorSpeed[side::LEFT] = motionController_.robotParams_.leftMotorDirection * leftMeasurements.motorVelocity;
    measurements_.drivetrainMeasurements.encoderPosition[side::RIGHT] = rightMeasurements.encoderPosition;
    measurements_.drivetrainMeasurements.encoderPosition[side::LEFT] = leftMeasurements.encoderPosition;

    measurements_.batteryVoltage = rightMeasurements.batteryVoltage;

    // Log
    if (currentTime_ > 0.0)
    {
        logger_.log("Robot.rightMotor.motorCurrent", currentTime_, rightMeasurements.motorCurrent);
        logger_.log("Robot.rightMotor.batteryVoltage", currentTime_, rightMeasurements.batteryVoltage);
        logger_.log("Robot.rightMotor.currentMode", currentTime_, rightMeasurements.currentMode);
        logger_.log("Robot.rightMotor.nCommunicationFailed", currentTime_, rightMeasurements.nCommunicationFailed);

        logger_.log("Robot.leftMotor.motorCurrent", currentTime_, leftMeasurements.motorCurrent);
        logger_.log("Robot.leftMotor.batteryVoltage", currentTime_, leftMeasurements.batteryVoltage);
        logger_.log("Robot.leftMotor.currentMode", currentTime_, leftMeasurements.currentMode);
        logger_.log("Robot.leftMotor.nCommunicationFailed", currentTime_, leftMeasurements.nCommunicationFailed);
    }
}

void Robot::applyMotorTarget(DrivetrainTarget const& target)
{
    static bool wasRunning = true;
    if (!hasMatchStarted_ || areMotorsLocked_)
    {
        rightMotor_.stop();
        leftMotor_.stop();
    }
    else
    {
        if (std::abs(target.motorSpeed[0]) < 0.001 && std::abs(target.motorSpeed[1]) < 0.001 )
        {
            if (wasRunning)
                logger_ << "[Robot] Motors stopping" << std::endl;
            wasRunning = false;
            rightMotor_.stop();
            leftMotor_.stop();
            logger_.log("MotorController.status", currentTime_, 0);
        }
        else
        {
            if (!wasRunning)
                logger_ << "[Robot] Motors running" << std::endl;
            wasRunning = true;
            int sign = motionController_.robotParams_.rightMotorDirection;
            rightMotor_.setTargetVelocity(sign * target.motorSpeed[0]);
            sign = motionController_.robotParams_.leftMotorDirection;
            leftMotor_.setTargetVelocity(sign * target.motorSpeed[1]);
            logger_.log("MotorController.status", currentTime_, 1);
        }
    }
}

void Robot::matchEnd()
{
    if (!testMode_ || !disableLidar_)
        lidar_.stop();
}

bool Robot::isStartingSwitchPluggedIn() const
{
    return RPi_readGPIO(START_SWITCH) == 0;
}

void Robot::shutdown()
{
    stopMotors();
    // Hack: lock motors to prevent override.
    areMotorsLocked_ = true;
    if (isLidarInit_)
        lidar_.stop();
    servos_.disable(0xFE);
    strategy_->shutdown();
    logger_.close();
}

