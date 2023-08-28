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
int const START_SWITCH = 20;

// Motor control parameters
double const Kp = 0.7;
double const Ki = 0.9;
double const maxOutput = 20;
double const filterCutoff = 10.0;
double const maxFeedforward = 0.4;


Robot::Robot(RobotParameters const& parameters, AbstractStrategy *strategy, RobotGUI *gui, bool const& testMode, bool const& disableLidar):
    RobotInterface(parameters, gui, strategy, testMode),
    lidar_(parameters.lidarOffset),
    disableLidar_(disableLidar),
    spiMotor_(RPI_SPI_00, 1000000),
    mcp_(&spiMotor_),
    motors_(&mcp_),
    rightController_(&motors_, parameters.rightMotorId, Kp, Ki, maxOutput, filterCutoff, maxFeedforward, parameters.maxWheelAcceleration / parameters.wheelRadius),
    leftController_(&motors_, parameters.leftMotorId, Kp, Ki, maxOutput, filterCutoff, maxFeedforward, parameters.maxWheelAcceleration / parameters.wheelRadius),
    spiEncoder_(RPI_SPI_01, 1000000),
    encoders_(&spiEncoder_, 2)
{
    guiState_.state = robotstate::INIT;
    guiState_.score = 0;

    lastEncoderPosition_.push_back(0);
    lastEncoderPosition_.push_back(0);
}


void Robot::stopMotors()
{
    motors_.setCurrent(motionController_.robotParams_.rightMotorId, 0);
    motors_.setCurrent(motionController_.robotParams_.leftMotorId, 0);
}


bool Robot::initSystem()
{
    RPi_setupGPIO(START_SWITCH, PI_GPIO_INPUT_PULLUP); // Starting cable.
    guiState_.debugStatus = "";

    // Motor init
    if (!isMCPInit_)
    {
        isMCPInit_ = mcp_.init();
        if (!isMCPInit_)
            guiState_.debugStatus += "MCP init failed\n";
    }

    if (isMCPInit_)
    {
        if (!isMotorsInit_)
        {
            isMotorsInit_ = true;
            if (!motors_.init(motionController_.robotParams_.rightMotorId, 0.300))
            {
                guiState_.debugStatus += "Right motor init failed\n";
                isMotorsInit_ = false;
            }
            if (!motors_.init(motionController_.robotParams_.leftMotorId, 0.300))
            {
                guiState_.debugStatus += "Left motor init failed\n";
                isMotorsInit_ = false;
            }
        }
    }

    // Encoder init
    if (!isEncodersInit_)
    {
        bool inaInit = ina226_.init(&RPI_I2C);

        isEncodersInit_ = encoders_.init() & inaInit;
        if (!isEncodersInit_)
            guiState_.debugStatus += "Encoders init failed\n";
        else
            lastEncoderPosition_ = encoders_.updatePosition();
    }

    if (!isServoInit_)
    {
        isServoInit_ = servos_.init("/dev/ttyAMA0", 18);
        if (!isServoInit_)
            guiState_.debugStatus += "Servo init failed\n";
    }

    if (!isLidarInit_ && !disableLidar_)
    {
        isLidarInit_ = lidar_.init("/dev/RPLIDAR", motionController_.robotParams_.lidarNPointsPerTurn);
        if (!isLidarInit_)
            guiState_.debugStatus += "Lidar init failed\n";
    }

    return isMCPInit_ & isMotorsInit_ & isEncodersInit_ & isServoInit_ & (isLidarInit_ || disableLidar_);
}


void Robot::wait(double const& waitTimeS)
{
    usleep(static_cast<int>(1e6 * waitTimeS));
}

void Robot::updateSensorData()
{
    std::vector<double> encoderPosition = encoders_.updatePosition();
    int const pR = motionController_.robotParams_.rightEncoderId;
    int const pL = motionController_.robotParams_.leftEncoderId;
    encoderPosition[pR] *= motionController_.robotParams_.rightEncoderDirection;
    encoderPosition[pL] *= motionController_.robotParams_.leftEncoderDirection;
    measurements_.drivetrainMeasurements.encoderPosition[side::RIGHT] = encoderPosition[pR];
    measurements_.drivetrainMeasurements.encoderPosition[side::LEFT] = encoderPosition[pL];
    measurements_.drivetrainMeasurements.encoderSpeed.right = encoderPosition[pR] - lastEncoderPosition_[pR];
    measurements_.drivetrainMeasurements.encoderSpeed.left = encoderPosition[pL] - lastEncoderPosition_[pL];
    lastEncoderPosition_ = encoderPosition;

    measurements_.batteryVoltage = 20.0; // Hack, for now: we don't have a good battery measurement system.
}

void Robot::applyMotorTarget(DrivetrainTarget const& target)
{
    static bool wasRunning = false;
    static bool panicMode = false;
    static double panicStartTime = 0.0;
    if (panicMode)
    {
        logger_ << "[Robot] Panic !" << std::endl;
        stopMotors();
        // If naturally stopped, exit panic.
        if (currentTime_ - panicStartTime > 100.0 && std::abs(target.motorSpeed[0]) < 0.001 && std::abs(target.motorSpeed[1]) < 0.001 )
        {
            panicMode = false;
            logger_ << "[Robot] Clearing panic state." << std::endl;
        }
    }
    else if (!hasMatchStarted_)
    {
        leftController_.stop(dt_);
        rightController_.stop(dt_);
    }
    else
    {
        if (std::abs(target.motorSpeed[0]) < 0.001 && std::abs(target.motorSpeed[1]) < 0.001 )
        {
            if (wasRunning)
                logger_ << "[Robot] Motors stopping" << std::endl;
            wasRunning = false;
            rightController_.stop(dt_);
            measurements_.drivetrainMeasurements.motorSpeed(0) = 0.0;
            leftController_.stop(dt_);
            measurements_.drivetrainMeasurements.motorSpeed(1) = 0.0;
            logger_.log("MotorController.status", currentTime_, 0);
        }
        else
        {
            if (!wasRunning)
                logger_ << "[Robot] Motors running" << std::endl;
            wasRunning = true;
            int sign = motionController_.robotParams_.rightMotorDirection;
            measurements_.drivetrainMeasurements.motorSpeed(0) = sign * rightController_.sendTarget(sign * target.motorSpeed[0], dt_);
            sign = motionController_.robotParams_.leftMotorDirection;
            measurements_.drivetrainMeasurements.motorSpeed(1) = sign * leftController_.sendTarget(sign * target.motorSpeed[1], dt_);
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
    motors_.mutex_.lock();
    if (isLidarInit_)
        lidar_.stop();
    servos_.disable(0xFE);
    strategy_->shutdown();
}

