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

    if (!isVlxInit_)
    {
        isVlxInit_ = vlxSensor_.init(&RPI_I2C);
        if (!isVlxInit_)
            guiState_.debugStatus += "VLX init failed\n";
        else
        {
            std::thread measureThread = std::thread(&Robot::updateRangeMeasurement, this);
            measureThread.detach();
        }
    }

    if (!isINAInit_)
    {
        isINAInit_ = ina226_.init(&RPI_I2C);
        if (!isINAInit_)
            guiState_.debugStatus += "Battery monitoring init failed\n";
    }

    if (!isMCPInit_)
    {
        isMCPInit_ = mcpIOExpander_.init(&RPI_I2C);
        if (!isMCPInit_)
            guiState_.debugStatus += "MCP23008 init failed\n";
    }

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

    return isMotorsInit_ & isEncodersInit_ & isServoInit_ & (isLidarInit_ || disableLidar_) & isINAInit_;
}


void Robot::wait(double const& waitTimeS)
{
    usleep(static_cast<int>(1e6 * waitTimeS));
}

void Robot::updateSensorData()
{
    NautilusMeasurements rightMeasurements = rightMotor_.updateMeasurements();
    NautilusMeasurements leftMeasurements = leftMotor_.updateMeasurements();

    if (rightMeasurements.currentMode == static_cast<uint16_t>(nautilus::Mode::Fault))
    {
        logger_ << "[ERROR] Right motor in fault: " << rightMotor_.getDebugStatus() << std::endl;
    }
    if (leftMeasurements.currentMode == static_cast<uint16_t>(nautilus::Mode::Fault))
    {
        logger_ << "[ERROR] Right motor in fault: " << leftMotor_.getDebugStatus() << std::endl;
    }

    rightMeasurements.encoderPosition *= motionController_.robotParams_.rightEncoderDirection;
    leftMeasurements.encoderPosition *= motionController_.robotParams_.leftEncoderDirection;

    WheelSpeed baseSpeed;

    WheelSpeed motorPosition;
    motorPosition.right = motionController_.robotParams_.rightMotorDirection * rightMeasurements.motorPosition;
    motorPosition.left = motionController_.robotParams_.leftMotorDirection * leftMeasurements.motorPosition;

    bool isCurrentEncoderValid = rightMeasurements.isEncoderPositionValid && leftMeasurements.isEncoderPositionValid;

    if (!isCurrentEncoderValid && !isEncoderInvalid_)
    {
        logger_ << "[Robot] Error: encoder invalid, fallback to motor reading" << std::endl;
        isEncoderInvalid_ = !isCurrentEncoderValid;
    }

    if (isEncoderInvalid_)
    {
        // Invalid encoder: use motor speed instead, using the drivetrain kinematics to fake the velocity.
        baseSpeed.right = motorPosition.right - lastMotorReading_.right;
        baseSpeed.left = motorPosition.left - lastMotorReading_.left;

        baseSpeed = motionController_.getKinematics().inverseKinematics(
            motionController_.getKinematics().forwardKinematics(baseSpeed, false), true);

        isEncoderInvalid_ = !isCurrentEncoderValid;
        if (!isEncoderInvalid_)
        {
            logger_ << "[Robot] Encoders are valid again, leaving fallback" << std::endl;
        }
    }
    else
    {
        baseSpeed.right = rightMeasurements.encoderPosition - lastEncoderReading_.right;
        baseSpeed.left = leftMeasurements.encoderPosition - lastEncoderReading_.left;
    }

    // Update state.
    isEncoderInvalid_ = !isCurrentEncoderValid;

    lastEncoderReading_.right = rightMeasurements.encoderPosition;
    lastEncoderReading_.left = leftMeasurements.encoderPosition;
    lastMotorReading_ = motorPosition;


    measurements_.drivetrainMeasurements.encoderSpeed = baseSpeed;

    measurements_.drivetrainMeasurements.motorSpeed.right = motionController_.robotParams_.rightMotorDirection * rightMeasurements.motorVelocity;
    measurements_.drivetrainMeasurements.motorSpeed.left = motionController_.robotParams_.leftMotorDirection * leftMeasurements.motorVelocity;
    measurements_.drivetrainMeasurements.encoderPosition.right = rightMeasurements.encoderPosition;
    measurements_.drivetrainMeasurements.encoderPosition.left = leftMeasurements.encoderPosition;

    INA226Reading inaReading = ina226_.read();

    measurements_.batteryVoltage = inaReading.voltage;

    if (!disableLidar_)
    {
        lidar_.update();
        measurements_.drivetrainMeasurements.lidarDetection = lidar_.detectedRobots_;
    }

    // Log
    if (currentTime_ > 0.0)
    {
        logger_.log("Robot.rightMotor.rawEncoder", currentTime_, rightMeasurements.encoderPosition);
        logger_.log("Robot.rightMotor.motorPosition", currentTime_, rightMeasurements.motorPosition);
        logger_.log("Robot.rightMotor.motorCurrent", currentTime_, rightMeasurements.motorCurrent);
        logger_.log("Robot.rightMotor.batteryVoltage", currentTime_, rightMeasurements.batteryVoltage);
        logger_.log("Robot.rightMotor.currentMode", currentTime_, rightMeasurements.currentMode);
        logger_.log("Robot.rightMotor.nCommunicationFailed", currentTime_, rightMeasurements.nCommunicationFailed);
        logger_.log("Robot.rightMotor.nEncoderInvalid", currentTime_, rightMotor_.nEncoderInvalid_);

        logger_.log("Robot.leftMotor.rawEncoder", currentTime_, leftMeasurements.encoderPosition);
        logger_.log("Robot.leftMotor.motorPosition", currentTime_, leftMeasurements.motorPosition);
        logger_.log("Robot.leftMotor.motorCurrent", currentTime_, leftMeasurements.motorCurrent);
        logger_.log("Robot.leftMotor.batteryVoltage", currentTime_, leftMeasurements.batteryVoltage);
        logger_.log("Robot.leftMotor.currentMode", currentTime_, leftMeasurements.currentMode);
        logger_.log("Robot.leftMotor.nCommunicationFailed", currentTime_, leftMeasurements.nCommunicationFailed);
        logger_.log("Robot.leftMotor.nEncoderInvalid", currentTime_, leftMotor_.nEncoderInvalid_);
        logger_.log("Robot.isEncoderInvalid", currentTime_, isEncoderInvalid_);

        logger_.log("Robot.battery.voltage", currentTime_, inaReading.voltage);
        logger_.log("Robot.battery.current", currentTime_, inaReading.current);
        logger_.log("Robot.battery.power", currentTime_, inaReading.power);
        logger_.log("Robot.vlxDistance", currentTime_, measurements_.vlxDistance);
    }
}

void Robot::applyMotorTarget(DrivetrainTarget const& target)
{
    static bool wasRunning = true;
    if (areMotorsLocked_)
    {
        rightMotor_.stop();
        leftMotor_.stop();
    }
    else if (!hasMatchStarted_)
    {
        if (gui_->getBlockMotors())
        {
            rightMotor_.stop();
            leftMotor_.stop();
        }
        else
        {
            rightMotor_.disable();
            leftMotor_.disable();
        }
    }
    else
    {
        if (std::abs(target.motorSpeed.right) < 0.001 && std::abs(target.motorSpeed.left) < 0.001 )
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
            rightMotor_.setTargetVelocity(sign * target.motorSpeed.right);
            sign = motionController_.robotParams_.leftMotorDirection;
            leftMotor_.setTargetVelocity(sign * target.motorSpeed.left);
            logger_.log("MotorController.status", currentTime_, 1);
        }
    }
}

void Robot::matchEnd()
{
    mcpIOExpander_.setOutputs(0);
    if (!testMode_ || !disableLidar_)
        lidar_.stop();
}

bool Robot::isStartingSwitchPluggedIn() const
{
    return RPi_readGPIO(START_SWITCH) == 0;
}

void Robot::shutdown()
{
    // No need to stop motors, they will stop by themselves.

    // Hack: lock motors to prevent override.
    areMotorsLocked_ = true;
    if (isLidarInit_)
        lidar_.stop();
    strategy_->shutdown();
    servos_.disable(0xFE);
    logger_.close();
    mcpIOExpander_.setOutputs(0);
}


void Robot::updateRangeMeasurement()
{
    pthread_setname_np(pthread_self(), "robot_vlx");

    // Offset from measurement to position of center of robot.
    // To update this: place the robot a fixed distance (e.g. 10cm) from
    // a flat surface, and look at the measurement value.
    // Don't forget to add robot width.
    // This offset thus integrates sensor position, sensor offset...
    int const OFFSET = 121;

    // Perform average of last N values.
    #define N_AVG 3
    int oldValues[N_AVG];
    for (int j = 0; j < N_AVG; j++)
        oldValues[j] = 0;
    while (true)
    {
        for (int j = 1; j < N_AVG; j++)
            oldValues[j - 1] = oldValues[j];
        oldValues[N_AVG - 1] = vlxSensor_.getMeasurement() + OFFSET;
        int average = 0;
        for (int j = 0; j < N_AVG; j++)
            average += oldValues[j];
        measurements_.vlxDistance = static_cast<double>(average) / N_AVG;
    }
}