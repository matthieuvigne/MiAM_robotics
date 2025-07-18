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

Robot::Robot(RobotParameters const& parameters, AbstractStrategy *strategy, RobotGUI *gui, bool const& testMode, bool const& disableLidar, bool const& silent):
    RobotInterface(parameters, gui, strategy, testMode, "", silent),
    lidar_(parameters.lidarOffset),
    disableLidar_(disableLidar),
    rightMotor_(RPI_SPI_00),
    leftMotor_(RPI_SPI_01)
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

    if (!isINAInit_)
    {
        isINAInit_ = ina226_.init(&RPI_I2C);
        if (!isINAInit_)
            guiState_.debugStatus += "Battery monitoring init failed\n";
    }

    if (isMotorsInit_ && !isServoInit_)
    {
        isServoInit_ = servos_.init("/dev/ttyAMA0", -1);
        if (!isServoInit_)
            guiState_.debugStatus += "Servo init failed\n";
    }

    // Only init lidar after motor, to make sure power supply is up.
    if (isMotorsInit_ && !isLidarInit_ && !disableLidar_)
    {
        isLidarInit_ = lidar_.init("/dev/RPLIDAR", motionController_.robotParams_.lidarNPointsPerTurn);
        if (!isLidarInit_)
            guiState_.debugStatus += "Lidar init failed\n";
    }
    // I2C init
    if (isI2CExpanderInit_)
    {
        if (!isINA7Init_)
        {
            isINA7Init_ = ina226_7V_.init(&RPI_I2C, 0x45, 0.010);
            if (!isINA7Init_)
                guiState_.debugStatus += "7V monitoring init failed\n";
        }
        if (!isINA12Init_)
        {
            isINA12Init_ = ina226_12V_.init(&RPI_I2C, 0x44, 0.010);
            if (!isINA12Init_)
                guiState_.debugStatus += "12V monitoring init failed\n";
        }
        if (!isIMUInit_)
        {
            isIMUInit_ = imu_.init(&RPI_I2C);
            if (!isIMUInit_)
                guiState_.debugStatus += "IMU init failed\n";
        }
    }
    else
    {
        isI2CExpanderInit_ = i2cExpander_.init(&RPI_I2C);
        if (!isI2CExpanderInit_)
            guiState_.debugStatus += "I2CExpander init failed\n";
        else
            i2cExpander_.setPorts(0xFF);
    }
    return isMotorsInit_ & isEncodersInit_ & isServoInit_ & (isLidarInit_ || disableLidar_) & isINAInit_ & isIMUInit_ & isINA12Init_ & isINA7Init_;
}


void Robot::wait(double const& waitTimeS)
{
    usleep(static_cast<int>(1e6 * waitTimeS));
}

void Robot::updateSensorData()
{
    NautilusMeasurements rightMeasurements = rightMotor_.updateMeasurements();
    NautilusMeasurements leftMeasurements = leftMotor_.updateMeasurements();

    if (rightMeasurements.currentMode != static_cast<uint16_t>(nautilus::Mode::Position) && rightMeasurements.currentMode != static_cast<uint16_t>(nautilus::Mode::Stopped) && rightMeasurements.currentMode != static_cast<uint16_t>(nautilus::Mode::Velocity))
    {
        logger_ << "[Warning] Right motor: " << rightMotor_.getDebugStatus() << std::endl;
    }
    if (leftMeasurements.currentMode != static_cast<uint16_t>(nautilus::Mode::Position) && rightMeasurements.currentMode != static_cast<uint16_t>(nautilus::Mode::Stopped) && leftMeasurements.currentMode != static_cast<uint16_t>(nautilus::Mode::Velocity))
    {
        logger_ << "[Warning] Right motor: " << leftMotor_.getDebugStatus() << std::endl;
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


    measurements_.drivetrainMeasurements.encoderPositionIncrement = baseSpeed;

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

    measurements_.drivetrainMeasurements.gyroscope = imu_.getGyroscopeReadings()(2);

    // Log
    // if (currentTime_ > 0.0)
    if (true)
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
        logger_.log("Servos.nFailed", currentTime_, servos_.nPosFailed_);


        INA226Reading inaReading = ina226_7V_.read();
        logger_.log("Robot.7V.voltage", currentTime_, inaReading.voltage);
        logger_.log("Robot.7V.current", currentTime_, inaReading.current);
        logger_.log("Robot.7V.power", currentTime_, inaReading.power);
        inaReading = ina226_12V_.read();
        logger_.log("Robot.12V.voltage", currentTime_, inaReading.voltage);
        logger_.log("Robot.12V.current", currentTime_, inaReading.current);
        logger_.log("Robot.12V.power", currentTime_, inaReading.power);

        // Eigen::Vector3f gyro = imu_.getGyroscopeReadings();
        // logger_.log("IMU.gyroX", currentTime_, gyro(0));
        // logger_.log("IMU.gyroY", currentTime_, gyro(1));
        // logger_.log("IMU.gyroZ", currentTime_, gyro(2));
    }

    if (!hasMatchStarted_ && !inBorderDetection_ && gui_->getAskedDetectBorders())
    {
        logger_ << "[Robot] Border detection started" << std::endl;
        std::thread measureThread = std::thread(&Robot::detectBorders, this);
        measureThread.detach();
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
    else if (!hasMatchStarted_ && !inBorderDetection_)
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
            // if (wasRunning)
            //     logger_ << "[Robot] Motors stopping" << std::endl;
            wasRunning = false;
            rightMotor_.stop();
            leftMotor_.stop();
            logger_.log("MotorController.status", currentTime_, 0);
        }
        else
        {
            // if (!wasRunning)
            //     logger_ << "[Robot] Motors running" << std::endl;
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
    shutdown();
    if (!testMode_ || !disableLidar_)
        lidar_.stop();
    servos_.disable(0xFE);

    // Show end button
    while (true)
    {
        if (isStartingSwitchPluggedIn())
        {
            guiState_.state = robotstate::MATCH_QUIT;
            gui_->update(guiState_);
            return;
        }
        wait(0.050);
    }
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
}


bool Robot::touchBorder()
{
    motionController_.goStraight(500, 0.1, tf::NO_WAIT_FOR_END);
    wait(1.0);
    bool still = false;
    while (!motionController_.isTrajectoryFinished())
    {
        if (std::abs(measurements_.drivetrainMeasurements.motorSpeed.right) > 1 && std::abs(measurements_.drivetrainMeasurements.motorSpeed.left) > 1)
            if (std::abs(measurements_.drivetrainMeasurements.encoderPositionIncrement.right / (1e-9 * ROBOT_UPDATE_PERIOD)) < 0.05 &&
                std::abs(measurements_.drivetrainMeasurements.encoderPositionIncrement.left / (1e-9 * ROBOT_UPDATE_PERIOD)) < 0.05)
            {
                motionController_.stopCurrentTrajectoryTracking();
                still = true;
            }
        }
    return still;
}

// static bool side = true;
void Robot::detectBorders()
{
    pthread_setname_np(pthread_self(), "robot_detectBorders");
    inBorderDetection_ = true;

    // motionController_.goStraight(-1000);
    // strategy_->testSquare(side, 500);
    // side = !side;
    // inBorderDetection_ = false;
    // return;

    if (!touchBorder())
    {
        logger_ << "[Robot] Failed to reach border, aborting!" << std::endl;
        inBorderDetection_ = false;
        return;
    }
    wait(0.5);
    double const CHASSIS = 140;
    RobotPosition pos(0, CHASSIS, -M_PI_2);
    motionController_.resetPosition(pos, false, true, true);

    // Touch other border
    motionController_.goStraight(-CHASSIS);
    motionController_.pointTurn(-M_PI_2);

    // Touch border
    if (!touchBorder())
    {
        logger_ << "[Robot] Failed to reach border, aborting!" << std::endl;
        inBorderDetection_ = false;
        return;
    }
    wait(0.5);
    pos = RobotPosition(CHASSIS, 0, M_PI);
    motionController_.resetPosition(pos, true, false, true);
    motionController_.goStraight(-CHASSIS);

    // Go to start position
    RobotPosition target = gui_->getStartPosition();
    target.y += CHASSIS;
    motionController_.goToStraightLine(target, 1.0, tf::IGNORE_END_ANGLE);

    motionController_.pointTurn(-M_PI_2);
    if (!touchBorder())
    {
        logger_ << "[Robot] Failed to reach border, aborting!" << std::endl;
        inBorderDetection_ = false;
        return;
    }
    RobotPosition currentPos = motionController_.getCurrentPosition();
    logger_ << "[Robot] Border position " << currentPos << std::endl;
    if (std::abs(currentPos.y - CHASSIS) < 30)
    {
        pos = RobotPosition(0, CHASSIS, -M_PI_2);
        motionController_.resetPosition(pos, false, true, true);
        motionController_.goStraight(-CHASSIS * 1.5);

        motionController_.goToStraightLine(gui_->getStartPosition(), 1.0, tf::BACKWARD);
        logger_ << "[Robot] Border setup successful!" << std::endl;
    }
    else
        logger_ << "[Robot] Border position mismatch!" << std::endl;
    inBorderDetection_ = false;

}
