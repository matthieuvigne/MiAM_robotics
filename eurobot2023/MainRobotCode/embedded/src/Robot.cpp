/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3
#include <unistd.h>
#include <thread>
#include <iomanip>

#include "Robot.h"
#include "common/RobotGUI.h"
#include <miam_utils/TextLogger.h>

// Update loop frequency
const double LOOP_PERIOD = 0.010;
const int START_SWITCH = 20;
double const UNDERVOLTAGE_LEVEL = 19.5;


// Motor control parameters
double const Kp = 0.7;
double const Ki = 0.9;
double const maxOutput = 2.5;
double const filterCutoff = 10.0;
double const maxFeedforward = 0.4;


Robot::Robot(RobotParameters const& parameters, AbstractStrategy *strategy, RobotGUI *gui, bool const& testMode, bool const& disableLidar):
    RobotInterface(parameters),
    gui_(gui),
    lidar_(parameters.lidarOffset),
    testMode_(testMode),
    disableLidar_(disableLidar),
    spiMotor_(RPI_SPI_00, 1000000),
    mcp_(&spiMotor_),
    motors_(&mcp_),
    rightController_(&motors_, parameters.rightMotorId, Kp, Ki, maxOutput, filterCutoff, maxFeedforward, parameters.maxWheelAcceleration / parameters.wheelRadius),
    leftController_(&motors_, parameters.leftMotorId, Kp, Ki, maxOutput, filterCutoff, maxFeedforward, parameters.maxWheelAcceleration / parameters.wheelRadius),
    spiEncoder_(RPI_SPI_01, 1000000),
    encoders_(&spiEncoder_, 2),
    strategy_(strategy)
{
    motionController_.init(RobotPosition());

    guiState_.state = robotstate::INIT;
    guiState_.score = 0;

    lastEncoderPosition_.push_back(0);
    lastEncoderPosition_.push_back(0);
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
            if (!motors_.init(motionController_.robotParams_.rightMotorId))
            {
                guiState_.debugStatus += "Right motor init failed\n";
                isMotorsInit_ = false;
            }
            if (!motors_.init(motionController_.robotParams_.leftMotorId))
            {
                guiState_.debugStatus += "Left motor init failed\n";
                isMotorsInit_ = false;
            }
        }
    }

    // Encoder init
    if (!isEncodersInit_)
    {
        isEncodersInit_ = encoders_.init();
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


bool Robot::setupBeforeMatchStart()
{
    // Once the match has started, nothing remains to be done.
    if (hasMatchStarted_)
        return true;
    // Action depends on current startup status
    if (guiState_.state == robotstate::INIT)
    {
        // Try to initialize system.
        bool isInit = initSystem();

        if (isInit)
            guiState_.debugStatus = "";
        if (isInit && guiState_.state != robotstate::UNDERVOLTAGE)
        {
            // Check voltage
            double const batteryVoltageRight = motors_.getStatus(motionController_.robotParams_.rightMotorId).batteryVoltage;
            double const batteryVoltageLeft = motors_.getStatus(motionController_.robotParams_.leftMotorId).batteryVoltage;
            // Wait for valid signal
            if (batteryVoltageRight < 5 || batteryVoltageLeft < 6 || std::abs(batteryVoltageLeft - batteryVoltageRight) > 2.0)
            {
                guiState_.debugStatus = "Waiting for battery voltage from motors";
                return false;
            }
            if ((batteryVoltageRight + batteryVoltageLeft) / 2.0 < UNDERVOLTAGE_LEVEL)
            {
                guiState_.state = robotstate::UNDERVOLTAGE;
                return false;
            }
            gui_->update(guiState_);
            guiState_.state = robotstate::STRATEGY_SETUP;
        }
    }
    if (guiState_.state == STRATEGY_SETUP)
    {
        bool isSetup = strategy_->setup(this);
        if (isSetup)
        {
            if (testMode_)
            {
                matchStartTime_ = -1;
                isPlayingRightSide_ = true;
            }
            guiState_.state = robotstate::WAITING_FOR_CABLE;
        }
    }
    else if (guiState_.state == robotstate::WAITING_FOR_CABLE)
    {
        if (matchStartTime_ < 0)
            matchStartTime_ = currentTime_;
        if (testMode_)
        {
            // Wait for motor to boot
            if (currentTime_ - matchStartTime_ > 1.5)
                return true;
        }
        else
        {
            // Wait for cable to be plugged in.
            if (RPi_readGPIO(START_SWITCH) == 0)
            {
                // Store plug time in matchStartTime_ to prevent false start due to switch bounce.
                matchStartTime_ = currentTime_;
                isPlayingRightSide_ = false;
                guiState_.state = robotstate::WAITING_FOR_START;
            }
        }
    }
    else if (guiState_.state == robotstate::WAITING_FOR_START)
    {
        // If start button is pressed, return true to end startup.
        if (currentTime_ - matchStartTime_ > 1.5 && (RPi_readGPIO(START_SWITCH) == 1))
            return true;
    }
    return false;
}

void Robot::lowLevelLoop()
{
    textlogger::setStartTime();
    textlog << "Low-level loop started." << std::endl;

    // Create metronome
    Metronome metronome(LOOP_PERIOD * 1e9);
    currentTime_ = 0;
    double lastTime = 0;

    std::thread strategyThread;
    pthread_t strategyHandle = 0;
    DrivetrainMeasurements measurements;

    // Loop until start of the match, then for 100 seconds after the start of the match.
    while(!hasMatchStarted_ || (currentTime_ < 100.0 + matchStartTime_))
    {
        // Wait for next tick.
        lastTime = currentTime_;
        metronome.wait();
        currentTime_ = metronome.getElapsedTime();
        double dt = currentTime_ - lastTime;

        // Once init has passed, call strategy update function
        if (guiState_.state != robotstate::INIT && guiState_.state != robotstate::UNDERVOLTAGE)
            strategy_->periodicAction();

        // If match hasn't started, perform setup and wait for switch.
        if (!hasMatchStarted_)
        {
            hasMatchStarted_ = setupBeforeMatchStart();
            if (hasMatchStarted_)
            {
                matchStartTime_ = currentTime_;
                isPlayingRightSide_ = gui_->getIsPlayingRightSide();
                guiState_.state = robotstate::MATCH;
                metronome.resetLag();
                // Start strategy thread.
                strategyThread = std::thread(&AbstractStrategy::match, strategy_);
                strategyHandle = strategyThread.native_handle();
                strategyThread.detach();
            }
        }

        // Update measurements data.
        std::vector<double> encoderPosition = encoders_.updatePosition();
        int const pR = motionController_.robotParams_.rightEncoderId;
        int const pL = motionController_.robotParams_.leftEncoderId;
        encoderPosition[pR] *= motionController_.robotParams_.rightEncoderDirection;
        encoderPosition[pL] *= motionController_.robotParams_.leftEncoderDirection;
        measurements.encoderPosition[side::RIGHT] = encoderPosition[pR];
        measurements.encoderPosition[side::LEFT] = encoderPosition[pL];
        measurements.encoderSpeed.right = encoderPosition[pR] - lastEncoderPosition_[pR];
        measurements.encoderSpeed.left = encoderPosition[pL] - lastEncoderPosition_[pL];
        lastEncoderPosition_ = encoderPosition;

        // If playing side::RIGHT side: invert side::RIGHT/side::LEFT encoders.
        if (isPlayingRightSide_)
        {
            double temp = measurements.encoderSpeed.right;
            measurements.encoderSpeed.right = measurements.encoderSpeed.left;
            measurements.encoderSpeed.left = temp;
        }

        // Update the lidar
        if (!testMode_ || !disableLidar_)
        {
            lidar_.update();
            measurements.lidarDetection = lidar_.detectedRobots_;
        }

        // Update leds.
        // FIXME
        // int coeff_ = 0;
        // if (coeff_ == 0)
        //     screen_.turnOnLED(lcd::LEFT_LED);
        // else
        //     screen_.turnOffLED(lcd::LEFT_LED);
        // if (coeff_ < 1.0)
        //     screen_.turnOnLED(lcd::MIDDLE_LED);
        // else
        //     screen_.turnOffLED(lcd::MIDDLE_LED);

        // Compute motion target.
        DrivetrainTarget target = motionController_.computeDrivetrainMotion(measurements, dt, hasMatchStarted_);

        // Apply target
        static bool wasRunning = false;
        if (!hasMatchStarted_)
        {
            leftController_.stop();
            rightController_.stop();
        }
        else
        {
            if (std::abs(target.motorSpeed[0]) < 0.001 && std::abs(target.motorSpeed[1]) < 0.001 )
            {
                if (wasRunning)
                    textlog << "[Robot] Motors stopping" << std::endl;
                wasRunning = false;
                rightController_.stop();
                measurements.motorSpeed(0) = 0.0;
                leftController_.stop();
                measurements.motorSpeed(1) = 0.0;
            }
            else
            {
                if (!wasRunning)
                    textlog << "[Robot] Motors running" << std::endl;
                wasRunning = true;
                int sign = motionController_.robotParams_.rightMotorDirection;
                measurements.motorSpeed(0) = sign * rightController_.sendTarget(sign * target.motorSpeed[0], dt);
                sign = motionController_.robotParams_.leftMotorDirection;
                measurements.motorSpeed(1) = sign * leftController_.sendTarget(sign * target.motorSpeed[1], dt);
            }
        }
        motionController_.log("MotorController.right.current", rightController_.current_);
        motionController_.log("MotorController.right.targetCurrent", rightController_.targetCurrent_);
        motionController_.log("MotorController.right.position", rightController_.position_);
        motionController_.log("MotorController.right.velocity", rightController_.velocity_);
        motionController_.log("MotorController.right.targetVelocity", rightController_.clampedTargetVelocity_);

        motionController_.log("MotorController.left.current", leftController_.current_);
        motionController_.log("MotorController.left.targetCurrent", leftController_.targetCurrent_);
        motionController_.log("MotorController.left.position", leftController_.position_);
        motionController_.log("MotorController.left.velocity", leftController_.velocity_);
        motionController_.log("MotorController.left.targetVelocity", leftController_.clampedTargetVelocity_);

        // Update gui
        guiState_.currentMatchTime = currentTime_ - matchStartTime_;
        guiState_.currentPosition = motionController_.getCurrentPosition();
        guiState_.detectedObstacles = motionController_.filteredDetectedObstacles_;
        gui_->update(guiState_);
    }
    // End of the match.
    std::cout << "Match end" << std::endl;
    guiState_.state = robotstate::MATCH_DONE;
    gui_->update(guiState_);
    for (auto handle: strategy_->createdThreads_)
        pthread_cancel(handle);
    pthread_cancel(strategyHandle);
    stopMotors();

    if (!testMode_ || !disableLidar_)
        lidar_.stop();

    // Wait for referees to count the score.
    while (true) ;;
}

double Robot::getMatchTime()
{
    if (!hasMatchStarted_)
        return 0;
    return currentTime_ - matchStartTime_;
}

void Robot::updateScore(int const& scoreIncrement)
{
    mutex_.lock();
    guiState_.score += scoreIncrement;
    mutex_.unlock();
    gui_->update(guiState_);
}

void Robot::stopMotors()
{
    motors_.setCurrent(motionController_.robotParams_.rightMotorId, 0);
    motors_.setCurrent(motionController_.robotParams_.leftMotorId, 0);
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
