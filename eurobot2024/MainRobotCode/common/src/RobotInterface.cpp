#include "common/RobotInterface.h"
#include "common/AbstractStrategy.h"
#include "common/RobotGUI.h"
#include <filesystem>

RobotInterface::RobotInterface(RobotParameters const& robotParameters,
                               RobotGUI *gui,
                               AbstractStrategy *strategy,
                               bool const& testMode,
                               std::string const& teleplotPrefix):
    motionController_(robotParameters, &logger_),
    servos_(),
    gui_(gui),
    strategy_(strategy),
    metronome_(ROBOT_UPDATE_PERIOD),
    testMode_(testMode),
    teleplotPrefix_(teleplotPrefix)
{
}

void RobotInterface::initLogger()
{
    // Create logger.
    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%SZ", std::localtime(&t));

    // Generate unique file ID, because raspberry pi clock is always reset.
    int count = 0;
    std::filesystem::path logDir{"logs/"};
    for (auto& p : std::filesystem::directory_iterator(logDir))
        count++;
    std::string filename = "logs/log" + std::to_string(count) + "_" + std::string(timestamp) + "_" + motionController_.robotParams_.name + ".hdf5";
    logger_.start(filename);
    logger_.start(filename, teleplotPrefix_);
}

void RobotInterface::lowLevelLoop()
{
    // Initalize / reset everything.
    initLogger();
    motionController_.init(RobotPosition());
    guiState_.state = robotstate::INIT;
    hasMatchStarted_ = false;
    matchStartTime_ = 0.0;
    currentTime_ = 0.0;

#ifdef SIMULATION
    metronome_.hasReset_ = false;
#endif

    logger_ << "Low-level loop started." << std::endl;

    double lastTime = 0;

    // Create strategy thread.
    std::thread strategyThread;
    pthread_t strategyHandle = 0;

    // Loop until start of the match, then for 100 seconds after the start of the match.
    while(!hasMatchStarted_ || (currentTime_ < 100.0 + matchStartTime_))
    {
        // Wait for next tick.
        lastTime = currentTime_;
        metronome_.wait();
#ifdef SIMULATION
        if (metronome_.hasReset_)
        {
            metronome_.hasReset_ = false;
            if (strategyHandle > 0)
                pthread_cancel(strategyHandle);
            logger_ << "[Robot] Termination signal received from simulation, exiting low-level loop" << std::endl;
            return;
        }
#endif
        currentTime_ = metronome_.getElapsedTime();
        dt_ = currentTime_ - lastTime;

        // If match hasn't started, perform setup and wait for switch.
        if (!hasMatchStarted_)
        {
            hasMatchStarted_ = setupBeforeMatchStart();
            if (hasMatchStarted_)
            {
                matchStartTime_ = currentTime_;
                motionController_.isPlayingRightSide_ = gui_->getIsPlayingRightSide();
                logger_ << "[Robot] Starting match, isPlayingRightSide_" << motionController_.isPlayingRightSide_ << std::endl;
                guiState_.state = robotstate::MATCH;
                metronome_.resetLag();
                // Start strategy thread.
                strategyThread = std::thread(&AbstractStrategy::match, strategy_);
                strategyHandle = strategyThread.native_handle();
                strategyThread.detach();
            }
        }

        // Can run only after init
        if (guiState_.state != robotstate::INIT)
        {
            updateSensorData();

            // If playing right side: invert right/left encoders)
            if (motionController_.isPlayingRightSide_)
            {
                double temp = measurements_.drivetrainMeasurements.encoderSpeed.right;
                measurements_.drivetrainMeasurements.encoderSpeed.right = measurements_.drivetrainMeasurements.encoderSpeed.left;
                measurements_.drivetrainMeasurements.encoderSpeed.left = temp;
            }

            // Compute motion target.
            DrivetrainTarget target = motionController_.computeDrivetrainMotion(measurements_.drivetrainMeasurements, dt_, hasMatchStarted_);

            // Apply target to the robot
            applyMotorTarget(target);
        }

        // Update gui
        guiState_.currentMatchTime = currentTime_ - matchStartTime_;
        guiState_.currentPosition = motionController_.getCurrentPosition();
        guiState_.detectedObstacles = motionController_.filteredDetectedObstacles_;
        guiState_.batteryVoltage = measurements_.batteryVoltage;
        gui_->update(guiState_);
    }

    // End of the match.
    logger_ << "[Robot] Match has ended" << std::endl;

    guiState_.state = robotstate::MATCH_DONE;
    gui_->update(guiState_);
    for (auto handle: strategy_->createdThreads_)
        pthread_cancel(handle);
    pthread_cancel(strategyHandle);
    stopMotors();

    matchEnd();

    // Wait for referees to count the score.
    while (true) ;;
}


RobotParameters RobotInterface::getParameters() const
{
    return motionController_.robotParams_;
}


bool RobotInterface::isStrategyTop() const
{
    return gui_->getIsTopStrategy();
}


bool RobotInterface::getTestMode() const
{
    return testMode_;
}


void RobotInterface::updateScore(int const& scoreIncrement)
{
    mutex_.lock();
    guiState_.score += scoreIncrement;
    mutex_.unlock();
    gui_->update(guiState_);
}

bool RobotInterface::isPlayingRightSide() const
{
    return motionController_.isPlayingRightSide_;
}

double RobotInterface::getMatchTime()
{
    if (!hasMatchStarted_)
        return 0;
    return currentTime_ - matchStartTime_;
}

bool RobotInterface::setupBeforeMatchStart()
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
            // Check battery voltage
            updateSensorData();
            if (measurements_.batteryVoltage  < UNDERVOLTAGE_LEVEL)
            {
                guiState_.state = robotstate::UNDERVOLTAGE;
                return false;
            }
            guiState_.state = robotstate::STRATEGY_SETUP;
            gui_->update(guiState_);
        }
    }
    if (guiState_.state == STRATEGY_SETUP)
    {
        bool isSetup = strategy_->setup(this);
        if (isSetup)
        {
            if (testMode_)
                matchStartTime_ = -1;
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
            if (isStartingSwitchPluggedIn())
            {
                // Store plug time in matchStartTime_ to prevent false start due to switch bounce.
                matchStartTime_ = currentTime_;
                motionController_.isPlayingRightSide_ = false;
                guiState_.state = robotstate::WAITING_FOR_START;
            }
        }
    }
    else if (guiState_.state == robotstate::WAITING_FOR_START)
    {
        // If start button is pressed, return true to end startup.
        if (currentTime_ - matchStartTime_ > 1.5 && !isStartingSwitchPluggedIn())
            return true;
    }
    return false;
}
