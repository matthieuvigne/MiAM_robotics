#include "common/RobotInterface.h"
#include "common/AbstractStrategy.h"
#include "common/RobotGUI.h"
#include <filesystem>
#include "common/ThreadHandler.h"

RobotInterface::RobotInterface(RobotParameters const& robotParameters,
                               RobotGUI *gui,
                               AbstractStrategy *strategy,
                               bool const& testMode,
                               std::string const& teleplotPrefix):
    metronome_(ROBOT_UPDATE_PERIOD),
    motionController_(robotParameters, &logger_),
    servos_(),
    gui_(gui),
    strategy_(strategy),
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
    for (auto& p __attribute__ ((unused)) : std::filesystem::directory_iterator(logDir))
        count++;
    std::string filename = "logs/log" + std::to_string(count) + "_" + std::string(timestamp) + "_" + motionController_.robotParams_.name + ".miam";
    logger_.start(filename);
    logger_.start(filename, teleplotPrefix_);
}

std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();


double getTime()
{

    std::chrono::high_resolution_clock::time_point astarTime = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<double>>(astarTime - startTime).count();
}

void RobotInterface::lowLevelLoop()
{
    pthread_setname_np(pthread_self(), "robot_lll");

    // Initalize / reset everything.
    initLogger();
    motionController_.init(RobotPosition());
    guiState_.state = robotstate::INIT;
    hasMatchStarted_ = false;
    matchStartTime_ = 0.0;
    currentTime_ = -1.0;
    gameState_ = GameState();

    // Setup
    while (guiState_.state == robotstate::INIT)
    {
        setupBeforeMatchStart();
        gui_->update(guiState_);
        usleep(10000);
    }

#ifdef SIMULATION
    metronome_.hasReset_ = false;
#else
    metronome_ = Metronome(ROBOT_UPDATE_PERIOD);
    logger_.setTimeOrigin(metronome_.getStartTime());
#endif
    currentTime_ = 0.0;

    logger_ << "Low-level loop started." << std::endl;

    double lastTime = 0;

    // Create strategy thread.
    std::thread strategyThread;
    pthread_t strategyHandle = 0;

    double debugTimeBeforeWait, debugTimeAfterWait, debugTimeBeforeSensor;
    double debugTimeAfterSensor, debugTimeAfterCompute, debugTimeAfterApply;
    // Loop until start of the match, then for 100 seconds after the start of the match.
    while(!hasMatchStarted_ || (currentTime_ < 100.0 + matchStartTime_))
    {
        // Wait for next tick.
        lastTime = currentTime_;
        debugTimeBeforeWait = getTime();
        metronome_.wait();
        debugTimeAfterWait = getTime();
#ifdef SIMULATION
        if (metronome_.hasReset_)
        {
            metronome_.hasReset_ = false;
            logger_ << "[Robot] Termination signal received from simulation, exiting low-level loop" << std::endl;
            return;
        }
#endif
        currentTime_ = metronome_.getElapsedTime();
        dt_ = currentTime_ - lastTime;

        if (dt_ > 0.50)
        {
            logger_ << "[LAG SPIKE] dt value: " << dt_ << std::endl;
            logger_ << "[LAG SPIKE] debugTimeBeforeWait: " << debugTimeBeforeWait;
            logger_ << "debugTimeAfterWait: " << debugTimeAfterWait;
            logger_ << "debugTimeBeforeSensor: " << debugTimeBeforeSensor << std::endl;
            logger_ << "[LAG SPIKE] debugTimeAfterSensor: " << debugTimeAfterSensor;
            logger_ << "debugTimeAfterCompute: " << debugTimeAfterCompute;
            logger_ << "debugTimeAfterApply: " << debugTimeAfterApply << std::endl;
        }
        // If match hasn't started, perform setup and wait for switch.
        if (!hasMatchStarted_)
        {
            motionController_.isPlayingRightSide_ = gui_->getIsPlayingRightSide();
            motionController_.resetPosition(getStartPosition());
            hasMatchStarted_ = setupBeforeMatchStart();
            if (hasMatchStarted_)
            {
                matchStartTime_ = currentTime_;
                logger_ << "[Robot] Starting match, isPlayingRightSide_: " << motionController_.isPlayingRightSide_ << std::endl;
                guiState_.state = robotstate::MATCH;
                metronome_.resetLag();
                // Start strategy thread.
                strategyThread = std::thread(&AbstractStrategy::match, strategy_);
                strategyHandle = ThreadHandler::addThread(strategyThread);
            }
        }

        // Can run only after init
        if (guiState_.state != robotstate::INIT)
        {
            debugTimeBeforeSensor = getTime();
            updateSensorData();
            debugTimeAfterSensor = getTime();

            // If playing right side: invert right/left encoders)
            if (motionController_.isPlayingRightSide_)
            {
                double const encoderRatio = getParameters().rightEncoderWheelRadius / getParameters().leftEncoderWheelRadius;

                double temp = measurements_.drivetrainMeasurements.encoderSpeed.right;
                measurements_.drivetrainMeasurements.encoderSpeed.right = measurements_.drivetrainMeasurements.encoderSpeed.left / encoderRatio;
                measurements_.drivetrainMeasurements.encoderSpeed.left = temp * encoderRatio;
            }

            // Compute motion target.
            DrivetrainTarget target = motionController_.computeDrivetrainMotion(measurements_.drivetrainMeasurements, dt_, hasMatchStarted_);
            debugTimeAfterCompute = getTime();

            // Apply target to the robot
            applyMotorTarget(target);
            debugTimeAfterApply = getTime();
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


bool RobotInterface::getTestMode() const
{
    return testMode_;
}


void RobotInterface::updateScore(int const& scoreIncrement, std::string const& pointsOrigin)
{
    mutex_.lock();
    guiState_.score += scoreIncrement;
    mutex_.unlock();
    gui_->update(guiState_);
    logger_ << "[Score update] +" << scoreIncrement << ", reason: " << pointsOrigin << std::endl;
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
