/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3
#include <unistd.h>
#include <thread>
#include <iomanip>

#include "Robot.h"
#include "common/RobotGUI.h"

// Update loop frequency
const double LOOP_PERIOD = 0.005;

const int START_SWITCH = 20;
const int RAIL_SWITCH = 13;

double const UNDERVOLTAGE_LEVEL = 19.5;


Robot::Robot(RobotParameters const& parameters, AbstractStrategy *strategy, RobotGUI *gui, bool const& testMode, bool const& disableLidar):
    handler_(&maestro_),
    gui_(gui),
    RobotInterface(parameters, &handler_),
    lidar_(-M_PI_4),
    testMode_(testMode),
    disableLidar_(disableLidar),
    strategy_(strategy),
    spiMotor_(RPI_SPI_00, 1000000),
    mcp_(&spiMotor_),
    motors_(&mcp_),
    spiEncoder_(RPI_SPI_01, 1000000),
    encoders_(&spiEncoder_, 2)
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
    RPi_setupGPIO(RAIL_SWITCH, PI_GPIO_INPUT_PULLUP); // Rail limit cable.
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
    }

    // TODO servo

    return isMCPInit_ & isMotorsInit_ & isEncodersInit_;
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
        if (isInit && guiState_.state != robotstate::UNDERVOLTAGE)
        {
            // Check voltage
            double const batteryVoltage = motors_.getStatus(motionController_.robotParams_.rightMotorId).batteryVoltage;
            // if (batteryVoltage < UNDERVOLTAGE_LEVEL)
            // {
            //     guiState_.state = robotstate::UNDERVOLTAGE;
            //     return false;
            // }

            strategy_->setup(this);

            // Set status.
            lastEncoderPosition_ = encoders_.updatePosition();
            if (testMode_)
            {
                matchStartTime_ = currentTime_;
                isPlayingRightSide_ = true;
            }
            guiState_.state = robotstate::WAITING_FOR_CABLE;
        }
    }
    else if (guiState_.state == robotstate::WAITING_FOR_CABLE)
    {
        if (testMode_)
        {
            // Wait for motor to boot
            if (currentTime_ - matchStartTime_ > 3.5)
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
        // TODO initial config

        // If start button is pressed, return true to end startup.
        if (currentTime_ - matchStartTime_ > 1.5 && (RPi_readGPIO(START_SWITCH) == 1))
            return true;
    }
    return false;
}

void Robot::lowLevelLoop()
{
    std::cout << "Low-level loop started." << std::endl;

    // Create metronome
    Metronome metronome(LOOP_PERIOD * 1e9);
    currentTime_ = 0;
    double lastTime = 0;

    std::thread strategyThread;
    int nIter = 0;
    DrivetrainMeasurements measurements;

    // Loop until start of the match, then for 100 seconds after the start of the match.
    while(!hasMatchStarted_ || (currentTime_ < 100.0 + matchStartTime_))
    {
        // Wait for next tick.
        lastTime = currentTime_;
        metronome.wait();
        currentTime_ = metronome.getElapsedTime();
        double dt = currentTime_ - lastTime;

        // Heartbeat.
        nIter ++;
        if (nIter % 50 == 0)
        {
            if (isMotorsInit_)
            {
                guiState_.batteryVoltage = motors_.getStatus(motionController_.robotParams_.rightMotorId).batteryVoltage;
                if (guiState_.batteryVoltage < UNDERVOLTAGE_LEVEL + 0.5)
                {
                    std::cout << "Warning: low battery: " << guiState_.batteryVoltage << "V. Critical level: " << UNDERVOLTAGE_LEVEL << "V" << std::endl;
                }
            }
        }

        // If match hasn't started, look at switch value to see if it has.
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
        int sign = motionController_.robotParams_.rightMotorDirection;
        measurements.motorSpeed(0) = sign * motors_.setSpeed(motionController_.robotParams_.rightMotorId, sign * target.motorSpeed[0]);
        sign = motionController_.robotParams_.leftMotorDirection;
        measurements.motorSpeed(1) = sign * motors_.setSpeed(motionController_.robotParams_.leftMotorId, sign * target.motorSpeed[1]);

        // Update gui
        gui_->update(guiState_);
    }
    // End of the match.
    std::cout << "Match end" << std::endl;
    guiState_.state = robotstate::MATCH_DONE;
    gui_->update(guiState_);
    pthread_cancel(strategyThread.native_handle());
    stopMotors();

    if (!testMode_ || !disableLidar_)
        lidar_.stop();
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
    // TODO brake.
    motors_.setSpeed(motionController_.robotParams_.rightMotorId, 0);
    motors_.setSpeed(motionController_.robotParams_.leftMotorId, 0);
}


void Robot::shutdown()
{
    servos_->shutdownServos();
    servos_->activatePump(false);
    stopMotors();
    lidar_.stop();
}