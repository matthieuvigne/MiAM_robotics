/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3
#include <unistd.h>
#include <thread>
#include <iomanip>

#include "Robot.h"

// Update loop frequency
const double LOOP_PERIOD = 0.005;

const int START_SWITCH = 21;
const int RAIL_SWITCH = 13;
const int RIGHT_RANGE_ACTIVATE = 4;


Robot::Robot(bool const& testMode, bool const& disableLidar):
    // handler_(&maestro_),
    // RobotInterface(&handler_),
    RobotInterface(),
    lidar_(-M_PI_4),
    testMode_(testMode),
    disableLidar_(disableLidar),
    score_(0),
    startupStatus_(startupstatus::INIT),
    initMotorBlocked_(false),
    // initStatueHigh_(true),
    // timeSinceLastCheckOnRailHeightDuringInit_(-1.0),
    motorSpi_(RPI_SPI_00)
{
    // PIDRail_ = miam::PID(controller::railKp, controller::railKd, controller::railKi, 0.1);

    motionController_.init(RobotPosition());

    // // Set initial rail target
    // targetRailPosition_ = -1;
}


bool Robot::initSystem()
{
    RPi_setupGPIO(START_SWITCH, PI_GPIO_INPUT_PULLUP); // Starting cable.
    // RPi_setupGPIO(RAIL_SWITCH, PI_GPIO_INPUT_PULLUP); // Rail limit cable.
    // RPi_setupGPIO(RIGHT_RANGE_ACTIVATE, PI_GPIO_OUTPUT); // Activation of right range sensor - for cusom I2C addressing.
    // RPi_writeGPIO(RIGHT_RANGE_ACTIVATE, true);
    bool allInitSuccessful = true;

    // if (!isScreenInit_)
    // {
    //     isScreenInit_ = screen_.init("/dev/LCDScreen");
    //     if(!isScreenInit_)
    //     {
    //         #ifdef DEBUG
    //             std::cout << "[Robot] Failed to init communication with LCD screen." << std::endl;
    //         #endif
    //         allInitSuccessful = false;
    //     }
    //     else
    //     {
    //         screen_.setText("Initializing", 0);
    //         screen_.setLCDBacklight(255, 255, 255);
    //     }

    // }

    if (!isArduinoInit_)
    {
        isArduinoInit_ = uCListener_start("/dev/arduinoUno");
        if(!isArduinoInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with Arduino." << std::endl;
            #endif
            allInitSuccessful = false;
            screen_.setText("Failed to init", 0);
            screen_.setText("Arduino", 1);
            screen_.setLCDBacklight(255, 0, 0);
        }
    }

    if (!isStepperInit_)
    {

        // init MCP
        motorMcp_ = miam::MCP2515(&motorSpi_);
        if (!motorMcp_.init())
        {
            std::cout << "Failed to talk to MCP" << std::endl;
            exit(0);
        }

        // init motors
        brushlessMotors_ = miam::RMDX(&motorMcp_, 0.01);


        motorMcp_.init();
        brushlessMotors_.enable(motorRightId);

        usleep(10000);

        // FIXME: mcp needs to be re-init between every message.
        motorMcp_.init();
        brushlessMotors_.enable(motorLeftId);

        usleep(5000000);
    }

    // if (!isServosInit_)
    // {
    //     isServosInit_ = servos_->init("/dev/maestro");
    //     if (!isServosInit_)
    //     {
    //         #ifdef DEBUG
    //             std::cout << "[Robot] Failed to init communication with servo driver." << std::endl;
    //         #endif
    //         allInitSuccessful = false;
    //         screen_.setText("Failed to init", 0);
    //         screen_.setText("servos", 1);
    //         screen_.setLCDBacklight(255, 0, 0);
    //     }
    // }

    // // Init range sensor - starting with left sensor
    // if (!isRangeSensorInit_[LEFT])
    // {
    //     RPi_writeGPIO(RIGHT_RANGE_ACTIVATE, false);
    //     // Try with defalut address
    //     isRangeSensorInit_[LEFT] = rangeSensors_[LEFT].init(&RPI_I2C);
    //     if (!isRangeSensorInit_[LEFT])
    //     {
    //         // Try with new address - in case it was already changed.
    //         isRangeSensorInit_[LEFT] = rangeSensors_[LEFT].init(&RPI_I2C, 0x42);
    //     }
    //     if (!isRangeSensorInit_[LEFT])
    //     {
    //         #ifdef DEBUG
    //             std::cout << "[Robot] Failed to init communication with left range sensor." << std::endl;
    //         #endif
    //         allInitSuccessful = false;
    //         screen_.setText("Failed to init", 0);
    //         screen_.setText("left range sensor", 1);
    //         screen_.setLCDBacklight(255, 0, 0);
    //     }
    //     else
    //     {
    //         rangeSensors_[LEFT].setAddress(0x42);
    //         RPi_writeGPIO(RIGHT_RANGE_ACTIVATE, true);
    //         usleep(10000);
    //     }
    // }
    // if (isRangeSensorInit_[LEFT] && !isRangeSensorInit_[RIGHT])
    // {
    //     isRangeSensorInit_[RIGHT] = rangeSensors_[RIGHT].init(&RPI_I2C);
    //     if (!isRangeSensorInit_[RIGHT])
    //     {
    //         #ifdef DEBUG
    //             std::cout << "[Robot] Failed to init communication with right range sensor." << std::endl;
    //         #endif
    //         allInitSuccessful = false;
    //         screen_.setText("Failed to init", 0);
    //         screen_.setText("right range sensor", 1);
    //         screen_.setLCDBacklight(255, 0, 0);
    //     }
    // }

    if (testMode_ && disableLidar_)
        isLidarInit_ = true;
    if (!isLidarInit_)
    {
        isLidarInit_ = lidar_.init("/dev/RPLIDAR");
        if (!isLidarInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with lidar driver." << std::endl;
            #endif
            // screen_.setText("Failed to init", 0);
            // screen_.setText("lidar", 1);
            // screen_.setLCDBacklight(255, 0, 0);
            allInitSuccessful = false;
        }
    }


    return allInitSuccessful;
}


bool Robot::setupBeforeMatchStart()
{
    // Once the match has started, nothing remains to be done.
    if (hasMatchStarted_)
        return true;
    // Action depends on current startup status
    if (startupStatus_ == startupstatus::INIT)
    {
        // Try to initialize system.
        bool isInit = initSystem();
        if (isInit)
        {
            // screen_.setText("Calibrating rail", 0);
            // screen_.setText("", 1);
            // screen_.setLCDBacklight(255, 255, 255);
            // calibrateRail();

            strategy_.setup(this);

            // Set status.
            // stepperMotors_.getError();
            microcontrollerData_ = uCListener_getData();
            if (testMode_)
            {
                usleep(1000000);
                // isPlayingRightSide_ = true;
                isPlayingRightSide_ = false;
                return true;
            }
            // moveRail(0.5);
            // initStatueHigh_ = false;
            startupStatus_ = startupstatus::WAITING_FOR_CABLE;
            // screen_.setText("Waiting for", 0);
            // screen_.setText("start switch", 1);
            // screen_.setLCDBacklight(255, 255, 255);
        }
    }
    else if (startupStatus_ == startupstatus::WAITING_FOR_CABLE)
    {
        // Wait for cable to be plugged in.
        if (RPi_readGPIO(START_SWITCH) == 0)
        {
            // Store plug time in matchStartTime_ to prevent false start due to switch bounce.
            matchStartTime_ = currentTime_;
            isPlayingRightSide_ = false;
            startupStatus_ = startupstatus::WAITING_FOR_START;
        }
    }
    else if (startupStatus_ == startupstatus::WAITING_FOR_START)
    {
        // // Switch side based on button press.
        // if (screen_.wasButtonPressedSinceLastCall(lcd::button::LEFT))
        // {
        //     isPlayingRightSide_ = !isPlayingRightSide_;
        // }
        // if (screen_.wasButtonPressedSinceLastCall(lcd::button::MIDDLE))
        //     initMotorBlocked_ = !initMotorBlocked_;
        // if (screen_.wasButtonPressedSinceLastCall(lcd::button::RIGHT))
        // {
        //     initStatueHigh_ = !initStatueHigh_;
        //     if (initStatueHigh_)
        //         moveRail(0.75);
        //     else
        //         moveRail(0.5);
        // }

        // if (timeSinceLastCheckOnRailHeightDuringInit_ < 0)
        //     timeSinceLastCheckOnRailHeightDuringInit_ = currentTime_;
        // if (currentTime_ - timeSinceLastCheckOnRailHeightDuringInit_ > 5.0)
        // {
        //     std::cout << "Resetting servo position since last time" << std::endl;
        //     if (initStatueHigh_)
        //         moveRail(0.75);
        //     else
        //         moveRail(0.5);

        //     timeSinceLastCheckOnRailHeightDuringInit_ = currentTime_;
        // }

        // screen_.setText("SIDE  MOT STATUE", 0);
        // std::string secondLine = "";
        // if (isPlayingRightSide_)
        // {
        //     secondLine = "PURPLE";
        //     screen_.setLCDBacklight(255, 0, 255);
        // }
        // else
        // {
        //     secondLine = "YELLOW";
        //     screen_.setLCDBacklight(255, 255, 0);
        // }
        // if (initMotorBlocked_)
        // {
        //     stepperMotors_.hardStop();
        //     secondLine += " ON ";
        // }
        // else
        // {
        //     stepperMotors_.highZ();
        //     secondLine += " OFF ";
        // }
        // if (initStatueHigh_)
        //     secondLine += "   UP";
        // else
        //     secondLine += " DOWN";
        // screen_.setText(secondLine, 1);

        // // If start button is pressed, return true to end startup.
        // if (currentTime_ - matchStartTime_ > 1.5 && (RPi_readGPIO(START_SWITCH) == 1))
        // {
        //     screen_.setText("", 0);
        //     screen_.setText("", 1);
        //     screen_.setLCDBacklight(255, 255, 255);
        //     return true;
        // }
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
    bool heartbeatLed = true;
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
            heartbeatLed = !heartbeatLed;
            // if (heartbeatLed)
            //     screen_.turnOnLED(lcd::RIGHT_LED);
            // else
            //     screen_.turnOffLED(lcd::RIGHT_LED);
            #ifdef DEBUG
            if (heartbeatLed)
            {
                std::cout << "Metronome tick ON" << std::endl;
            }
            else
            {
                std::cout << "Metronome tick OFF" << std::endl;
            }
            #endif
        }

        // If match hasn't started, look at switch value to see if it has.
        if (!hasMatchStarted_)
        {
            hasMatchStarted_ = setupBeforeMatchStart();
            if (hasMatchStarted_)
            {
                matchStartTime_ = currentTime_;
                metronome.resetLag();
                // Start strategy thread.
                strategyThread = std::thread(&Strategy::match, strategy_);
                strategyThread.detach();
                // // Start range aquisition
                // std::thread measureThread = std::thread(&Robot::updateRangeMeasurement, this);
                // measureThread.detach();
            }

        }

        // Update arduino data.
        uCData oldData = microcontrollerData_;
        microcontrollerData_ = uCListener_getData();

        // Compute drivetrain measurements.

        DrivetrainMeasurements measurements;
        measurements.encoderPosition[0] = microcontrollerData_.encoderValues[0];
        measurements.encoderPosition[1] = microcontrollerData_.encoderValues[1];
        measurements.encoderSpeed.right = microcontrollerData_.encoderValues[RIGHT] - oldData.encoderValues[RIGHT];
        measurements.encoderSpeed.left = microcontrollerData_.encoderValues[LEFT] - oldData.encoderValues[LEFT];

        // If playing right side: invert right/left encoders.
        if (isPlayingRightSide_)
        {
            double temp = measurements.encoderSpeed.right;
            measurements.encoderSpeed.right = measurements.encoderSpeed.left;
            measurements.encoderSpeed.left = temp;
        }

        // FIXME
        // std::vector<double> const motorSpeed = brushlessMotors_.getSpeed();
        measurements.motorSpeed(0) = 0;
        measurements.motorSpeed(1) = 0;

        // Update the lidar
        if (!testMode_ || !disableLidar_)
        {
            lidar_.update();
            measurements.lidarDetection = lidar_.detectedRobots_;
        }

        // // Update leds.
        // // FIXME
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
        DrivetrainTarget target = motionController_.computeDrivetrainMotion(measurements, dt, hasMatchStarted_, isPlayingRightSide_);


        // Apply target.
        // std::vector<double> speed;
        // speed.push_back(target.motorSpeed[0] / robotdimensions::stepSize);
        // speed.push_back(target.motorSpeed[1] / robotdimensions::stepSize);
        // stepperMotors_.setSpeed(speed);

        // FIXME
        motorMcp_.init();
        brushlessMotors_.setSpeed(motorRightId, -target.motorSpeed[RIGHT]);
        motorMcp_.init();
        brushlessMotors_.setSpeed(motorLeftId, target.motorSpeed[LEFT]);
    }
    // End of the match.
    std::cout << "Match end" << std::endl;
    pthread_cancel(strategyThread.native_handle());
    stopMotors();

    if (!testMode_ || !disableLidar_)
        lidar_.stop();
}


// void Robot::moveRail(double const& position)
// {
//     // Compute target potentiometer value.
//     int targetValue = railHigh_ + robotdimensions::MIAM_POTENTIOMETER_RANGE * (position - 1);

//     // Compute error
//     int error = uCListener_getData().potentiometerPosition - targetValue;
//     int nIter = 0;
//     while (std::abs(error) > 8 && nIter < 120)
//     {
//         int targetVelocity = -PIDRail_.computeValue(error, 0.020);
//         targetVelocity = std::max(
//             std::min(
//                 robotdimensions::MIAM_RAIL_SERVO_ZERO_VELOCITY + targetVelocity,
//                 robotdimensions::MIAM_RAIL_SERVO_MAX_UP_VELOCITY
//                 ),
//             robotdimensions::MIAM_RAIL_SERVO_MAX_DOWN_VELOCITY
//         );
//         // Send target to servo
//         servos_->moveRail(targetVelocity);

//         usleep(20000);
//         error = uCListener_getData().potentiometerPosition - targetValue;
//         nIter++;
//     }
//     servos_->moveRail(robotdimensions::MIAM_RAIL_SERVO_ZERO_VELOCITY);

// }

// void Robot::calibrateRail()
// {
//     servos_->moveRail(robotdimensions::MIAM_RAIL_SERVO_ZERO_VELOCITY - 250);
//     while (RPi_readGPIO(RAIL_SWITCH) == 1)
//     {
//         usleep(20000);
//     }
//     servos_->moveRail(robotdimensions::MIAM_RAIL_SERVO_ZERO_VELOCITY);
//     railHigh_ = uCListener_getData().potentiometerPosition;
// }

double Robot::getMatchTime()
{
    if (!hasMatchStarted_)
        return 0;
    return currentTime_ - matchStartTime_;
}

void Robot::updateScore(int const& scoreIncrement)
{
    mutex_.lock();
    score_ += scoreIncrement;
    mutex_.unlock();
    std::string text = "Score: " + std::to_string(score_);
    screen_.setText(text, 0);
}

void Robot::stopMotors()
{
    // stepperMotors_.hardStop();
    // usleep(50000);
    // stepperMotors_.highZ();
    motorMcp_.init();
    brushlessMotors_.setSpeed(motorRightId, 0);
    motorMcp_.init();
    brushlessMotors_.setSpeed(motorLeftId, 0);
}


// ExcavationSquareColor Robot::getExcavationReadings(bool readRightSide)
// {
//     if (readRightSide)
//         return microcontrollerData_.rightArmColor;
//     return microcontrollerData_.leftArmColor;
// }


// void Robot::updateRangeMeasurement()
// {
//     // Offset from measurement to position of center of robot.
//     // To update this: place the robot a fixed distance (e.g. 10cm) from
//     // a flat surface, and look at the measurement value.
//     // Don't forget to add robot width.
//     // This offset thus integrates sensor position, sensor offset...
//     int const OFFSET[2] = {78, 75};

//     // Perform average of last N values.
//     #define N_AVG 3
//     int oldValues[2][N_AVG];
//     for (int i = 0; i < 2; i++)
//         for (int j = 0; j < N_AVG; j++)
//             oldValues[i][j] = 0;
//     while (true)
//     {
//         for (int i = 0; i < 2; i++)
//         {
//             for (int j = 1; j < N_AVG; j++)
//                 oldValues[i][j - 1] = oldValues[i][j];
//             oldValues[i][N_AVG - 1] = rangeSensors_[i].getMeasurement() + OFFSET[i];
//             int average = 0;
//             for (int j = 0; j < N_AVG; j++)
//                 average += oldValues[i][j];
//             rangeMeasurements_[i] = static_cast<double>(average) / N_AVG;
//         }
//     }
// }

void Robot::shutdown()
{
    servos_->shutdownServos();
    servos_->activatePump(false);
    stopMotors();
    lidar_.stop();
}