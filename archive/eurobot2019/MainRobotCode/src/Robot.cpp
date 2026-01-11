/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3
#include <unistd.h>
#include <thread>
#include <iomanip>

#include "Robot.h"


// Update loop frequency
const double LOOP_PERIOD = 0.010;

const int START_SWITCH = 21;

// Potentiometer
const int MIAM_POTENTIOMETER_LOW_VALUE = 130;
const int MIAM_POTENTIOMETER_HIGH_VALUE = 380;
const int MIAM_RAIL_TOLERANCE = 10;

const int MIAM_RAIL_SERVO_ZERO_VELOCITY = 1450;
const int MIAM_RAIL_SERVO_MAX_UP_VELOCITY = 2000;
const int MIAM_RAIL_SERVO_MAX_DOWN_VELOCITY = 1000;


Robot::Robot():
    AbstractRobot(),
    isScreenInit_(false),
    isServosInit_(false),
    isArduinoInit_(false),
    isLidarInit_(false),
    startupStatus_(startupstatus::INIT),
    initMotorState_(1),
    score_(5),  // Initial score: 5, for the experiment.
    hasExperimentStarted_(false),
    experiment_(),
    lidar_(M_PI_4),
    curvilinearAbscissa_(0.0),
    ignoreDetection_(false),
    askedForReset_(false),
    avoidanceTimeout_(250)
{
    kinematics_ = DrivetrainKinematics(robotdimensions::wheelRadius,
                                      robotdimensions::wheelSpacing,
                                      robotdimensions::encoderWheelRadius,
                                      robotdimensions::encoderWheelSpacing);
    // Update trajectory config.
    miam::trajectory::setTrajectoryGenerationConfig(robotdimensions::maxWheelSpeedTrajectory,
                                                    robotdimensions::maxWheelAccelerationTrajectory,
                                                    robotdimensions::wheelSpacing);

    // Create logger.
    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%dT%H%M%SZ", std::localtime(&t));
    std::string filename = "logs/log" + std::string(timestamp) + ".csv";
    std::string headers = getHeaderStringList();
    // Log robot dimensions in header.
    std::string info = "wheelRadius:" + std::to_string(robotdimensions::wheelRadius) + \
                        "_wheelSpacing:" + std::to_string(robotdimensions::wheelSpacing) + \
                        "_stepSize:" + std::to_string(robotdimensions::stepSize);

    logger_ = Logger(filename, "Match code", info, getHeaderStringList());

    // Set initial positon.
    RobotPosition initialPosition;
    initialPosition.x = 150 + 75;
    initialPosition.y = 1100 + 150 + 30;
    initialPosition.theta = -M_PI_2;
    currentPosition_.set(initialPosition);
    currentBaseSpeed_.linear = 0;
    currentBaseSpeed_.angular = 0;

    // Set PIDs.
    PIDLinear_ = miam::PID(controller::linearKp, controller::linearKd, controller::linearKi, 0.2);
    PIDAngular_ = miam::PID(controller::rotationKp, controller::rotationKd, controller::rotationKi, 0.15);
    PIDRail_ = miam::PID(controller::railKp, controller::railKd, controller::railKi, 0.1);

    // Set initial rail target
    targetRailPosition_ = -1;
}


bool Robot::initSystem()
{
    RPi_setupGPIO(START_SWITCH, PI_GPIO_INPUT_PULLUP); // Starting cable.
    bool allInitSuccessful = true;

    if (!isScreenInit_)
    {
        isScreenInit_ = screen_.init("/dev/LCDScreen");
        if(!isScreenInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with LCD screen." << std::endl;
            #endif
            allInitSuccessful = false;
        }
        else
        {
            screen_.setText("Initializing", 0);
            robot.screen_.setLCDBacklight(255, 255, 255);
        }

    }

    if (!isArduinoInit_)
    {
        isArduinoInit_ = uCListener_start("/dev/arduinoUno");
        if(!isArduinoInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with Arduino." << std::endl;
            #endif
            allInitSuccessful = false;
            robot.screen_.setText("Failed to init", 0);
            robot.screen_.setText("Arduino", 1);
            robot.screen_.setLCDBacklight(255, 0, 0);
        }
    }

    if (!isStepperInit_)
    {
        // Motor config values.
        const int MOTOR_KVAL_HOLD = 0x30;
        const int MOTOR_BEMF[4] = {0x3B, 0x1430, 0x22, 0x53};

        // Compute max stepper motor speed.
        int maxSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
        int maxAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;
        // Initialize both motors.
        stepperMotors_ = miam::L6470(RPI_SPI_00, 2);
        isStepperInit_ = stepperMotors_.init(maxSpeed, maxAcceleration, MOTOR_KVAL_HOLD,
                                             MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);
        if (!isStepperInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with stepper motors." << std::endl;
            #endif
            allInitSuccessful = false;
            robot.screen_.setText("Failed to init", 0);
            robot.screen_.setText("stepper motors", 1);
            robot.screen_.setLCDBacklight(255, 0, 0);
        }
    }

    if (!isServosInit_)
    {
        isServosInit_ = servos_.init("/dev/ttyACM2");
        if (!isServosInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with servo driver." << std::endl;
            #endif
            allInitSuccessful = false;
            robot.screen_.setText("Failed to init", 0);
            robot.screen_.setText("servos", 1);
            robot.screen_.setLCDBacklight(255, 0, 0);
        }
    }

    if (!isLidarInit_)
    {
        isLidarInit_ = lidar_.init("/dev/RPLIDAR");
        if (!isLidarInit_)
        {
            #ifdef DEBUG
                std::cout << "[Robot] Failed to init communication with lidar driver." << std::endl;
            #endif
            // Temporary: make lidar optional, robot can work without it.
            allInitSuccessful = false;
            robot.screen_.setText("Failed to init", 0);
            robot.screen_.setText("lidar", 1);
            robot.screen_.setLCDBacklight(255, 0, 0);

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
            startupStatus_ = startupstatus::WAITING_FOR_CABLE;
            robot.screen_.setText("Waiting for", 0);
            robot.screen_.setText("start switch", 1);
            robot.screen_.setLCDBacklight(255, 255, 255);
            servos_.moveSuction(true);
            for (int i = 0; i < 3; i++)
                servos_.openTube(i);
            servos_.foldArms();
            // Test motors.
            std::cout << "here" << std::endl;
            robot.stepperMotors_.getError();
            std::vector<double> speed;
            motorSpeed_[0] = 150;
            motorSpeed_[1] = 150;
            stepperMotors_.setSpeed(motorSpeed_);
            usleep(300000);
            motorSpeed_[0] = 0;
            motorSpeed_[1] = 0;
            stepperMotors_.setSpeed(motorSpeed_);
        }
    }
    else if (startupStatus_ == startupstatus::WAITING_FOR_CABLE)
    {
        // Wait for cable to be plugged in.
        if (RPi_readGPIO(START_SWITCH) == 0)
        {
            // Store plug time in matchStartTime_ to prevent false start due to switch bounce.
            matchStartTime_ = currentTime_;
            startupStatus_ = startupstatus::PLAYING_LEFT;
            isPlayingRightSide_ = false;
            robot.screen_.setText("     YELLOW    >", 0);
            robot.screen_.setLCDBacklight(255, 200, 0);
        }
    }
    else
    {
        // Switch side based on button press.
        if (startupStatus_ == startupstatus::PLAYING_RIGHT && (robot.screen_.getButtonState() & lcd::LEFT_BUTTON))
        {
            startupStatus_ = startupstatus::PLAYING_LEFT;
            isPlayingRightSide_ = false;
            robot.screen_.setText("     YELLOW    >", 0);
            robot.screen_.setLCDBacklight(255, 255, 0);
        }
        else if (startupStatus_ == startupstatus::PLAYING_LEFT && (robot.screen_.getButtonState() & lcd::RIGHT_BUTTON))
        {
            startupStatus_ = startupstatus::PLAYING_RIGHT;
            isPlayingRightSide_ = true;
            robot.screen_.setText("<    PURPLE     ", 0);
            robot.screen_.setLCDBacklight(255, 0, 255);
        }

        // Use down button to lock / unlock the motors.
        if ((robot.screen_.getButtonState() & lcd::MIDDLE_BUTTON))
        {
            // Only modify state if the button was just pressed.
            if (initMotorState_ < 2)
            {
                if (initMotorState_ == 0)
                    stepperMotors_.hardStop();
                else
                    stepperMotors_.highZ();
                initMotorState_ = (initMotorState_ + 1) % 2 + 2;
            }
        }
        else
        {
            // Button release: change state to be less that two.
            initMotorState_ = initMotorState_ % 2;
        }

        // Line 1 : experiment status
        if (experiment_.isConnected())
            robot.screen_.setText("Xp: OK", 1);
        else
            robot.screen_.setText("Xp: NOT OK", 1);

        // If start button is pressed, return true to end startup.
        if (currentTime_ - matchStartTime_ > 0.5 && (RPi_readGPIO(START_SWITCH) == 1))
        {
            robot.screen_.setText("", 0);
            robot.screen_.setText("", 1);
            robot.screen_.setLCDBacklight(255, 255, 255);
            return true;
        }
    }
    return false;
}


void Robot::performPositionReset(miam::RobotPosition const& resetPosition, bool const& resetX, bool const& resetY, bool const& resetTheta)
{
    reset_ = currentPosition_.get();
    if(resetX)
        reset_.x = resetPosition.x;
    if(resetY)
        reset_.y = resetPosition.y;
    if(resetTheta)
        reset_.theta = resetPosition.theta;
    askedForReset_ = true;
    usleep(20000);
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
    bool heartbeatLed = false;
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
            if (heartbeatLed)
                robot.screen_.turnOnLED(lcd::RIGHT_LED);
            else
                robot.screen_.turnOffLED(lcd::RIGHT_LED);
        }

        if (askedForReset_)
        {
            askedForReset_ = false;
            currentPosition_.set(reset_);
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
                strategyThread = std::thread(&matchStrategy);
                strategyThread.detach();
                // Start experiment.
                experiment_.start();
            }
        }

        // Update arduino data.
        uCData oldData = microcontrollerData_;
        microcontrollerData_ = uCListener_getData();

        // Compute encoder update.
        WheelSpeed encoderIncrement;
        encoderIncrement.right = microcontrollerData_.encoderValues[RIGHT] - oldData.encoderValues[RIGHT];
        encoderIncrement.left = microcontrollerData_.encoderValues[LEFT] - oldData.encoderValues[LEFT];

        // If playing right side: invert right/left encoders, with minus sign because both motors are opposite of each other.
        if (isPlayingRightSide_)
        {
            double temp = encoderIncrement.right;
            encoderIncrement.right = encoderIncrement.left;
            encoderIncrement.left = temp;
        }

        // Update motor position.
        motorPosition_ = stepperMotors_.getPosition();

        // Update the lidar
        nLidarPoints_ = lidar_.update();
        coeff_ = avoidOtherRobots();
        // Update leds.
        if (coeff_ == 0)
            screen_.turnOnLED(lcd::LEFT_LED);
        else
            screen_.turnOffLED(lcd::LEFT_LED);
        if (coeff_ < 1.0)
            screen_.turnOnLED(lcd::MIDDLE_LED);
        else
            screen_.turnOffLED(lcd::MIDDLE_LED);

        // Update position and perform tracking only after match start.
        if (hasMatchStarted_)
        {
            // Integrate encoder measurements.
            RobotPosition currentPosition = currentPosition_.get();
            kinematics_.integratePosition(encoderIncrement, currentPosition);
            currentPosition_.set(currentPosition);

            // Perform trajectory tracking.
            updateTrajectoryFollowingTarget(dt);
        }

        // Get current encoder speeds (in rad/s)
        WheelSpeed instantWheelSpeedEncoder;
        instantWheelSpeedEncoder.right = encoderIncrement.right / dt;
        instantWheelSpeedEncoder.left = encoderIncrement.left / dt;

        // Get base speed
        currentBaseSpeed_ = kinematics_.forwardKinematics(instantWheelSpeedEncoder, true);

        // Update lidar.
        lidar_.update();

        // Check experiment status.
        if (!hasExperimentStarted_)
        {
            hasExperimentStarted_ = experiment_.hasStarted();
            if (hasExperimentStarted_)
                updateScore(35);
        }

        // Update log.
        updateLog();
    }
    // End of the match.
    std::cout << "Match end" << std::endl;
    pthread_cancel(strategyThread.native_handle());
    stopMotors();
    lidar_.stop();
}


void Robot::moveRail(double position)
{
    // Compute target potentiometer value.
    int targetValue = MIAM_POTENTIOMETER_LOW_VALUE - (MIAM_POTENTIOMETER_LOW_VALUE - MIAM_POTENTIOMETER_HIGH_VALUE) * position;

    // Compute error
    int error = microcontrollerData_.potentiometerPosition - targetValue;
    int nIter = 0;
    while (std::abs(error) > 8 && nIter < 120)
    {
        int targetVelocity = -PIDRail_.computeValue(error, 0.020);
        targetVelocity = std::max(
            std::min(
                MIAM_RAIL_SERVO_ZERO_VELOCITY + targetVelocity,
                MIAM_RAIL_SERVO_MAX_UP_VELOCITY
                ),
            MIAM_RAIL_SERVO_MAX_DOWN_VELOCITY
        );
        // Send target to servo
        robot.servos_.moveRail(targetVelocity);

        usleep(20000);
        error = microcontrollerData_.potentiometerPosition - targetValue;
        nIter++;
    }
    robot.servos_.moveRail(MIAM_RAIL_SERVO_ZERO_VELOCITY);
}

void Robot::updateLog()
{
    logger_.setData(LOGGER_TIME, currentTime_);
    logger_.setData(LOGGER_COMMAND_VELOCITY_RIGHT, motorSpeed_[RIGHT]);
    logger_.setData(LOGGER_COMMAND_VELOCITY_LEFT, motorSpeed_[LEFT]);
    logger_.setData(LOGGER_MOTOR_POSITION_RIGHT, motorPosition_[RIGHT]);
    logger_.setData(LOGGER_MOTOR_POSITION_LEFT, motorPosition_[LEFT]);
    logger_.setData(LOGGER_ENCODER_RIGHT, microcontrollerData_.encoderValues[RIGHT]);
    logger_.setData(LOGGER_ENCODER_LEFT, microcontrollerData_.encoderValues[LEFT]);

    RobotPosition currentPosition = currentPosition_.get();
    logger_.setData(LOGGER_CURRENT_POSITION_X, currentPosition.x);
    logger_.setData(LOGGER_CURRENT_POSITION_Y, currentPosition.y);
    logger_.setData(LOGGER_CURRENT_POSITION_THETA, currentPosition.theta);
    logger_.setData(LOGGER_CURRENT_VELOCITY_LINEAR, currentBaseSpeed_.linear);
    logger_.setData(LOGGER_CURRENT_VELOCITY_ANGULAR, currentBaseSpeed_.angular);

    logger_.setData(LOGGER_TARGET_POSITION_X, trajectoryPoint_.position.x);
    logger_.setData(LOGGER_TARGET_POSITION_Y, trajectoryPoint_.position.y);
    logger_.setData(LOGGER_TARGET_POSITION_THETA, trajectoryPoint_.position.theta);
    logger_.setData(LOGGER_TARGET_LINEAR_VELOCITY, trajectoryPoint_.linearVelocity);
    logger_.setData(LOGGER_TARGET_ANGULAR_VELOCITY, trajectoryPoint_.angularVelocity);

    logger_.setData(LOGGER_TRACKING_LONGITUDINAL_ERROR, trackingLongitudinalError_);
    logger_.setData(LOGGER_TRACKING_TRANSVERSE_ERROR, trackingTransverseError_);
    logger_.setData(LOGGER_TRACKING_ANGLE_ERROR, trackingAngleError_);

    logger_.setData(LOGGER_RAIL_POSITION, microcontrollerData_.potentiometerPosition);

    logger_.setData(LOGGER_LINEAR_P_I_D_CORRECTION, PIDLinear_.getCorrection());
    logger_.setData(LOGGER_ANGULAR_P_I_D_CORRECTION, PIDAngular_.getCorrection());
    logger_.setData(LOGGER_RAIL_P_I_D_CORRECTION, PIDRail_.getCorrection());
    logger_.setData(LOGGER_DETECTION_COEFF, this->coeff_);
    logger_.setData(LOGGER_LIDAR_N_POINTS, nLidarPoints_);
    logger_.writeLine();
}

double Robot::getMatchTime()
{
    if (!hasMatchStarted_)
        return 0;
    return currentTime_ - matchStartTime_;
}

void Robot::updateScore(int scoreIncrement)
{
    mutex_.lock();
    score_ += scoreIncrement;
    mutex_.unlock();
    std::string text = "Score: " + std::to_string(score_);
    screen_.setText(text, 0);
}
