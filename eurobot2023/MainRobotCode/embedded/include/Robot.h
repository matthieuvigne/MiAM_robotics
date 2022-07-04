/// \file Robot.h
/// \brief This file defines global variables representing the hardware on the robot, that needs sharing between
///        different files.
///
/// \details This header is included in all other source files. It defines a structure representing the physical robot
///          in the code. Note that these variables are directly available, unprotected: it is to the user to ensure
///          no race condition may occur, for instance when writing the current robot position.
/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3

#ifndef ROBOT_H
     #define ROBOT_H

    ///< Global includes
    #include <miam_utils/miam_utils.h>
    #include <miam_utils/drivers/USBLCDDriver.h>
    #include <miam_utils/raspberry_pi/RaspberryPi.h>
    #include <miam_utils/trajectory/PointTurn.h>
    #include <miam_utils/trajectory/Utilities.h>
    #include <miam_utils/trajectory/DrivetrainKinematics.h>
    #include "miam_utils/RPLidarHandler.h"
    #include <math.h>
    #include <stdlib.h>
    #include <stdio.h>

    #include <memory>
    #include <vector>
    #include <mutex>

    #include "uCListener.h"
    #include "ServoHandler.h"
    #include "RobotInterface.h"
    #include "Strategy.h"

    // Right and left macros, for array addressing.
    using miam::RobotPosition;
    using miam::ProtectedPosition;
    using miam::trajectory::Trajectory;

    ///< The various steps of the startup process.
    enum startupstatus{
        INIT,
        WAITING_FOR_CABLE,
        WAITING_FOR_START
    };


    // Controller parameters
    namespace controller
    {
        double const railKp = 20.0;
        double const railKd = 0.0;
        double const railKi = 0.0;
    }

    class Robot : public RobotInterface
    {
        public:

            /// \brief Constructor: do nothing for now.
            Robot(bool const& testMode, bool const& disableLidar);

            /// \brief The low-level thread of the robot.
            /// \details This thread runs a periodic loop. At each iteration, it updates sensor values,
            ///          estimates the position of the robot on the table, and performs motor servoing.
            ///          It also logs everything in a log file.
            void lowLevelLoop() override;

            /// \brief Set a new target to the rail.
            ///
            /// \param position Relative rail position, form 0 (down) to 1 (up).
            /// \param wait If set, this function blocks until motion is complete.
            void moveRail(double const& position) override;

            /// \brief Get current rail position.
            ///
            /// \return Current relative rail position, form 0 (down) to 1 (up).
            double getRailPosition();

            /// \brief Update the robot score.
            /// \details This function increments the score then updates the display accordingly.
            ///
            /// \param[in] scoreIncrement Amount to increment the score, both positive or negative.
            void updateScore(int const& scoreIncrement) override;

            void stopMotors() override;


            bool isPlayingRightSide() const override
            {
                return isPlayingRightSide_;
            }

            /// \brief Get time in current match.
            /// \return Time since start of the match, or 0 if not started.
            double getMatchTime();

            ExcavationSquareColor getExcavationReadings(bool readRightSide) override;

            bool getTestMode() const override
            {
                return testMode_;
            }

            double getRangeSensorMeasurement(bool measureRightSide) const override
            {
                return rangeMeasurements_[measureRightSide ? RIGHT : LEFT];
            }

            /// \brief Shut down the robot when Ctrl+X is pressed.
            void shutdown();

        private:
            USBLCD screen_; ///< LCD screen and buttons.
            RPLidarHandler lidar_; ///< Lidar
            MaestroDriver maestro_; ///< Servo driver
            ServoHandler handler_;

            bool testMode_; // Test mode: no initial wait.
            bool disableLidar_; // Disable lidar (works only in test mode)


            /// \brief Initialize every system of the robot.
            /// \details This function tries to initialize every component of the robot, by creating the software
            ///          object and, when applicable, testing the connection with the real component.
            ///          This function can be called several times, and only tries to re-initialize a component
            ///          if previous initializations failed.
            /// \return true if all components were initialized successfully, false otherwise.
            bool initSystem();

            /// \brief Perform robot setup, return wheather the match has started or not.
            ///
            /// \details This function is called periodically before the match starts. It is responsible for
            ///          updating the display and status according to user input. It returns true whenever the match
            ///          has started.
            bool setupBeforeMatchStart();

            /// \brief Move the rail to the target position.
            /// \details This function computes the velocity to be applied to the rail servo in order to reach
            /// the desired target rail position using the potentiometer data.
            /// \param[in] dt Time since last servoing call, for PID controller.
            void updateMoveRail(double const& dt);

            void calibrateRail();
            int railHigh_;

            miam::L6470 stepperMotors_; ///< Robot driving motors.

            uCData microcontrollerData_; ///< Data structure containing informations from the arduino board.

            // Rail PID
            miam::PID PIDRail_; ///< PID for the rail.
            int targetRailPosition_; ///< The desired rail position (in potentiometer unit). Should be -1 if not yet set during the match.


            // Init variables.
            bool isScreenInit_ = {false}; ///< Boolean representing the initialization of the screen motors.
            bool isStepperInit_ = {false}; ///< Boolean representing the initialization of the stepper motors.
            bool isServosInit_ = {false}; ///< Boolean representing the initialization of the servo driving board.
            bool isArduinoInit_ = {false}; ///< Boolean representing the initialization of the slave arduino board.
            bool isLidarInit_ = {false}; ///< Boolean representing the initialization of the lidar.
            bool isRangeSensorInit_[2] = {false, false}; ///< Initialization of range sensors.
            int score_={0}; ///< Current robot score.
            std::mutex mutex_; ///< Mutex, for thread safety.

            startupstatus startupStatus_; ///< Current startup status.
            bool initMotorBlocked_; ///< State of the motors during init.
            bool initStatueHigh_; ///< State of the motors during init.


            VL53L0X rangeSensors_[2];

            double rangeMeasurements_[2] = {0, 0};
            void updateRangeMeasurement(); // Range measurement thread

            Strategy strategy_;

            double timeSinceLastCheckOnRailHeightDuringInit_;

            bool hasMatchStarted_{false};
            bool isPlayingRightSide_{false};
            double matchStartTime_{0.0};
            double currentTime_{0.0};
    };

    extern Robot robot;    ///< The robot instance, representing the current robot.
 #endif
