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
    #include <miam_utils/raspberry_pi/RaspberryPi.h>
    #include <miam_utils/drivers/SPI-Wrapper.h>
    #include <miam_utils/drivers/RMDX.h>
    #include <miam_utils/drivers/AS5045Driver.h>
    #include <miam_utils/trajectory/PointTurn.h>
    #include <miam_utils/trajectory/Utilities.h>
    #include <miam_utils/trajectory/DrivetrainKinematics.h>
    #include <miam_utils/RPLidarHandler.h>


    #include <math.h>
    #include <stdlib.h>
    #include <stdio.h>

    #include <memory>
    #include <vector>
    #include <mutex>

    #include "common/RobotInterface.h"
    #include "common/AbstractStrategy.h"

    // Right and left macros, for array addressing.
    using miam::RobotPosition;
    using miam::ProtectedPosition;
    using miam::trajectory::Trajectory;

    class RobotGUI;

    // Controller parameters
    namespace controller
    {
        double const railKp = 20.0;
        double const railKd = 0.0;
        double const railKi = 0.0;
    }

    class RailMeasurements
    {
        public:
            RailMeasurements();
            int turn_counts;
            int encoder_counts;
            bool init; // was the rail read once?

            double getTurnCounts();
    };

    class Robot : public RobotInterface
    {
        public:

            /// \brief Constructor: do nothing for now.
            Robot(RobotParameters const& parameters, AbstractStrategy *strategy, RobotGUI *gui, bool const& testMode, bool const& disableLidar);

            /// \brief The low-level thread of the robot.
            /// \details This thread runs a periodic loop. At each iteration, it updates sensor values,
            ///          estimates the position of the robot on the table, and performs motor servoing.
            ///          It also logs everything in a log file.
            void lowLevelLoop();

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


            bool getTestMode() const override
            {
                return testMode_;
            }

            /// \brief Shut down the robot when Ctrl+X is pressed.
            void shutdown();

        private:

            RobotGUI *gui_;
            RPLidarHandler lidar_; ///< Lidar
            bool isLidarInit_ = false; ///< Boolean representing the initialization of the lidar.

            bool isServoInit_ = false;

            RobotGUIData guiState_;

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

            // miam::PID PIDRail_;
            void calibrateRail();
            void updateRailHeight();
            void moveRail(int railHeight);
            RailMeasurements currentRailMeasurements;
            double highTurnCount;
            double lowTurnCount;

            SPIWrapper spiMotor_;
            MCP2515 mcp_;
            RMDX motors_;
            SPIWrapper spiEncoder_;
            AS5045 encoders_;

            bool isMCPInit_ = false;
            bool isMotorsInit_ = false;
            bool isEncodersInit_ = false;

            std::vector<double> lastEncoderPosition_;

            // Init variables.
            bool isServosInit_ = {false}; ///< Boolean representing the initialization of the servo driving board.
            std::mutex mutex_; ///< Mutex, for thread safety.

            AbstractStrategy *strategy_;

            bool hasMatchStarted_{false};
            bool isPlayingRightSide_{false};
            double matchStartTime_{0.0};
            double currentTime_{0.0};
    };
 #endif
