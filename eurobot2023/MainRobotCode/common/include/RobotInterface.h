/// \file RobotInterface.h
/// \brief Interface for 2021 robot - for use on the robot and in simulation
/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3

#ifndef ROBOT_INTERFACE_H
     #define ROBOT_INTERFACE_H

    #include <miam_utils/AbstractRobot.h>
    #include <unistd.h>
    #include "Parameters.h"
    #include "MotionController.h"
    #include "ServoHandler.h"

    class RobotInterface
    {
        public:
            RobotInterface(ServoHandler const& servos):
                motionController_(),
                servos_(servos)
            { }

            /// \brief Set a new target to the rail.
            /// \param position Relative rail position, form 0 (down) to 1 (up).
            virtual void moveRail(double const& position) = 0;

            // Sleep a specified number of seconds.
            virtual void wait(double const& waitTimeS)
            {
                usleep(static_cast<int>(1e6 * waitTimeS));
            }

            virtual bool isPlayingRightSide() const
            {
                return false;
            }

            virtual ExcavationSquareColor getExcavationReadings(bool readRightSide)
            {
                return ExcavationSquareColor::RED;
            }

            virtual bool getTestMode() const
            {
                return false;
            }

            // Return the measured distance, from robot center, given by the range sensor, in mm.
            virtual double getRangeSensorMeasurement(bool measureRightSide) const
            {
                return 0;
            }

            /// \brief The low-level thread of the robot.
            /// \details This thread runs a periodic loop. At each iteration, it updates sensor values,
            ///          estimates the position of the robot on the table, and performs motor servoing.
            ///          It also logs everything in a log file.
            virtual void lowLevelLoop() = 0;

            /// \brief Update the robot score.
            /// \details This function increments the score then updates the display accordingly.
            ///
            /// \param[in] scoreIncrement Amount to increment the score, both positive or negative.
            virtual void updateScore(int const& scoreIncrement) = 0;

            /// \brief Get time in current match.
            /// \return Time since start of the match, or 0 if not started.
            virtual double getMatchTime() = 0;

            /// \brief Stop the wheel motors.
            virtual void stopMotors() = 0;

            MotionController* getMotionController() { return &motionController_;}
            ServoHandler* getServos() { return &servos_;}

        protected:
            MotionController motionController_;
            ServoHandler servos_;
    };
 #endif
