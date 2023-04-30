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
    #include "common/RobotParameters.h"
    #include "common/MotionController.h"
    #include "common/ServoHandler.h"
    #include "common/Types.h"
    #include <miam_utils/drivers/STSServoDriver.h>

    class RobotInterface
    {
        public:
            RobotInterface(RobotParameters const& robotParameters):
                motionController_(robotParameters),
                servos_()
            {
            }

            RobotParameters getParameters()
            {
                return motionController_.robotParams_;
            }

            // Sleep a specified number of seconds.
            virtual void wait(double const& waitTimeS)
            {
                usleep(static_cast<int>(1e6 * waitTimeS));
            }

            virtual bool isPlayingRightSide() const
            {
                return false;
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


            /// \brief Helper to log a variable at the current (match) time.
            /// \param[in] variableName Name of the variable
            /// \param[in] variableValue Value of the variable at the current time
            void log(std::string const& variableName, double const& variableValue)
            {
                motionController_.log(variableName, variableValue);
            }

            MotionController* getMotionController() { return &motionController_;}
            STSServoDriver* getServos() { return &servos_;}

        protected:
            MotionController motionController_;
            STSServoDriver servos_;
    };
 #endif
